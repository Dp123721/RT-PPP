// Configures Qt startup, command-line modes, logging, and orderly application shutdown.
#include "mainwindow.h"

#include <QAccessible>
#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QDateTime>
#include <QFileInfo>
#include <QMessageLogContext>
#include <QTimer>

#include <csignal>
#include <cstdio>

#ifdef Q_OS_WIN
#include <windows.h>
#endif

namespace {

volatile std::sig_atomic_t gShutdownRequested = 0;

extern "C" void requestShutdownFromSignal(int) {
    gShutdownRequested = 1;
}

#ifdef Q_OS_WIN
BOOL WINAPI consoleControlHandler(DWORD controlType) {
    switch (controlType) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        gShutdownRequested = 1;
        return TRUE;
    default:
        return FALSE;
    }
}
#endif

void rtPppMessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg) {
    const char* level = "INFO";
    switch (type) {
    case QtDebugMsg: level = "DEBUG"; break;
    case QtInfoMsg: level = "INFO"; break;
    case QtWarningMsg: level = "WARN"; break;
    case QtCriticalMsg: level = "CRITICAL"; break;
    case QtFatalMsg: level = "FATAL"; break;
    }
    QString line = QStringLiteral("%1 [%2] %3")
                       .arg(QDateTime::currentDateTime().toString(Qt::ISODateWithMs),
                            QString::fromLatin1(level), msg);
    if (context.file) {
        line += QStringLiteral(" (%1:%2, %3)")
                    .arg(QString::fromLocal8Bit(context.file))
                    .arg(context.line)
                    .arg(QString::fromLocal8Bit(context.function));
    }
    const QByteArray encoded = line.toLocal8Bit();
    std::fprintf(stderr, "%s\n", encoded.constData());
    std::fflush(stderr);
#ifdef Q_OS_WIN
    const std::wstring debuggerLine = (line + QLatin1Char('\n')).toStdWString();
    OutputDebugStringW(debuggerLine.c_str());
#endif

    if (type == QtFatalMsg) {
        abort();
    }
}

} // namespace

int main(int argc, char *argv[])
{
    qInstallMessageHandler(rtPppMessageHandler);
    bool requestedHeadless = false;
    for (int index = 1; index < argc; ++index) {
        const QString argument = QString::fromLocal8Bit(argv[index]);
        if (argument == QLatin1String("--headless") || argument == QLatin1String("-H")) {
            requestedHeadless = true;
            break;
        }
    }
    if (requestedHeadless) {
#ifdef Q_OS_UNIX
        // Linux services need a platform plugin even when no display server exists.
        qputenv("QT_QPA_PLATFORM", "offscreen");
#endif
    }
    qputenv("QT_ACCESSIBILITY", "0");
    QAccessible::setActive(false);
    QApplication a(argc, argv);
    QAccessible::setActive(false);
    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("RT-PPP realtime PPP client"));
    parser.addHelpOption();
    const QCommandLineOption headlessOption(
        QStringList() << QStringLiteral("H") << QStringLiteral("headless"),
        QStringLiteral("Run saved realtime streams without displaying the GUI."));
    const QCommandLineOption configOption(
        QStringList() << QStringLiteral("c") << QStringLiteral("config"),
        QStringLiteral("Read configuration from <file> instead of rtppp_settings.ini."),
        QStringLiteral("file"));
    const QCommandLineOption runSecondsOption(
        QStringList() << QStringLiteral("run-seconds"),
        QStringLiteral("Exit cleanly after <seconds>; useful for service probes."),
        QStringLiteral("seconds"));
    parser.addOption(headlessOption);
    parser.addOption(configOption);
    parser.addOption(runSecondsOption);
    parser.process(a);
    const bool headless = parser.isSet(headlessOption);
    const QString configPath = parser.isSet(configOption)
        ? QFileInfo(parser.value(configOption)).absoluteFilePath()
        : QString();
    if (!configPath.isEmpty() && !QFileInfo::exists(configPath)) {
        qCritical().noquote() << QStringLiteral("Configuration file does not exist: %1").arg(configPath);
        return 2;
    }
    MainWindow w(configPath);
    if (headless) {
        QString error;
        if (!w.startHeadless(&error)) {
            return 3;
        }
    } else {
        w.show();
    }

    std::signal(SIGINT, requestShutdownFromSignal);
#ifdef SIGTERM
    std::signal(SIGTERM, requestShutdownFromSignal);
#endif
#ifdef Q_OS_WIN
    SetConsoleCtrlHandler(consoleControlHandler, TRUE);
#endif
    QTimer shutdownPoll;
    shutdownPoll.setInterval(200);
    QObject::connect(&shutdownPoll, &QTimer::timeout, &a, [&a]() {
        if (gShutdownRequested) {
            qInfo().noquote() << QStringLiteral("RT-PPP shutdown signal received; stopping event loop.");
            a.quit();
        }
    });
    shutdownPoll.start();

    bool runSecondsOk = false;
    const qint64 runSeconds = parser.isSet(runSecondsOption)
        ? parser.value(runSecondsOption).toLongLong(&runSecondsOk)
        : 0;
    if (parser.isSet(runSecondsOption) &&
        (!runSecondsOk || runSeconds <= 0 || runSeconds > 2147483)) {
        qCritical().noquote() << QStringLiteral("--run-seconds must be between 1 and 2147483");
        return 2;
    }
    if (runSeconds > 0) {
        QTimer::singleShot(static_cast<int>(runSeconds * 1000), &a, [&a]() {
            qInfo().noquote() << QStringLiteral("RT-PPP --run-seconds limit reached; stopping event loop.");
            a.quit();
        });
    }
    bool smokeExitOk = false;
    const int smokeExitMs = qEnvironmentVariableIntValue("RTPPP_SMOKE_EXIT_MS", &smokeExitOk);
    if (smokeExitOk && smokeExitMs > 0) {
        QTimer::singleShot(qBound(1, smokeExitMs, 60000), &a, &QCoreApplication::quit);
    }
    return a.exec();
}
