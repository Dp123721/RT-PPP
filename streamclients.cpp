// Implements NTRIP, TCP, and MQTT connections, reconnects, and byte delivery.
#include "streamclients.h"

#include <QRandomGenerator>
#include <QUrl>

namespace {

QString toShortSocketError(QAbstractSocket::SocketError error, const QString& errorString) {
    return QString("Socket error (%1): %2").arg(static_cast<int>(error)).arg(errorString);
}

QByteArray rtpppNtripUserAgent() {
    return QByteArrayLiteral("NTRIP RT-PPP/1.0");
}

QByteArray decodedLatin1(const QString& text) {
    QByteArray out = text.trimmed().toLatin1();
    for (int i = 0; i < 2 && out.contains('%'); ++i) {
        const QByteArray decoded = QUrl::fromPercentEncoding(out).toLatin1();
        if (decoded == out) {
            break;
        }
        out = decoded;
    }
    return out;
}

QByteArray normalizedGgaLatin1(const QString& text) {
    const QString gga = text.trimmed();
    if (gga.isEmpty() ||
        gga.compare(QStringLiteral("no"), Qt::CaseInsensitive) == 0 ||
        gga.compare(QStringLiteral("none"), Qt::CaseInsensitive) == 0 ||
        gga.compare(QStringLiteral("false"), Qt::CaseInsensitive) == 0 ||
        gga == QStringLiteral("0")) {
        return QByteArray();
    }
    if (!gga.startsWith('$')) {
        return QByteArray();
    }
    return gga.toLatin1();
}

int ntripHeaderEnd(const QByteArray& data, int* consume) {
    int pos = data.indexOf("\r\n\r\n");
    if (pos >= 0) {
        if (consume) {
            *consume = 4;
        }
        return pos;
    }
    pos = data.indexOf("\n\n");
    if (pos >= 0) {
        if (consume) {
            *consume = 2;
        }
        return pos;
    }
    return -1;
}

bool isAsciiHeaderLine(const QByteArray& line) {
    if (line.trimmed().isEmpty() || !line.contains(':')) {
        return false;
    }
    for (char ch : line) {
        const uchar b = static_cast<uchar>(ch);
        if (b == '\r' || b == '\n' || b == '\t') {
            continue;
        }
        if (b < 0x20 || b >= 0x7f) {
            return false;
        }
    }
    return true;
}

QByteArray takeOptionalPostStatusLine(const QByteArray& data, int* cursor) {
    if (!cursor || *cursor >= data.size()) {
        return QByteArray();
    }

    if (data.mid(*cursor, 2) == "\r\n") {
        *cursor += 2;
        return QByteArrayLiteral("\r\n");
    }
    if (data.at(*cursor) == '\n') {
        *cursor += 1;
        return QByteArrayLiteral("\n");
    }

    const int lineEnd = data.indexOf('\n', *cursor);
    if (lineEnd < 0) {
        return QByteArray();
    }
    const QByteArray line = data.mid(*cursor, lineEnd - *cursor + 1);
    if (!isAsciiHeaderLine(line)) {
        return QByteArray();
    }
    *cursor = lineEnd + 1;
    return line;
}

} // namespace

BaseStreamClient::BaseStreamClient(const QString& sourceName, QObject* parent)
    : QObject(parent), _sourceName(sourceName) {
    _state.stateText = "Stopped";
}

BaseStreamClient::~BaseStreamClient() = default;

StreamRuntimeState BaseStreamClient::runtimeState() const {
    return _state;
}

QString BaseStreamClient::sourceName() const {
    return _sourceName;
}

void BaseStreamClient::setStateText(const QString& text) {
    if (_state.stateText == text) {
        return;
    }
    _state.stateText = text;
    emit stateChanged();
}

void BaseStreamClient::setRunning(bool running) {
    if (_state.running == running) {
        return;
    }
    _state.running = running;
    emit stateChanged();
}

void BaseStreamClient::setLastError(const QString& errorText) {
    _state.lastError = errorText;
    emit stateChanged();
}

void BaseStreamClient::clearLastError() {
    if (_state.lastError.isEmpty()) {
        return;
    }
    _state.lastError.clear();
    emit stateChanged();
}

void BaseStreamClient::onDataReceived(const QByteArray& payload) {
    if (!_state.running || payload.isEmpty()) {
        return;
    }
    _state.bytes += static_cast<quint64>(payload.size());
    _state.frames += 1;
    _state.lastDataUtc = QDateTime::currentDateTimeUtc();
    emit stateChanged();
    emit dataFrame(_sourceName, payload.size(), payload);
}

void BaseStreamClient::emitLog(const QString& message) {
    emit logMessage(_sourceName, message);
}

NtripStreamClient::NtripStreamClient(QObject* parent, const QString& sourceName)
    : BaseStreamClient(sourceName, parent), _socket(new QSslSocket(this)) {
    connect(_socket, &QTcpSocket::connected, this, &NtripStreamClient::onSocketConnected);
    connect(_socket, &QTcpSocket::readyRead, this, &NtripStreamClient::onSocketReadyRead);
    connect(_socket,
            QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error),
            this,
            &NtripStreamClient::onSocketError);
    connect(_socket, &QTcpSocket::disconnected, this, &NtripStreamClient::onSocketDisconnected);
    connect(_socket, &QSslSocket::encrypted, this, &NtripStreamClient::onSocketEncrypted);
    connect(_socket,
            QOverload<const QList<QSslError>&>::of(&QSslSocket::sslErrors),
            this,
            &NtripStreamClient::onSslErrors);

    _reconnectTimer.setSingleShot(true);
    connect(&_reconnectTimer, &QTimer::timeout, this, &NtripStreamClient::reconnect);

    // A live TCP socket can remain connected while the caster has stopped
    // forwarding data. Poll the last payload time to recover that silent
    // failure instead of leaving the stream falsely marked online.
    _inactivityTimer.setInterval(5000);
    connect(&_inactivityTimer, &QTimer::timeout,
            this, &NtripStreamClient::onInactivityTimeout);
}

NtripStreamClient::~NtripStreamClient() = default;

void NtripStreamClient::setConfig(const NtripStreamConfig& config) {
    _config = config;
}

NtripStreamConfig NtripStreamClient::config() const {
    return _config;
}

void NtripStreamClient::start() {
    if (_state.running) {
        return;
    }
    setRunning(true);
    clearLastError();
    _chunkedTransfer = false;
    _chunkBytesRemaining = -1;
    _usingV1Fallback = false;
    _triedV1Fallback = false;
    _streamStartedUtc = QDateTime();
    _inactivityTimer.stop();
    connectNow();
}

void NtripStreamClient::stop() {
    _reconnectTimer.stop();
    _inactivityTimer.stop();
    setRunning(false);
    _rxBuffer.clear();
    _chunkedTransfer = false;
    _chunkBytesRemaining = -1;
    _usingV1Fallback = false;
    _triedV1Fallback = false;
    _streamStartedUtc = QDateTime();
    _socket->abort();
    setStateText("Stopped");
}

void NtripStreamClient::connectNow() {
    if (_config.host.trimmed().isEmpty() || _config.mountPoint.trimmed().isEmpty() || _config.port <= 0) {
        setLastError("Missing host/port/mount point");
        setStateText("Config Error");
        setRunning(false);
        emitLog("Cannot start: missing host/port/mount point");
        return;
    }
    _headerParsed = false;
    _rxBuffer.clear();
    _chunkedTransfer = false;
    _chunkBytesRemaining = -1;
    _streamStartedUtc = QDateTime::currentDateTimeUtc();
    _inactivityTimer.stop();
    setStateText("Connecting");
    emitLog(QString("Connecting to %1:%2/%3").arg(_config.host).arg(_config.port).arg(_config.mountPoint));
    _socket->abort();
    const bool secure = (_config.ntripVersion.trimmed().toLower() == "2s");
    if (secure) {
        setStateText("Encrypting");
        _socket->connectToHostEncrypted(_config.host, static_cast<quint16>(_config.port));
    } else {
        _socket->connectToHost(_config.host, static_cast<quint16>(_config.port));
    }
}

QByteArray NtripStreamClient::buildRequest() const {
    QString mountPath = _config.mountPoint.trimmed();
    if (!mountPath.startsWith('/')) {
        mountPath.prepend('/');
    }
    const QByteArray hostLatin1 = _config.host.trimmed().toLatin1();
    const QByteArray mountLatin1 = mountPath.toLatin1();
    const QByteArray gga = normalizedGgaLatin1(_config.nmea);
    const QString version = _usingV1Fallback
                                ? QStringLiteral("1")
                                : _config.ntripVersion.trimmed().toLower();
    const bool v2 = (version == "2" || version == "2s");
    QByteArray authLine;
    if (!_config.user.isEmpty() || !_config.password.isEmpty()) {
        const QByteArray credentials = (decodedLatin1(_config.user) + ":" + decodedLatin1(_config.password)).toBase64();
        authLine = "Authorization: Basic " + credentials + "\r\n";
    }

    QByteArray req;
    if (!v2) {
        req += "GET " + mountLatin1 + " HTTP/1.0\r\n";
        req += "User-Agent: " + rtpppNtripUserAgent() + "\r\n";
        req += "Host: " + hostLatin1 + "\r\n";
        req += authLine;
        req += "\r\n";
        if (!gga.isEmpty()) {
            req += gga + "\r\n";
        }
        return req;
    }

    req += "GET " + mountLatin1 + " HTTP/1.1\r\n";
    req += "Host: " + hostLatin1 + "\r\n";
    if (v2) {
        req += "Ntrip-Version: Ntrip/2.0\r\n";
    }
    req += "User-Agent: " + rtpppNtripUserAgent() + "\r\n";
    req += authLine;
    if (!gga.isEmpty()) {
        req += "Ntrip-GGA: " + gga + "\r\n";
    }
    req += "Connection: close\r\n";

    req += "\r\n";
    return req;
}

bool NtripStreamClient::parseResponseHeader(const QByteArray& headerBlock) {
    const QList<QByteArray> lines = headerBlock.split('\n');
    if (lines.isEmpty()) {
        setLastError("Empty response header");
        return false;
    }
    const QByteArray statusLine = lines.first().trimmed();
    const bool ok = (statusLine.startsWith("ICY 200") ||
                     (statusLine.startsWith("HTTP/") && statusLine.contains("200 OK"))) &&
                    !statusLine.contains("SOURCETABLE");
    emitLog(QString("Caster response: %1").arg(QString::fromUtf8(statusLine)));
    if (!ok) {
        setLastError(QString("Caster response not OK: %1").arg(QString::fromUtf8(statusLine)));
        return false;
    }
    const QByteArray lowerHeader = headerBlock.toLower();
    _chunkedTransfer = lowerHeader.contains("transfer-encoding:") &&
                       lowerHeader.contains("chunked");
    if (_chunkedTransfer) {
        _chunkBytesRemaining = -1;
        emitLog("NTRIP response uses HTTP chunked transfer; decoding chunks before RTCM parsing.");
    }
    return true;
}

bool NtripStreamClient::tryParseV1Response(bool finalAttempt) {
    bool proxyResponse = false;
    int cursor = 0;
    int responseCount = 0;
    QByteArray responseText;
    QByteArray headerLog;

    while (true) {
        int lineEnd = _rxBuffer.indexOf('\n', cursor);
        if (lineEnd < 0) {
            if (!finalAttempt || cursor >= _rxBuffer.size()) {
                return false;
            }
            lineEnd = _rxBuffer.size() - 1;
        }

        const int lineLen = lineEnd - cursor + 1;
        const QByteArray line = _rxBuffer.mid(cursor, lineLen);
        cursor += lineLen;
        headerLog += line;

        if (line.indexOf("ICY 200 OK") == -1 &&
            line.indexOf("HTTP") != -1 &&
            line.indexOf("200 OK") != -1) {
            proxyResponse = true;
        }

        if (!proxyResponse && !line.trimmed().isEmpty()) {
            responseText += line;
            ++responseCount;
        }

        bool done = false;
        if (line.trimmed().isEmpty()) {
            if (proxyResponse) {
                proxyResponse = false;
            } else {
                done = true;
            }
        }

        if (line.indexOf("Unauthorized") != -1) {
            done = true;
        }

        if (!proxyResponse &&
            line.indexOf("200 OK") != -1 &&
            line.indexOf("SOURCETABLE") == -1) {
            responseText.clear();
            responseCount = 0;
            const QByteArray extraLine = takeOptionalPostStatusLine(_rxBuffer, &cursor);
            if (!extraLine.isEmpty()) {
                headerLog += extraLine;
            }
            done = true;
        }

        if (done) {
            break;
        }
    }

    _rxBuffer.remove(0, cursor);
    emitLog(QString("NTRIP response header:\n%1").arg(QString::fromUtf8(headerLog.trimmed())));

    if (responseCount > 0) {
        const QString response = QString::fromUtf8(responseText).trimmed();
        setLastError(QString("Wrong caster response\n%1").arg(response));
        emitLog(QString("Wrong caster response\n%1").arg(response));
        return true;
    }

    _headerParsed = true;
    _streamStartedUtc = QDateTime::currentDateTimeUtc();
    _inactivityTimer.start();
    setStateText("Streaming");
    emitLog("NTRIP stream started");
    return true;
}

bool NtripStreamClient::tryParseBufferedHttpResponse(bool finalAttempt) {
    if (_headerParsed || _rxBuffer.isEmpty()) {
        return false;
    }

    const QString version = _usingV1Fallback
                                ? QStringLiteral("1")
                                : _config.ntripVersion.trimmed().toLower();
    if (version == "1" || _rxBuffer.startsWith("ICY ")) {
        return tryParseV1Response(finalAttempt);
    }

    int consume = 0;
    int headerEnd = ntripHeaderEnd(_rxBuffer, &consume);
    if (headerEnd >= 0) {
        const QByteArray headerBlock = _rxBuffer.left(headerEnd);
        _rxBuffer.remove(0, headerEnd + consume);

        emitLog(QString("NTRIP response header:\n%1").arg(QString::fromUtf8(headerBlock)));
        const bool ok = parseResponseHeader(headerBlock);
        if (ok) {
            _headerParsed = true;
            _streamStartedUtc = QDateTime::currentDateTimeUtc();
            _inactivityTimer.start();
            setStateText("Streaming");
            emitLog("NTRIP stream started");
        }
        return true;
    }

    if (!finalAttempt) {
        return false;
    }

    const QByteArray trimmed = _rxBuffer.trimmed();
    if (!(trimmed.startsWith("HTTP/") || trimmed.startsWith("ICY "))) {
        return false;
    }
    headerEnd = _rxBuffer.size();
    const QByteArray headerBlock = _rxBuffer.left(headerEnd);
    _rxBuffer.clear();

    emitLog(QString("NTRIP response header:\n%1").arg(QString::fromUtf8(headerBlock)));
    const bool ok = parseResponseHeader(headerBlock);
    if (ok) {
        _headerParsed = true;
        _streamStartedUtc = QDateTime::currentDateTimeUtc();
        _inactivityTimer.start();
        setStateText("Streaming");
        emitLog("NTRIP stream started");
    }
    return true;
}

bool NtripStreamClient::canRetryWithV1() const {
    if (_triedV1Fallback || _usingV1Fallback || _headerParsed) {
        return false;
    }
    const QString version = _config.ntripVersion.trimmed().toLower();
    return version == "2" && !_config.host.trimmed().isEmpty() && _config.port > 0;
}

void NtripStreamClient::retryWithV1() {
    _triedV1Fallback = true;
    _usingV1Fallback = true;
    _headerParsed = false;
    _rxBuffer.clear();
    _chunkedTransfer = false;
    _chunkBytesRemaining = -1;
    _streamStartedUtc = QDateTime();
    _inactivityTimer.stop();
    clearLastError();
    setStateText("Retry NTRIP v1");
    emitLog("Remote closed before a valid NTRIP response; retrying with NTRIP v1 HTTP/1.0 request.");
    _socket->abort();
    _socket->connectToHost(_config.host, static_cast<quint16>(_config.port));
}

void NtripStreamClient::onSocketConnected() {
    const bool secure = (!_usingV1Fallback && _config.ntripVersion.trimmed().toLower() == "2s");
    if (secure) {
        setStateText("Encrypting");
        return;
    }
    clearLastError();
    setStateText("Authorizing");
    const QByteArray req = buildRequest();
    emitLog(QString("NTRIP request path=%1 version=%2 auth=%3 ua=%4 hostHeader=%5")
                .arg(_config.mountPoint.trimmed(),
                     _usingV1Fallback ? QStringLiteral("1") : _config.ntripVersion.trimmed(),
                     _config.user.trimmed().isEmpty() ? QStringLiteral("no") : QStringLiteral("yes"),
                     QString::fromUtf8(rtpppNtripUserAgent()),
                     _config.host.trimmed()));
    _socket->write(req);
    _socket->flush();
}

void NtripStreamClient::onSocketEncrypted() {
    clearLastError();
    setStateText("Authorizing");
    const QByteArray req = buildRequest();
    emitLog(QString("NTRIP request path=%1 version=%2 auth=%3 ua=%4 hostHeader=%5")
                .arg(_config.mountPoint.trimmed(),
                     _usingV1Fallback ? QStringLiteral("1") : _config.ntripVersion.trimmed(),
                     _config.user.trimmed().isEmpty() ? QStringLiteral("no") : QStringLiteral("yes"),
                     QString::fromUtf8(rtpppNtripUserAgent()),
                     _config.host.trimmed()));
    _socket->write(req);
    _socket->flush();
}

void NtripStreamClient::onSslErrors(const QList<QSslError>& errors) {
    if (errors.isEmpty()) {
        return;
    }
    emitLog(QString("SSL warnings: %1 (ignored)").arg(errors.first().errorString()));
    _socket->ignoreSslErrors();
}

QByteArray NtripStreamClient::takeChunkedPayload() {
    QByteArray payload;

    while (true) {
        if (_chunkBytesRemaining < 0) {
            const int lineEnd = _rxBuffer.indexOf('\n');
            if (lineEnd < 0) {
                break;
            }

            QByteArray sizeLine = _rxBuffer.left(lineEnd).trimmed();
            _rxBuffer.remove(0, lineEnd + 1);
            const int semicolon = sizeLine.indexOf(';');
            if (semicolon >= 0) {
                sizeLine = sizeLine.left(semicolon).trimmed();
            }

            bool ok = false;
            const int chunkSize = QString::fromLatin1(sizeLine).toInt(&ok, 16);
            if (!ok || chunkSize < 0) {
                setLastError(QString("Invalid HTTP chunk size before RTCM payload: %1")
                                 .arg(QString::fromLatin1(sizeLine)));
                emitLog(_state.lastError);
                _rxBuffer.clear();
                break;
            }
            if (chunkSize == 0) {
                _rxBuffer.clear();
                break;
            }
            _chunkBytesRemaining = chunkSize;
        }

        if (_rxBuffer.size() < _chunkBytesRemaining) {
            break;
        }
        if (_rxBuffer.size() == _chunkBytesRemaining) {
            break;
        }
        if (_rxBuffer.at(_chunkBytesRemaining) == '\r' &&
            _rxBuffer.size() < _chunkBytesRemaining + 2) {
            break;
        }
        if (_rxBuffer.mid(_chunkBytesRemaining, 2) == "\r\n") {
            payload += _rxBuffer.left(_chunkBytesRemaining);
            _rxBuffer.remove(0, _chunkBytesRemaining + 2);
            _chunkBytesRemaining = -1;
            continue;
        }
        if (_rxBuffer.at(_chunkBytesRemaining) == '\n') {
            payload += _rxBuffer.left(_chunkBytesRemaining);
            _rxBuffer.remove(0, _chunkBytesRemaining + 1);
            _chunkBytesRemaining = -1;
            continue;
        }

        payload += _rxBuffer.left(_chunkBytesRemaining);
        _rxBuffer.remove(0, _chunkBytesRemaining);
        _chunkBytesRemaining = -1;

        if (_rxBuffer.startsWith("\r\n")) {
            _rxBuffer.remove(0, 2);
        } else if (_rxBuffer.startsWith('\n')) {
            _rxBuffer.remove(0, 1);
        } else {
            break;
        }
    }

    return payload;
}

void NtripStreamClient::onSocketReadyRead() {
    if (!_state.running) {
        _socket->readAll();
        _rxBuffer.clear();
        return;
    }
    _rxBuffer += _socket->readAll();

    if (!_headerParsed) {
        if (!tryParseBufferedHttpResponse(false)) {
            return;
        }
        if (!_headerParsed) {
            emitLog("Invalid NTRIP response. Closing socket.");
            _socket->disconnectFromHost();
            return;
        }
    }

    if (_headerParsed && !_rxBuffer.isEmpty()) {
        const QByteArray payload = _chunkedTransfer ? takeChunkedPayload() : _rxBuffer;
        if (!_chunkedTransfer) {
            _rxBuffer.clear();
        }
        if (payload.isEmpty()) {
            return;
        }
        onDataReceived(payload);
    }
}

void NtripStreamClient::onSocketError(QAbstractSocket::SocketError socketError) {
    if (socketError == QAbstractSocket::RemoteHostClosedError && canRetryWithV1()) {
        emitLog("Socket closed by caster before streaming; retrying with the NTRIP v1 request form.");
        return;
    }
    const QString message = toShortSocketError(socketError, _socket->errorString());
    setLastError(message);
    emitLog(message);
    scheduleReconnect(QStringLiteral("socket error"));
}

int NtripStreamClient::inactivityTimeoutSec() const {
    // SSR streams normally update every few seconds and broadcast streams
    // less frequently. Keep enough margin for short caster pauses while
    // still detecting the multi-hour silent-stream failure.
    const QString mount = _config.mountPoint.trimmed().toUpper();
    if (mount.contains(QStringLiteral("SSR"))) {
        return 120;
    }
    if (mount.contains(QStringLiteral("BCEP")) || mount.contains(QStringLiteral("BRDC"))) {
        return 180;
    }
    return 120;
}

void NtripStreamClient::onInactivityTimeout() {
    if (!_state.running || !_headerParsed || !_socket) {
        return;
    }

    const QDateTime now = QDateTime::currentDateTimeUtc();
    // lastDataUtc is cumulative across reconnects. Do not let a payload from
    // the previous socket make the newly established socket look healthy.
    QDateTime reference = _streamStartedUtc;
    if (_state.lastDataUtc.isValid() &&
        (!reference.isValid() || _state.lastDataUtc > reference)) {
        reference = _state.lastDataUtc;
    }
    if (!reference.isValid()) {
        return;
    }

    const qint64 idleSec = reference.secsTo(now);
    const int timeoutSec = inactivityTimeoutSec();
    if (idleSec < timeoutSec) {
        return;
    }

    const QString lastData = _state.lastDataUtc.isValid()
        ? _state.lastDataUtc.toString(Qt::ISODate)
        : QStringLiteral("never");
    const QString reason = QString("NTRIP data watchdog: no payload for %1 s (timeout=%2 s, lastDataUtc=%3); reconnecting")
                               .arg(idleSec)
                               .arg(timeoutSec)
                               .arg(lastData);
    setLastError(reason);
    emitLog(reason);
    _inactivityTimer.stop();
    _streamStartedUtc = QDateTime();
    _socket->abort();
    scheduleReconnect(QStringLiteral("data inactivity"));
}

void NtripStreamClient::scheduleReconnect(const QString& reason) {
    if (!_state.running) {
        return;
    }
    _inactivityTimer.stop();
    setStateText("Reconnecting");
    if (!_reconnectTimer.isActive()) {
        emitLog(QString("%1. Reconnect in %2 ms")
                    .arg(reason)
                    .arg(_config.reconnectMs));
        _reconnectTimer.start(qMax(100, _config.reconnectMs));
    }
}

void NtripStreamClient::onSocketDisconnected() {
    if (!_headerParsed) {
        if (tryParseBufferedHttpResponse(true) && !_headerParsed) {
            emitLog("NTRIP server closed after non-success HTTP response.");
        }
        if (canRetryWithV1()) {
            retryWithV1();
            return;
        }
    }
    if (!_state.running) {
        return;
    }
    _headerParsed = false;
    _streamStartedUtc = QDateTime();
    scheduleReconnect(QStringLiteral("Disconnected"));
}

void NtripStreamClient::reconnect() {
    if (_state.running) {
        connectNow();
    }
}

TcpStreamClient::TcpStreamClient(QObject* parent, const QString& sourceName)
    : BaseStreamClient(sourceName, parent), _socket(new QTcpSocket(this)) {
    connect(_socket, &QTcpSocket::connected, this, &TcpStreamClient::onSocketConnected);
    connect(_socket, &QTcpSocket::readyRead, this, &TcpStreamClient::onSocketReadyRead);
    connect(_socket,
            QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error),
            this,
            &TcpStreamClient::onSocketError);
    connect(_socket, &QTcpSocket::disconnected, this, &TcpStreamClient::onSocketDisconnected);

    _reconnectTimer.setSingleShot(true);
    connect(&_reconnectTimer, &QTimer::timeout, this, &TcpStreamClient::reconnect);
}

TcpStreamClient::~TcpStreamClient() = default;

void TcpStreamClient::setConfig(const TcpStreamConfig& config) {
    _config = config;
}

TcpStreamConfig TcpStreamClient::config() const {
    return _config;
}

void TcpStreamClient::start() {
    if (_state.running) {
        return;
    }
    setRunning(true);
    clearLastError();
    connectNow();
}

void TcpStreamClient::stop() {
    _reconnectTimer.stop();
    setRunning(false);
    _socket->abort();
    setStateText("Stopped");
}

void TcpStreamClient::connectNow() {
    if (_config.host.trimmed().isEmpty() || _config.port <= 0) {
        setLastError("Missing host/port");
        setStateText("Config Error");
        setRunning(false);
        emitLog("Cannot start: missing host/port");
        return;
    }
    setStateText("Connecting");
    emitLog(QString("Connecting to %1:%2").arg(_config.host).arg(_config.port));
    _socket->abort();
    _socket->connectToHost(_config.host, static_cast<quint16>(_config.port));
}

void TcpStreamClient::onSocketConnected() {
    clearLastError();
    setStateText("Streaming");
    emitLog("TCP stream connected");
}

void TcpStreamClient::onSocketReadyRead() {
    if (!_state.running) {
        _socket->readAll();
        return;
    }
    onDataReceived(_socket->readAll());
}

void TcpStreamClient::onSocketError(QAbstractSocket::SocketError socketError) {
    const QString message = toShortSocketError(socketError, _socket->errorString());
    setLastError(message);
    setStateText("Error");
    emitLog(message);
}

void TcpStreamClient::onSocketDisconnected() {
    if (!_state.running) {
        return;
    }
    setStateText("Reconnecting");
    emitLog(QString("Disconnected. Reconnect in %1 ms").arg(_config.reconnectMs));
    _reconnectTimer.start(qMax(100, _config.reconnectMs));
}

void TcpStreamClient::reconnect() {
    if (_state.running) {
        connectNow();
    }
}

MqttStreamClient::MqttStreamClient(QObject* parent, const QString& sourceName)
    : BaseStreamClient(sourceName, parent), _socket(new QTcpSocket(this)) {
    connect(_socket, &QTcpSocket::connected, this, &MqttStreamClient::onSocketConnected);
    connect(_socket, &QTcpSocket::readyRead, this, &MqttStreamClient::onSocketReadyRead);
    connect(_socket,
            QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error),
            this,
            &MqttStreamClient::onSocketError);
    connect(_socket, &QTcpSocket::disconnected, this, &MqttStreamClient::onSocketDisconnected);

    _reconnectTimer.setSingleShot(true);
    connect(&_reconnectTimer, &QTimer::timeout, this, &MqttStreamClient::reconnect);

    _pingTimer.setSingleShot(false);
    connect(&_pingTimer, &QTimer::timeout, this, &MqttStreamClient::sendPing);
}

MqttStreamClient::~MqttStreamClient() = default;

void MqttStreamClient::setConfig(const MqttStreamConfig& config) {
    _config = config;
}

MqttStreamConfig MqttStreamClient::config() const {
    return _config;
}

void MqttStreamClient::start() {
    if (_state.running) {
        return;
    }
    setRunning(true);
    clearLastError();
    connectNow();
}

void MqttStreamClient::stop() {
    _reconnectTimer.stop();
    _pingTimer.stop();
    setRunning(false);
    resetProtocolState();
    _socket->abort();
    setStateText("Stopped");
}

void MqttStreamClient::connectNow() {
    if (_config.host.trimmed().isEmpty() || _config.port <= 0 || _config.topic.trimmed().isEmpty()) {
        setLastError("Missing host/port/topic");
        setStateText("Config Error");
        setRunning(false);
        emitLog("Cannot start: missing host/port/topic");
        return;
    }
    resetProtocolState();
    setStateText("Connecting");
    emitLog(QString("Connecting to %1:%2 topic=%3").arg(_config.host).arg(_config.port).arg(_config.topic));
    _socket->abort();
    _socket->connectToHost(_config.host, static_cast<quint16>(_config.port));
}

void MqttStreamClient::resetProtocolState() {
    _rxBuffer.clear();
    _receivedConnAck = false;
    _subscribed = false;
    _packetId = 1;
}

void MqttStreamClient::onSocketConnected() {
    clearLastError();
    setStateText("Authorizing");
    _socket->write(buildConnectPacket());
    _socket->flush();
}

void MqttStreamClient::onSocketReadyRead() {
    if (!_state.running) {
        _socket->readAll();
        resetProtocolState();
        return;
    }
    _rxBuffer += _socket->readAll();
    parsePackets();
}

void MqttStreamClient::onSocketError(QAbstractSocket::SocketError socketError) {
    const QString message = toShortSocketError(socketError, _socket->errorString());
    setLastError(message);
    setStateText("Error");
    emitLog(message);
}

void MqttStreamClient::onSocketDisconnected() {
    _pingTimer.stop();
    if (!_state.running) {
        return;
    }
    setStateText("Reconnecting");
    emitLog(QString("Disconnected. Reconnect in %1 ms").arg(_config.reconnectMs));
    _reconnectTimer.start(qMax(100, _config.reconnectMs));
}

void MqttStreamClient::reconnect() {
    if (_state.running) {
        connectNow();
    }
}

void MqttStreamClient::sendPing() {
    if (_socket->state() == QAbstractSocket::ConnectedState) {
        _socket->write(buildPingPacket());
    }
}

void MqttStreamClient::parsePackets() {
    while (true) {
        QByteArray packet;
        quint8 type = 0;
        quint8 flags = 0;
        if (!tryReadPacket(packet, type, flags)) {
            return;
        }

        if (type == 2) { // CONNACK
            if (packet.size() < 2) {
                setLastError("Invalid CONNACK packet");
                _socket->disconnectFromHost();
                return;
            }
            const quint8 returnCode = static_cast<quint8>(packet.at(1));
            if (returnCode != 0) {
                setLastError(QString("MQTT CONNACK rejected code=%1").arg(returnCode));
                emitLog("Broker rejected CONNECT");
                _socket->disconnectFromHost();
                return;
            }
            _receivedConnAck = true;
            emitLog("MQTT connected. Sending SUBSCRIBE");
            _socket->write(buildSubscribePacket(_config.topic, 0));
            _socket->flush();
            continue;
        }

        if (type == 9) { // SUBACK
            _subscribed = true;
            setStateText("Streaming");
            emitLog("MQTT subscription active");
            _pingTimer.start(qMax(5000, _config.keepAliveSec * 500));
            continue;
        }

        if (type == 3) { // PUBLISH
            if (packet.size() < 2) {
                continue;
            }
            int offset = 0;
            const int topicSize = (static_cast<quint8>(packet.at(offset)) << 8)
                                  + static_cast<quint8>(packet.at(offset + 1));
            offset += 2;
            if (packet.size() < offset + topicSize) {
                continue;
            }
            const QString topic = QString::fromUtf8(packet.mid(offset, topicSize));
            offset += topicSize;

            const quint8 qos = (flags >> 1) & 0x03;
            if (qos > 0) {
                if (packet.size() < offset + 2) {
                    continue;
                }
                offset += 2; // packet id
            }

            if (packet.size() < offset) {
                continue;
            }
            const QByteArray payload = packet.mid(offset);
            onDataReceived(payload);

            if (_state.frames == 1) {
                emitLog(QString("First MQTT payload from topic %1").arg(topic));
            }
            continue;
        }
    }
}

QByteArray MqttStreamClient::buildConnectPacket() const {
    QByteArray variableHeader;
    variableHeader += encodeString("MQTT");
    variableHeader += char(0x04); // protocol level

    quint8 connectFlags = 0;
    connectFlags |= 0x02; // clean session
    if (!_config.user.isEmpty()) {
        connectFlags |= 0x80;
    }
    if (!_config.password.isEmpty()) {
        connectFlags |= 0x40;
    }
    variableHeader += char(connectFlags);

    const quint16 keepAlive = static_cast<quint16>(qBound(5, _config.keepAliveSec, 600));
    variableHeader += char((keepAlive >> 8) & 0xFF);
    variableHeader += char(keepAlive & 0xFF);

    QString clientId = _config.clientId.trimmed();
    if (clientId.isEmpty()) {
        clientId = QString("rtppp_%1").arg(QRandomGenerator::global()->generate(), 8, 16, QChar('0'));
    }

    QByteArray payload;
    payload += encodeString(clientId);
    if (!_config.user.isEmpty()) {
        payload += encodeString(_config.user);
    }
    if (!_config.password.isEmpty()) {
        payload += encodeString(_config.password);
    }

    QByteArray packet;
    packet += char(0x10); // CONNECT
    const int remainingLength = variableHeader.size() + payload.size();
    packet += encodeRemainingLength(remainingLength);
    packet += variableHeader;
    packet += payload;
    return packet;
}

QByteArray MqttStreamClient::buildSubscribePacket(const QString& topic, quint8 qos) {
    QByteArray variableHeader;
    variableHeader += char((_packetId >> 8) & 0xFF);
    variableHeader += char(_packetId & 0xFF);
    _packetId++;

    QByteArray payload;
    payload += encodeString(topic);
    payload += char(qos);

    QByteArray packet;
    packet += char(0x82); // SUBSCRIBE with flags
    packet += encodeRemainingLength(variableHeader.size() + payload.size());
    packet += variableHeader;
    packet += payload;
    return packet;
}

QByteArray MqttStreamClient::buildPingPacket() const {
    return QByteArray::fromHex("C000");
}

bool MqttStreamClient::tryReadPacket(QByteArray& packet, quint8& packetType, quint8& flags) {
    if (_rxBuffer.size() < 2) {
        return false;
    }

    const quint8 fixedHeader = static_cast<quint8>(_rxBuffer.at(0));
    packetType = (fixedHeader >> 4) & 0x0F;
    flags = fixedHeader & 0x0F;

    int multiplier = 1;
    int value = 0;
    int consumed = 1;
    quint8 encodedByte = 0;
    do {
        if (_rxBuffer.size() <= consumed) {
            return false;
        }
        encodedByte = static_cast<quint8>(_rxBuffer.at(consumed));
        value += (encodedByte & 127) * multiplier;
        multiplier *= 128;
        consumed++;
    } while ((encodedByte & 128) != 0 && consumed < 5);

    const int totalSize = consumed + value;
    if (_rxBuffer.size() < totalSize) {
        return false;
    }

    packet = _rxBuffer.mid(consumed, value);
    _rxBuffer.remove(0, totalSize);
    return true;
}

QByteArray MqttStreamClient::encodeString(const QString& text) {
    const QByteArray utf8 = text.toUtf8();
    QByteArray out;
    out += char((utf8.size() >> 8) & 0xFF);
    out += char(utf8.size() & 0xFF);
    out += utf8;
    return out;
}

QByteArray MqttStreamClient::encodeRemainingLength(int length) {
    QByteArray out;
    int remaining = qMax(0, length);
    do {
        quint8 encodedByte = remaining % 128;
        remaining /= 128;
        if (remaining > 0) {
            encodedByte |= 128;
        }
        out += char(encodedByte);
    } while (remaining > 0);
    return out;
}
