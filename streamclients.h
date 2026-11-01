// Declares NTRIP, TCP, and MQTT transports that deliver raw byte streams.
#ifndef STREAMCLIENTS_H
#define STREAMCLIENTS_H

#include <QByteArray>
#include <QDateTime>
#include <QObject>
#include <QSslError>
#include <QSslSocket>
#include <QTcpSocket>
#include <QTimer>

struct NtripStreamConfig {
    QString host;
    int port = 2101;
    QString mountPoint;
    QString user;
    QString password;
    QString nmea;
    QString ntripVersion = "2";
    int reconnectMs = 3000;
};

struct TcpStreamConfig {
    QString host;
    int port = 0;
    int reconnectMs = 3000;
};

struct MqttStreamConfig {
    QString host;
    int port = 1883;
    QString topic;
    QString clientId;
    QString user;
    QString password;
    int reconnectMs = 3000;
    int keepAliveSec = 30;
};

struct StreamRuntimeState {
    QString stateText;
    quint64 bytes = 0;
    quint64 frames = 0;
    QDateTime lastDataUtc;
    QString lastError;
    bool running = false;
};

class BaseStreamClient : public QObject
{
    Q_OBJECT

public:
    explicit BaseStreamClient(const QString& sourceName, QObject* parent = nullptr);
    ~BaseStreamClient() override;

    virtual void start() = 0;
    virtual void stop() = 0;

    StreamRuntimeState runtimeState() const;
    QString sourceName() const;

signals:
    void stateChanged();
    void logMessage(const QString& source, const QString& message);
    void dataFrame(const QString& source, qint64 bytes, const QByteArray& payload);

protected:
    void setStateText(const QString& text);
    void setRunning(bool running);
    void setLastError(const QString& errorText);
    void clearLastError();
    void onDataReceived(const QByteArray& payload);
    void emitLog(const QString& message);

    StreamRuntimeState _state;
    QString _sourceName;
};

class NtripStreamClient : public BaseStreamClient
{
    Q_OBJECT

public:
    explicit NtripStreamClient(QObject* parent = nullptr, const QString& sourceName = "NTRIP");
    ~NtripStreamClient() override;

    void setConfig(const NtripStreamConfig& config);
    NtripStreamConfig config() const;

    void start() override;
    void stop() override;

private slots:
    void onSocketConnected();
    void onSocketReadyRead();
    void onSocketError(QAbstractSocket::SocketError socketError);
    void onSocketDisconnected();
    void onSocketEncrypted();
    void onSslErrors(const QList<QSslError>& errors);
    void onInactivityTimeout();
    void reconnect();

private:
    void connectNow();
    QByteArray buildRequest() const;
    bool parseResponseHeader(const QByteArray& headerBlock);
    bool tryParseV1Response(bool finalAttempt);
    bool tryParseBufferedHttpResponse(bool finalAttempt);
    QByteArray takeChunkedPayload();
    bool canRetryWithV1() const;
    void retryWithV1();
    void scheduleReconnect(const QString& reason);
    int inactivityTimeoutSec() const;

    QSslSocket* _socket = nullptr;
    QTimer _reconnectTimer;
    QTimer _inactivityTimer;
    NtripStreamConfig _config;
    QByteArray _rxBuffer;
    bool _headerParsed = false;
    bool _chunkedTransfer = false;
    int _chunkBytesRemaining = -1;
    bool _usingV1Fallback = false;
    bool _triedV1Fallback = false;
    QDateTime _streamStartedUtc;
};

class TcpStreamClient : public BaseStreamClient
{
    Q_OBJECT

public:
    explicit TcpStreamClient(QObject* parent = nullptr, const QString& sourceName = "TCP");
    ~TcpStreamClient() override;

    void setConfig(const TcpStreamConfig& config);
    TcpStreamConfig config() const;

    void start() override;
    void stop() override;

private slots:
    void onSocketConnected();
    void onSocketReadyRead();
    void onSocketError(QAbstractSocket::SocketError socketError);
    void onSocketDisconnected();
    void reconnect();

private:
    void connectNow();

    QTcpSocket* _socket = nullptr;
    QTimer _reconnectTimer;
    TcpStreamConfig _config;
};

class MqttStreamClient : public BaseStreamClient
{
    Q_OBJECT

public:
    explicit MqttStreamClient(QObject* parent = nullptr, const QString& sourceName = "MQTT");
    ~MqttStreamClient() override;

    void setConfig(const MqttStreamConfig& config);
    MqttStreamConfig config() const;

    void start() override;
    void stop() override;

private slots:
    void onSocketConnected();
    void onSocketReadyRead();
    void onSocketError(QAbstractSocket::SocketError socketError);
    void onSocketDisconnected();
    void reconnect();
    void sendPing();

private:
    void connectNow();
    void resetProtocolState();
    void parsePackets();
    QByteArray buildConnectPacket() const;
    QByteArray buildSubscribePacket(const QString& topic, quint8 qos);
    QByteArray buildPingPacket() const;
    bool tryReadPacket(QByteArray& packet, quint8& packetType, quint8& flags);
    static QByteArray encodeString(const QString& text);
    static QByteArray encodeRemainingLength(int length);

    QTcpSocket* _socket = nullptr;
    QTimer _reconnectTimer;
    QTimer _pingTimer;
    MqttStreamConfig _config;
    QByteArray _rxBuffer;
    quint16 _packetId = 1;
    bool _receivedConnAck = false;
    bool _subscribed = false;
};

#endif // STREAMCLIENTS_H
