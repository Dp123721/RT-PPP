// Declares GNSS signal, frequency, and observation-code mapping helpers.
#pragma once

#include <QChar>
#include <QString>
#include <QStringList>
#include <QVector>

QString msmSignalToRinexType2ch(QChar system, int signalId);
QString msmSignalToRinexType2ch(QChar system, int signalId, const QVector<int>& signalIdsInMessage);
int rinexType2chToMsmSignalId(QChar system, const QString& type2ch);
QString ssrSignalToRinexType2ch(QChar system, int signalCode);
QString ssrSignalToRinexType2ch(QChar system, int signalCode, int messageType);
int rinexType2chToSsrSignalCode(QChar system, const QString& type2ch);
double signalWavelengthMeters(QChar system, const QString& type2ch, int glonassChannel);
QStringList rinexObsTypesForSystem(QChar system);
