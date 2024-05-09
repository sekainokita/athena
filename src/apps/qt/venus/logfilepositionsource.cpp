// Copyright (C) 2017 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "logfilepositionsource.h"

#include <QtCore/qdebug.h>
#include <QtCore/qfile.h>
#include <QtCore/qtimer.h>

static double s_dLatitude = 37.40611064950719;
static double s_dLongitude = 127.10226288596837;
static double s_dAddPosition = 0.000001;

LogFilePositionSource::LogFilePositionSource(QObject *parent)
    : QGeoPositionInfoSource(parent),
      logFile(new QFile(this)),
      timer(new QTimer(this))
{
    connect(timer, &QTimer::timeout, this, &LogFilePositionSource::readNextPosition);

    logFile->setFileName("../ui/simplelog.txt");
    if (!logFile->open(QIODevice::ReadOnly))
    {
        qWarning() << "Error: cannot open source file" << logFile->fileName();
    }
    else
    {
        qDebug() << "opened file: simplelog.txt";
    }
    qDebug() << "LogFilePositionSource() is initialized.";
}

QGeoPositionInfo LogFilePositionSource::lastKnownPosition(bool /*satelliteMethodsOnly*/) const
{
    return lastPosition;
}

LogFilePositionSource::PositioningMethods LogFilePositionSource::supportedPositioningMethods() const
{
    return AllPositioningMethods;
}

int LogFilePositionSource::minimumUpdateInterval() const
{
    return 100;
}

void LogFilePositionSource::startUpdates()
{
    lastError = QGeoPositionInfoSource::NoError;
    int interval = updateInterval();

    qDebug() << "interval" << interval;

    if (interval < minimumUpdateInterval())
        interval = minimumUpdateInterval();

    qDebug() << "minimum interval" << interval;

    timer->start(interval);
}

double LogFilePositionSource::getGpsInfo(double dLatitude, double dLongitude)
{
    qDebug() << "getGpsInfo lantitude" << dLatitude << "longitude" << dLongitude;

    return 111.111;
}

double LogFilePositionSource::getGpsLatitude(void)
{
    s_dLatitude = s_dLatitude + s_dAddPosition;
    return s_dLatitude;
}

double LogFilePositionSource::getGpsLongitude(void)
{
    s_dLongitude = s_dLongitude + s_dAddPosition;
    return s_dLongitude;
}

void LogFilePositionSource::stopUpdates()
{
    timer->stop();
}

void LogFilePositionSource::requestUpdate(int /*timeout*/)
{
    // For simplicity, ignore timeout - assume that if data is not available
    // now, no data will be added to the file later
    lastError = QGeoPositionInfoSource::NoError;
    if (logFile->canReadLine()) {
        readNextPosition();
    } else {
        lastError = QGeoPositionInfoSource::UpdateTimeoutError;
        emit QGeoPositionInfoSource::errorOccurred(lastError);
    }
}

void LogFilePositionSource::readNextPosition()
{
    QByteArray line = logFile->readLine().trimmed();
    if (!line.isEmpty()) {
        QList<QByteArray> data = line.split(' ');
        double latitude;
        double longitude;
        bool hasLatitude = false;
        bool hasLongitude = false;
        QDateTime timestamp = QDateTime::fromString(QString(data.value(0)), Qt::ISODate);
        latitude = data.value(1).toDouble(&hasLatitude);
        longitude = data.value(2).toDouble(&hasLongitude);

        if (hasLatitude && hasLongitude && timestamp.isValid()) {
            QGeoCoordinate coordinate(latitude, longitude);
            QGeoPositionInfo info(coordinate, timestamp);
            if (info.isValid()) {
                lastPosition = info;
                emit positionUpdated(info);
            }
        }
    }
}

QGeoPositionInfoSource::Error LogFilePositionSource::error() const
{
    return lastError;
}
