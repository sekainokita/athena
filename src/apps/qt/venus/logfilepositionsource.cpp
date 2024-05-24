// Copyright (C) 2017 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "logfilepositionsource.h"

#include <QtCore/qdebug.h>
#include <QtCore/qfile.h>
#include <QtCore/qtimer.h>

#define CONNECTED_VEHICLE_MAX_CNT 5

typedef enum {
    eCONNECTED_VEHICLE_0 = 0,
    eCONNECTED_VEHICLE_1,
    eCONNECTED_VEHICLE_2,
    eCONNECTED_VEHICLE_3,
    eCONNECTED_VEHICLE_4,
    eCONNECTED_VEHICLE_MAX
} CONNECTED_VEHICLE_Es;

static double s_dLatitude = 37.40611064950719;
static double s_dLongitude = 127.10226288596837;

static double s_dConnectedVehicleLatitude[CONNECTED_VEHICLE_MAX_CNT] = {37.40611064950719, 127.10226288596837};
static double s_dConnectedVehicleLongitude[CONNECTED_VEHICLE_MAX_CNT] = {37.40611064950719, 127.10226288596837};

static unsigned int s_unHeading = 0;

static double s_dConnectedVehicleHeading[CONNECTED_VEHICLE_MAX_CNT] = {37.40611064950719, 127.10226288596837};

LogFilePositionSource::LogFilePositionSource(QObject *parent)
    : QGeoPositionInfoSource(parent),
      logFile(new QFile(this)),
      timer(new QTimer(this))
{
    logFile->setFileName("/tmp/db_v2x_rx_temp_writing.csv");
    if (!logFile->open(QIODevice::ReadOnly))
    {
        qWarning() << "Error: cannot open source file" << logFile->fileName();
    }
    else
    {
        qDebug() << "opened file: db_v2x_rx_temp_writing.csv";
    }

    qDebug() << "LogFilePositionSource() is initialized.";
}

QGeoPositionInfo LogFilePositionSource::lastKnownPosition(bool /*satelliteMethodsOnly*/) const
{
    /* Not Used */
    return lastPosition;
}

LogFilePositionSource::PositioningMethods LogFilePositionSource::supportedPositioningMethods() const
{
    /* Not Used */
    return AllPositioningMethods;
}

int LogFilePositionSource::minimumUpdateInterval() const
{
    /* Not Used */
    return 0;
}

void LogFilePositionSource::startUpdates()
{
    /* Not Used */
    return;
}

double LogFilePositionSource::getGpsConnectedvehicleLatitude(void)
{
    QByteArray line = logFile->readLine().trimmed();

    if (!line.isEmpty())
    {
        QList<QByteArray> data = line.split(',');
        double latitude;
        bool hasLatitude = false;
        QDateTime timestamp = QDateTime::fromString(QString(data.value(DB_TIME_COLUMN)).mid(0, 17), "yyyyMMddHHmmsszzz");
        qDebug() << "time" << timestamp;
        latitude = data.value(DB_LATITUDE_COLUMN).toDouble(&hasLatitude);

        s_dConnectedVehicleLatitude[eCONNECTED_VEHICLE_0] = latitude;
    }
    return s_dConnectedVehicleLatitude[eCONNECTED_VEHICLE_0];
}

double LogFilePositionSource::getGpsConnectedvehicleLongitude(void)
{
    QByteArray line = logFile->readLine().trimmed();

    if (!line.isEmpty())
    {
        QList<QByteArray> data = line.split(',');
        double longitude;
        bool hasLongitude = false;
        QDateTime timestamp = QDateTime::fromString(QString(data.value(DB_TIME_COLUMN)).mid(0, 17), "yyyyMMddHHmmsszzz");
        longitude = data.value(DB_LONGITUDE_COLUMN).toDouble(&hasLongitude);

        s_dConnectedVehicleLongitude[eCONNECTED_VEHICLE_0] = longitude;
    }
    return s_dConnectedVehicleLongitude[eCONNECTED_VEHICLE_0];
}

unsigned int LogFilePositionSource::updateGpsPosition(void)
{
    QByteArray line = logFile->readLine().trimmed();

    unsigned int unheading;
    double latitude;
    double longitude;

    bool bHasHeading = false;
    bool hasLatitude = false;
    bool hasLongitude = false;

    if (!line.isEmpty())
    {
        QList<QByteArray> data = line.split(',');
        QDateTime timestamp = QDateTime::fromString(QString(data.value(DB_TIME_COLUMN)).mid(0, 17), "yyyyMMddHHmmsszzz");
        qDebug() << "time" << timestamp;

        unheading = data.value(DB_HEADING_COLUMN).toDouble(&bHasHeading);
        s_unHeading = unheading;

        latitude = data.value(DB_LATITUDE_COLUMN).toDouble(&hasLatitude);
        s_dLatitude = latitude;
        qDebug() << "s_dLatitude" << s_dLatitude;

        longitude = data.value(DB_LONGITUDE_COLUMN).toDouble(&hasLongitude);
        s_dLongitude = longitude;
        qDebug() << "s_dLongitude" << s_dLongitude;
    }

    return 1;
}


unsigned int LogFilePositionSource::getGpsHeading(void)
{
    qDebug() << "s_unHeading" << s_unHeading;
    return s_unHeading;
}

double LogFilePositionSource::getGpsLatitude(void)
{
    qDebug() << "s_dLatitude" << s_dLatitude;
    return s_dLatitude;
}

double LogFilePositionSource::getGpsLongitude(void)
{
    qDebug() << "s_dLongitude" << s_dLongitude;
    return s_dLongitude;
}

void LogFilePositionSource::stopUpdates()
{
    /* Not Used */
    return;
}

void LogFilePositionSource::requestUpdate(int /*timeout*/)
{
    /* Not Used */
    return;
}

void LogFilePositionSource::readNextPosition()
{
    /* Not Used */
    return;
}

QGeoPositionInfoSource::Error LogFilePositionSource::error() const
{
    /* Not Used */
    return lastError;
}
