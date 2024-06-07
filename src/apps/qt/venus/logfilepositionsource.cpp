// Copyright (C) 2017 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "logfilepositionsource.h"

#include <QtCore/qdebug.h>
#include <QtCore/qfile.h>
#include <QtCore/qtimer.h>

//#define CONFIG_DEBUG (1)
#define CONNECTED_VEHICLE_MAX_CNT (5)

typedef enum {
    eCONNECTED_VEHICLE_0 = 0,
    eCONNECTED_VEHICLE_1,
    eCONNECTED_VEHICLE_2,
    eCONNECTED_VEHICLE_3,
    eCONNECTED_VEHICLE_4,
    eCONNECTED_VEHICLE_MAX
} CONNECTED_VEHICLE_E;

static double s_dLatitude = 37.40611064950719f;
static double s_dLongitude = 127.10226288596837f;

static double s_dCvLatitude[CONNECTED_VEHICLE_MAX_CNT] = {37.40611064950719f, 127.10226288596837f};
static double s_dCvLongitude[CONNECTED_VEHICLE_MAX_CNT] = {37.40611064950719f, 127.10226288596837f};

static unsigned int s_unSpeed = 0;
static unsigned int s_unDistance = 0;

static unsigned int s_unHeading = 0;
static double s_dCvHeading[CONNECTED_VEHICLE_MAX_CNT] = {0,};

static QString s_qStrDeviceId;
static double s_dPdr = 0.0f;

static QString s_qStrCvDeviceId[CONNECTED_VEHICLE_MAX_CNT] = {0,};

LogFilePositionSource::LogFilePositionSource(QObject *parent)
    : QGeoPositionInfoSource(parent),
      logFile(new QFile(this)),
      timer(new QTimer(this))
{
    //logFile->setFileName("/tmp/db_v2x_rx_temp_writing.csv");
    logFile->setFileName("/home/bman/work/athena/src/apps/qt/venus/db/rx_db_sample_1.csv");
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

unsigned int LogFilePositionSource::updateGpsPosition(void)
{
    QByteArray line = logFile->readLine().trimmed();

    unsigned int unHeading;
    unsigned int unSpeed;
    unsigned int unDistance;
    double dPdr;
    double dLatitude;
    double dLongitude;
    QString qStrDeviceId;

    bool bHasHeading = false;
    bool bHasLatitude = false;
    bool bHasLongitude = false;
    bool bHasDeviceId = false;
    bool bHasPdr = false;
    bool bHasDistance = false;
    bool bHasSpeed = false;

    unsigned int aunCvHeading[CONNECTED_VEHICLE_MAX_CNT];
    double adCvLatitude[CONNECTED_VEHICLE_MAX_CNT];
    double adCvLongitude[CONNECTED_VEHICLE_MAX_CNT];
    QString aqStrCvDeviceId[CONNECTED_VEHICLE_MAX_CNT];

    bool abHasCvHeading[CONNECTED_VEHICLE_MAX_CNT] = {false,};
    bool abHasCvLatitude[CONNECTED_VEHICLE_MAX_CNT] = {false,};
    bool abHasCvLongitude[CONNECTED_VEHICLE_MAX_CNT] = {false,};

    if (!line.isEmpty())
    {
        QList<QByteArray> data = line.split(',');
        QDateTime timestamp = QDateTime::fromString(QString(data.value(DB_TIME_COLUMN)).mid(0, 17), "yyyyMMddHHmmsszzz");
        qDebug() << "time" << timestamp;

        /* Rx Vehicle */
        unHeading = data.value(DB_HEADING_COLUMN).toDouble(&bHasHeading);
        s_unHeading = unHeading;

        dLatitude = data.value(DB_LATITUDE_COLUMN).toDouble(&bHasLatitude);
        s_dLatitude = dLatitude;

        dLongitude = data.value(DB_LONGITUDE_COLUMN).toDouble(&bHasLongitude);
        s_dLongitude = dLongitude;

        qStrDeviceId = data.value(DB_DEVICEID_COLUMN);
        s_qStrDeviceId = qStrDeviceId;

        dPdr = data.value(DB_PDR_COLUMN).toDouble(&bHasPdr);
        s_dPdr = dPdr;

        unDistance = data.value(DB_DISTANCE_COLUMN).toDouble(&bHasDistance);
        s_unDistance = unDistance;

        unSpeed = data.value(DB_SPEED_COLUMN).toDouble(&bHasSpeed);
        s_unSpeed = unSpeed;

        /* Tx Vehicle */
        aunCvHeading[eCONNECTED_VEHICLE_0] = data.value(DB_CV_HEADING_COLUMN).toDouble(&abHasCvHeading[eCONNECTED_VEHICLE_0]);
        s_dCvHeading[eCONNECTED_VEHICLE_0] = aunCvHeading[eCONNECTED_VEHICLE_0];

        adCvLatitude[eCONNECTED_VEHICLE_0] = data.value(DB_CV_LATITUDE_COLUMN).toDouble(&abHasCvLatitude[eCONNECTED_VEHICLE_0]);
        s_dCvLatitude[eCONNECTED_VEHICLE_0] = adCvLatitude[eCONNECTED_VEHICLE_0];

        adCvLongitude[eCONNECTED_VEHICLE_0] = data.value(DB_CV_LONGITUDE_COLUMN).toDouble(&abHasCvLongitude[eCONNECTED_VEHICLE_0]);
        s_dCvLongitude[eCONNECTED_VEHICLE_0] = adCvLongitude[eCONNECTED_VEHICLE_0];

        aqStrCvDeviceId[eCONNECTED_VEHICLE_0] = data.value(DB_CV_DEVICEID_COLUMN);
        s_qStrCvDeviceId[eCONNECTED_VEHICLE_0] = aqStrCvDeviceId[eCONNECTED_VEHICLE_0];
    }

    return 1;
}

unsigned int LogFilePositionSource::getGpsHeading(void)
{
#if defined(CONFIG_DEBUG)
    qDebug() << "s_unHeading" << s_unHeading;
#endif
    return s_unHeading;
}

unsigned int LogFilePositionSource::getGpsSpeed(void)
{
#if defined(CONFIG_DEBUG)
    qDebug() << "s_unSpeed" << s_unSpeed;
#endif
    return s_unSpeed;
}

unsigned int LogFilePositionSource::getGpsDistance(void)
{
#if defined(CONFIG_DEBUG)
    qDebug() << "s_unDistance" << s_unDistance;
#endif
    return s_unDistance;
}

double LogFilePositionSource::getGpsPdr(void)
{
#if defined(CONFIG_DEBUG)
    qDebug() << "s_dPdr" << s_dPdr;
#endif
    return s_dPdr;
}

double LogFilePositionSource::getGpsLatitude(void)
{
#if defined(CONFIG_DEBUG)
    qDebug() << "s_dLatitude" << s_dLatitude;
#endif
    return s_dLatitude;
}

double LogFilePositionSource::getGpsLongitude(void)
{
#if defined(CONFIG_DEBUG)
    qDebug() << "s_dLongitude" << s_dLongitude;
#endif
    return s_dLongitude;
}

QString LogFilePositionSource::getGpsDeviceId(void)
{
#if defined(CONFIG_DEBUG)
    qDebug() << "s_qStrDeviceId" << s_qStrDeviceId;
#endif
    return s_qStrDeviceId;
}

unsigned int LogFilePositionSource::getGpsCvHeading(void)
{
#if defined(CONFIG_DEBUG)
    qDebug() << "s_unHeading" << s_unHeading;
#endif
    return s_dCvHeading[eCONNECTED_VEHICLE_0];
}

double LogFilePositionSource::getGpsCvLatitude(void)
{
#if defined(CONFIG_DEBUG)
    qDebug() << "s_dCvLatitude" << s_dCvLatitude[eCONNECTED_VEHICLE_0];
#endif
    return s_dCvLatitude[eCONNECTED_VEHICLE_0];
}

double LogFilePositionSource::getGpsCvLongitude(void)
{
#if defined(CONFIG_DEBUG)
    qDebug() << "s_dCvLongitude" << s_dCvLongitude[eCONNECTED_VEHICLE_0];
#endif
    return s_dCvLongitude[eCONNECTED_VEHICLE_0];
}

QString LogFilePositionSource::getGpsCvDeviceId(void)
{
#if defined(CONFIG_DEBUG)
    qDebug() << "s_qStrCvDeviceId" << s_qStrCvDeviceId[eCONNECTED_VEHICLE_0];
#endif
    return s_qStrCvDeviceId[eCONNECTED_VEHICLE_0];
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
