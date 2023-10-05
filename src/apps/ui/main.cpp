// Copyright (C) 2017 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include <QQmlApplicationEngine>
#include <QQmlContext>

#include <QGuiApplication>
#include <QApplication>
#include <QObject>
#include <QTime>
#include <QBasicTimer>
#include <QDebug>
#include <QEasingCurve>
#include <QGeoCoordinate>
#include <QtPositioning/private/qwebmercator_p.h>

#include "clientapplication.h"
#include "logfilepositionsource.h"

#include <QtNetwork/QSslSocket>
#include <iostream>

using namespace Qt::StringLiterals;

static const char *help = R"(Usage:
--plugin.<parameter_name> <parameter_value>    -  Sets parameter = value for plugin
)";

static QVariantMap parseArgs(QStringList args)
{
    QVariantMap parameters;
    while (!args.isEmpty()) {
        QString param = args.takeFirst();
        if (param.startsWith(u"--plugin.")) {
            param.remove(0, 9);
            if (args.isEmpty() || args.constFirst().startsWith(u"--")) {
                parameters.insert(param, QVariant(true));
            } else {
                QString value = args.takeFirst();
                if (value == u"true" || value == u"on" || value == u"enabled") {
                    parameters.insert(param, QVariant(true));
                } else if (value == u"false" || value == u"off"
                           || value == u"disable") {
                    parameters.insert(param, QVariant(false));
                } else {
                    parameters.insert(param, QVariant(value));
                }
            }
        }
    }
    return parameters;
}

int main(int argc, char *argv[])
{
    const QString additionalLibraryPaths = qEnvironmentVariable("QTLOCATION_EXTRA_LIBRARY_PATH");
    for (const auto &p : additionalLibraryPaths.split(u':', Qt::SkipEmptyParts))
    {
        QCoreApplication::addLibraryPath(p);
    }

    QGuiApplication application(argc, argv);
    QApplication app(argc, argv);
    ClientApplication client;

    QCoreApplication::setApplicationName(u"QtLocation app-v2x-ui example"_s);

    QStringList args = QCoreApplication::arguments();
    args.removeFirst();
    if (args.contains(u"--help")) {
        std::cout << qPrintable(QCoreApplication::applicationName()) << "\n\n" << help;
        return 0;
    }

    QVariantMap parameters = parseArgs(args);
    if (!parameters.contains(u"osm.useragent"_s))
    {
        parameters.insert(u"osm.useragent"_s, QCoreApplication::applicationName());
    }

    // add Class to QML
    qmlRegisterType<LogFilePositionSource>("Qt.LogFilePositionSource", 1, 0, "LogFilePositionSource");

    QQmlApplicationEngine engine;

    engine.rootContext()->setContextProperty("supportsSsl", QSslSocket::supportsSsl());
    engine.addImportPath(u":/imports"_s);
    engine.load(QUrl(u"qrc:///app-v2x-ui.qml"_s));
    QObject::connect(&engine, &QQmlApplicationEngine::quit, qApp, QCoreApplication::quit);

    auto *item = engine.rootObjects().value(0);
    if (item == nullptr)
    {
        return -1;
    }

    QMetaObject::invokeMethod(item, "initializeProviders", QVariant::fromValue(parameters));

    client.show();

    return application.exec();
}

