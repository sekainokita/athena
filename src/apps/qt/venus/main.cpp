// Copyright (C) 2017 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include <QtQml/QQmlApplicationEngine>
#include <QtQml/QQmlContext>

#include <QtGui/QGuiApplication>
#include "clientapplication.h"
#include "logfilepositionsource.h"

#if QT_CONFIG(ssl)
#include <QtNetwork/QSslSocket>
#endif

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
#if QT_CONFIG(library)
    const QString additionalLibraryPaths = qEnvironmentVariable("QTLOCATION_EXTRA_LIBRARY_PATH");
    for (const auto &p : additionalLibraryPaths.split(u':', Qt::SkipEmptyParts))
        QCoreApplication::addLibraryPath(p);
#endif

    QGuiApplication application(argc, argv);
    QCoreApplication::setApplicationName(u"KETI Venus Application"_s);

    QStringList args = QCoreApplication::arguments();
    args.removeFirst();
    if (args.contains(u"--help")) {
        std::cout << qPrintable(QCoreApplication::applicationName()) << "\n\n" << help;
        return 0;
    }

    QVariantMap parameters = parseArgs(args);
    if (!parameters.contains(u"osm.useragent"_s))
    {
        std::cout << qPrintable(QCoreApplication::applicationName());
        parameters.insert(u"osm.useragent"_s, QCoreApplication::applicationName());
    }

    parameters.insert(u"osm.mapping.custom.host"_s, "https://tile.thunderforest.com/cycle/%z/%x/%y.png?apikey=15bb234f9b46448abd2f2b656f166270");
    parameters.insert(u"osm.mapping.custom.host"_s, "https://tile.thunderforest.com/transport/%z/%x/%y.png?apikey=15bb234f9b46448abd2f2b656f166270");
    parameters.insert(u"osm.mapping.custom.host"_s, "https://tile.thunderforest.com/landscape/%z/%x/%y.png?apikey=15bb234f9b46448abd2f2b656f166270");
    parameters.insert(u"osm.mapping.custom.host"_s, "https://tile.thunderforest.com/outdoors/%z/%x/%y.png?apikey=15bb234f9b46448abd2f2b656f166270");
    parameters.insert(u"osm.mapping.custom.host"_s, "https://tile.thunderforest.com/transport-dark/%z/%x/%y.png?apikey=15bb234f9b46448abd2f2b656f166270");
    parameters.insert(u"osm.mapping.custom.host"_s, "https://tile.thunderforest.com/spinal-map/%z/%x/%y.png?apikey=15bb234f9b46448abd2f2b656f166270");
    parameters.insert(u"osm.mapping.custom.host"_s, "https://tile.thunderforest.com/pioneer/%z/%x/%y.png?apikey=15bb234f9b46448abd2f2b656f166270");
    parameters.insert(u"osm.mapping.custom.host"_s, "https://tile.thunderforest.com/mobile-atlas/%z/%x/%y.png?apikey=15bb234f9b46448abd2f2b656f166270");
    parameters.insert(u"osm.mapping.custom.host"_s, "https://tile.thunderforest.com/neighbourhood/%z/%x/%y.png?apikey=15bb234f9b46448abd2f2b656f166270");
    parameters.insert(u"osm.mapping.custom.host"_s, "https://tile.thunderforest.com/atlas/%z/%x/%y.png?apikey=15bb234f9b46448abd2f2b656f166270");

    //parameters.insert(u"osm.mapping.providersrepository.address"_s, "https://tile.thunderforest.com/cycle/%z/%x/%y.png?apikey=15bb234f9b46448abd2f2b656f166270");
    // add Class to QML
    qmlRegisterType<LogFilePositionSource>("Qt.LogFilePositionSource", 1, 0, "LogFilePositionSource");
    qRegisterMetaType<QGeoPositionInfo>();

    QQmlApplicationEngine engine;
#if QT_CONFIG(ssl)
    engine.rootContext()->setContextProperty("supportsSsl", QSslSocket::supportsSsl());
#else
    engine.rootContext()->setContextProperty("supportsSsl", false);
#endif
    engine.loadFromModule("Venus", "Main");

    auto *item = engine.rootObjects().value(0);
    if (item == nullptr)
        return -1;

    QMetaObject::invokeMethod(item, "initializeProviders", QVariant::fromValue(parameters));

    return application.exec();
}
