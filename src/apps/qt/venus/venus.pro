TARGET = venus
TEMPLATE = app

QT += core qml network quick positioning positioning-private location widgets gui location charts

SOURCES += main.cpp
SOURCES += logfilepositionsource.cpp
SOURCES += clientapplication.cpp

HEADERS += logfilepositionsource.h \
    linegraph.h
HEADERS += clientapplication.h
HEADERS += linegraph.h

# Workaround for QTBUG-38735
QT_FOR_CONFIG += location-private

qml_resources.files = \
    qmldir \
    Main.qml \
    helper.js \
    map/MapComponent.qml \
    map/MapSliders.qml \
    map/Marker.qml \
    map/MiniMap.qml \
    menus/ItemPopupMenu.qml \
    menus/MainMenu.qml \
    menus/MapPopupMenu.qml \
    menus/MarkerPopupMenu.qml \
    forms/Geocode.qml \
    forms/GeocodeForm.ui.qml\
    forms/Message.qml \
    forms/MessageForm.ui.qml \
    forms/ReverseGeocode.qml \
    forms/ReverseGeocodeForm.ui.qml \
    forms/RouteCoordinate.qml \
    forms/Locale.qml \
    forms/LocaleForm.ui.qml \
    forms/RouteAddress.qml \
    forms/RouteAddressForm.ui.qml \
    forms/RouteCoordinateForm.ui.qml \
    forms/RouteList.qml \
    forms/RouteListDelegate.qml \
    forms/RouteListHeader.qml \
    resources/marker.png \
    resources/marker_blue.png \
    resources/vehicle.png \
    resources/scale.png \
    resources/scale_end.png

qml_resources.prefix = /qt/qml/Venus

RESOURCES += \
    qml_resources \
    logfile.qrc

target.path = $$[QT_INSTALL_EXAMPLES]/location/venus
INSTALLS += target

DISTFILES += \
    db/Rx_OBU_CKCOX23120002_20240529-10-26-50_20240529-11-16-38_2988secs.csv \
    db/Rx_OBU_CKCOX23120002_20240529-11-17-22_20240529-11-36-13_1131secs.csv \
    db/rx_db_sample_1.csv \
    db/rx_db_sample_2.csv \
    db_v2x_rx_temp_writing.csv \
    map/linegraph.qml
    map/LineGraph.qml
