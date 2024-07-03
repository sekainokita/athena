//import QtQuick
//import QtCharts
//import QtQuick.Controls
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtCharts 2.15
import Qt.LogFilePositionSource 1.0

ApplicationWindow {
    visible: true
    height: 720
    width: 1280
    title: qsTr("PDR Chart")

    LogFilePositionSource{
        id: positionSource
        onPositionChanged: {
            var timestamp = new Date(positionSource.timestamp).getTime();
            lineSeries.append(timestamp, positionSource.pdr);
        }
    }

    ChartView {
        anchors.fill: parent
        antialiasing: true

        LineSeries {
            id: lineSeries
            name: "PDR"
            useOpenGL: true
        }

        DateTimeAxis {
            id: timeAxis
            format: "hh:mm:ss"
            titleText: "Time"
            tickCount: 10
        }

        ValueAxis {
            id: pdrAxis
            min: 0
            max: 100
            tickCount: 10
            titleText: "PDR"
        }

        LineSeries {
            axisX: timeAxis
            axisY: pdrAxis
        }
    }
}
