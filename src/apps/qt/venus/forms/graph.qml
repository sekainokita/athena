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

    property int xAxisValue: 0
    property real maxYAxisValue: 100

    LogFilePositionSource {
        id: positionSource
        onPositionChanged: {
            var pdrValue = positionSource.pdr;
            console.log("PDR value:", pdrValue);
            if (isFinite(pdrValue)) {
                xAxisValue += 1;
                lineSeries.append(xAxisValue, pdrValue);
                labelPdr.text = "PDR: " + pdrValue.toFixed(2) + "%";
                console.log(labelPdr.text);

                timeAxis.max = xAxisValue;

                if (pdrValue > maxYAxisValue) {
                    maxYAxisValue = pdrValue;
                    pdrAxis.max = maxYAxisValue;
                }
            } else {
                console.log("Ignored NaN, Inf, or -Inf value.");
            }
        }
    }

    ChartView {
        anchors.fill: parent
        antialiasing: true

        LineSeries {
            id: lineSeries
            name: "PDR"
            useOpenGL: true
            axisX: timeAxis
            axisY: pdrAxis
        }

        ValueAxis {
            id: timeAxis
            min: 0
            max: 100  
            tickCount: 10
            titleText: "Updates"
        }

        ValueAxis {
            id: pdrAxis
            min: 0
            max: 100  
            tickCount: 10
            titleText: "PDR"
        }
    }

    Label {
        id: labelPdr
        text: "PDR: 0.00%"
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.rightMargin: 20
        anchors.topMargin: 20
        font.pixelSize: 20
        color: "blue"
    }
}
