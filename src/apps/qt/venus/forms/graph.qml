import QtQuick
import QtCharts
import QtQuick.Controls

Rectangle {
    height: 720
    width: 1280
    visible: true

    Rectangle {
        width: 640
        height: 480

        ChartView {
            title: "Line Graph"
            anchors.fill: parent
            antialiasing: true

            LineSeries {
                name: "Line"
                XYPoint {x:0; y:0}
                XYPoint {x:1; y:2}
                XYPoint {x:2; y:4}
                XYPoint {x:3; y:6}
            }
        }
    }
}
