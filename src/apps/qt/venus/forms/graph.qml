import QtQuick
import QtCharts

ChartView {
    title: "Line Chart"
    anchors.fill: parent
    antialiasing: true

    LineSeries {
        name: "Line"
        XYPoint { x: 0; y: 0 }
        XYPoint { x: 1; y: 2 }
        XYPoint { x: 2; y: 4 }
        XYPoint { x: 3; y: 6 }
    }
}
