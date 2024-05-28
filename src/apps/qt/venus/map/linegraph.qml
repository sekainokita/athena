import QtQuick
import QtQuick.Layouts
import QtGraphs
import QtCharts

Rectangle{
    function minimumScaleFactor()
    {
        var hscalefactor = (400.0 / Math.max(Math.min(mapview.width, 1000), 400)) * 0.5
        var vscalefactor = (400.0 / Math.max(Math.min(mapview.height, 1000), 400)) * 0.5
        return Math.min(hscalefactor,vscalefactor)
    }

    function avgScaleFactor()
    {
        var hscalefactor = (400.0 / Math.max(Math.min(mapview.width, 1000), 400)) * 0.5
        var vscalefactor = (400.0 / Math.max(Math.min(mapview.height, 1000), 400)) * 0.5
        return (hscalefactor+vscalefactor) * 0.5
    }

    id: linegraphRect
    width: Math.floor(mapview.width * avgScaleFactor()) + 2
    height: Math.floor(mapview.height * avgScaleFactor()) + 2
    anchors.right: (parent) ? parent.right : undefined
    anchors.rightMargin: 10
    anchors.top: (parent) ? parent.top : undefined
    anchors.topMargin: 10
    color: "#242424"

    ChartView {
        id:linegraph
        anchors.top: parent.top
        anchors.topMargin: 1
        anchors.left: parent.left
        anchors.leftMargin: 1
        width: Math.floor(mapview.width * avgScaleFactor())
        height: Math.floor(mapview.height * avgScaleFactor())

        title: "PDR Graph"

        LineSeries {id:ls}
    }
    MyLineGraphProvider {LineSeries: ls}
}
