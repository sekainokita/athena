#ifndef LINEGRAPH_H
#define LINEGRAPH_H

#include <QObject>
#include <QQmlEngine>
#include <QtCharts/QLineSeries>

class MyLineGraphProvider : public QObject {
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(QLineSeries* lineSeries READ lineSeries WRITE setLineSeries NOTIFY lineSeriesChanged)
public:
    QLineSeries* lineSeries() { return series; }
    void setLineSeries(QLineSeries* s) { series = s; emit lineSeriesChanged(); }
    void updateData(QList<QPointF> allData) { series->replace(allData); } // call this from your logic in c++
signals:
    void lineSeriesChanged();
private:
    QLineSeries* series = nullptr;
};

#endif // LINEGRAPH_H
