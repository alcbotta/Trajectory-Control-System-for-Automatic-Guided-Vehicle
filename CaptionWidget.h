#ifndef CAPTIONWIDGET_H
#define CAPTIONWIDGET_H

#include <QWidget>
#include <QPainter>

class CaptionWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CaptionWidget(QWidget *parent = 0);
    
protected:
    void paintEvent(QPaintEvent* /*event*/) ;
signals:
    
public slots:
    
};

#endif // CAPTIONWIDGET_H
