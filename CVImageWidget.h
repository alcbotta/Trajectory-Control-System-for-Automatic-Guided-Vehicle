#ifndef CVIMAGEWIDGET_H
#define CVIMAGEWIDGET_H

#include <QWidget>
#include <QImage>
#include <QPainter>
#include <opencv2/opencv.hpp>
#include <QMouseEvent>
#include "Environment.h"
#include "Robot.h"
#include "mapa.h"
#include <QSettings>
#include "ControlInterface.h"

#define AUX_GUIDE_PATH 40

class CVImageWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CVImageWidget(QWidget *parent = 0);

    QSize sizeHint() const;
    QSize minimumSizeHint() const;

    void mouseDoubleClickEvent ( QMouseEvent *event );
    void mouseMoveEvent  ( QMouseEvent *event );
    void mousePressEvent ( QMouseEvent *event );
    void mouseReleaseEvent  ( QMouseEvent *event );

    int getInitX ();
    int getInitY ();
    int getNumberOfCols ();
    int getNumberOfLines ();
    int getCircleRadius ();
    int getDistanceBetweenLinesX ();
    int getDistanceBetweenLinesY ();

    ControlInterface *getControlInterface();

//    void updateSettings ( QSettings &settings );

    void updateEnvironment ();
//    void initEnvironment ();

    void getStateOfClick ( int &row, int &col, QMouseEvent *event );

//    int lowBlue, lowGreen, lowRed;
//    int highBlue, highGreen, highRed;

    void painterArrow(QPainter &painter, qreal x, qreal y, qreal u, qreal v, QColor arrowColor);
    signals:
    void newMousePos (int x, int y, int r, int g, int b);

public slots:

    void executeAStar ();
//    void updateRange(int *values);
    void showImage(cv::Mat image);
    void getNewImage ();

    //  Eventos da aba de Guide-path:
    void setNumberOfLines ( int value );
    void setNumberOfCols ( int value );
    void setInitX ( int value );
    void setInitY ( int value );
    void setFinalX ( int value );
    void setFinalY ( int value );
    void setWidthOfLine ( int value );
    void setCircleRadius ( int value );
    void setDistanceBetweenLinesX ( int value );
    void setDistanceBetweenLinesY ( int value );
    void paintEvent(QPaintEvent* /*event*/) ;

//protected:


//    cv::VideoCapture cap;

//    ImageAnalyzis imageAnalyzis;
//    Environment environment;
//    std::vector <Robot *> robots;

private:
//    vector<point_t> *auxExecuteAStar( Robot *robot );
//    vector<char> generateSeqMov( vector<point_t>* shortestPath, Robot *nearest );
    void updateFinalX ();
    void updateFinalY ();

    QImage qimage;
    cv::Mat tmp;
    ControlInterface controlInterface;

    int numberOfLines;
    int numberOfCols;
    int initX;
    int initY;
    int finalX;
    int finalY;
    int widthOfLine;
    int distanceBetweenLinesX;
    int distanceBetweenLinesY;
    int circleRadius;
};

#endif // CVIMAGEWIDGET_H
