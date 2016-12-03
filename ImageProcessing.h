#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

//#include <QObject>

//class ImageProcessing : public QObject
//{
//    Q_OBJECT
//public:
//    explicit ImageProcessing(QObject *parent = 0);
    
//signals:
    
//public slots:
    
//};

#include <QObject>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QDebug>
#include <iostream>
#include <ctype.h>
#include <vector>
#include <queue>
#include <algorithm>
using namespace std;

#define NUMBER_OF_RESULTS 6

//#define DEBUG_SYSTEM_COLOR

class Robot;

class ImageProcessing : public QObject
{
    Q_OBJECT
public:

    explicit ImageProcessing(QObject *parent = 0);
    bool readNewImage ( cv::Mat &image );
    void drawArrow ( cv::Mat image, int x, int y, int u, int v, cv::Scalar color, int size, int thickness );
    void detectRobots (cv:: Mat image, cv::Scalar lowRange, cv::Scalar highRange, Robot *robot,
                       int minRadiusRobot, int maxRadiusRobot, int maxDistCenters, cv::Mat *results = NULL );
    string getImgType(int imgTypeInt);
    void setSystemColorRgb ( bool sysColor );
    bool getSystemColorRgb ();

signals:

public slots:

private:
    cv::VideoCapture cap;
    bool systemColorRgb;
};

#endif // IMAGEPROCESSING_H
