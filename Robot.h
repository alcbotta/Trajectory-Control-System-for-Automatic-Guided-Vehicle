#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>
#include "Util.h"
#include <QPoint>

class Robot : public QObject
{
    Q_OBJECT
public:
    explicit Robot(QColor nColor,int nId, QObject *parent = 0);
    double getDistanceTo ( cv::Point2f otherPoint );
    void setPosEnvironment ( QPoint pos );
    int getPosRowEnvironment ();
    int getPosColEnvironment ();
    int getId ();
    void setRangeRgb ( cv::Scalar lowRange, cv::Scalar highRange );
    void setRangeHsv ( cv::Scalar lowRange, cv::Scalar highRange );
    cv::Scalar getLowRangeRgb ();
    cv::Scalar getHighRangeRgb ();
    cv::Scalar getLowRangeHsv ();
    cv::Scalar getHighRangeHsv ();

    void setRobotColor ( QColor nColor );
    QColor getRobotColor ();
signals:
    
public slots:

public:
    cv::Point2f center;
    QPoint posEnvironment;
    double direction;
    int id;
    cv::Scalar lowRangeRgb;
    cv::Scalar highRangeRgb;
    cv::Scalar lowRangeHsv;
    cv::Scalar highRangeHsv;
    QColor robotColor;

private:
    static int GLOABAL_ID;


};

#endif // ROBOT_H
