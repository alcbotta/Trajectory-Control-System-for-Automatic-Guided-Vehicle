#include "Robot.h"
#include <QPoint>


int Robot::GLOABAL_ID = 0;
Robot::Robot(QColor nColor, int nId, QObject *parent) :
    QObject(parent),
    robotColor ( nColor ),
    id ( nId )
{
    center = cv::Point2f ( -1, -1 );
    posEnvironment = QPoint (-1, -1 );
//    id = GLOABAL_ID++;
}

double Robot::getDistanceTo(cv::Point2f otherPoint)
{
    return sqrt ( pow (  otherPoint.x - center.x ,2  ) +
                  pow (  otherPoint.y - center.y ,2  ) );
}

void Robot::setPosEnvironment(QPoint pos)
{
    posEnvironment = pos;
}

int Robot::getPosRowEnvironment()
{
    return posEnvironment.y ();
}

int Robot::getPosColEnvironment()
{
    return posEnvironment.x();
}

int Robot::getId()
{
    return id;
}

void Robot::setRangeRgb(cv::Scalar lowRange, cv::Scalar highRange)
{
//    qDebug() << "setRangeRgb" << id;
//    qDebug() << lowRange [ 0 ] << lowRange [ 1 ] << lowRange [ 2 ];
//    qDebug() << highRange [ 0 ] << highRange [ 1 ] << highRange [ 2 ] << endl;
    lowRangeRgb = lowRange;
    highRangeRgb = highRange;
}

void Robot::setRangeHsv(cv::Scalar lowRange, cv::Scalar highRange)
{
//    qDebug() << "setRangeHsv" << id;
//    qDebug() << lowRange [ 0 ] << lowRange [ 1 ] << lowRange [ 2 ];
//    qDebug() << highRange [ 0 ] << highRange [ 1 ] << highRange [ 2 ] << endl;
    lowRangeHsv = lowRange;
    highRangeHsv = highRange;
}

cv::Scalar Robot::getLowRangeRgb()
{
    return lowRangeRgb;
}

cv::Scalar Robot::getHighRangeRgb()
{
    return highRangeRgb;
}

cv::Scalar Robot::getLowRangeHsv()
{
    return lowRangeHsv;
}

cv::Scalar Robot::getHighRangeHsv()
{
    return highRangeHsv;
}

void Robot::setRobotColor(QColor nColor)
{
    robotColor = nColor;
}

QColor Robot::getRobotColor()
{
    return robotColor;
}
