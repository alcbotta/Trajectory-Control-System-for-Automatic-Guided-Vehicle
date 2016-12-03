#include "CVImageWidget.h"
#include <QDebug>
#include <QPointF>
#include "Util.h"
#include "Robot.h"

#include <QMessageBox>
#include <map>
#include <QString>
#include <QStaticText>



CVImageWidget::CVImageWidget(QWidget *parent)
    : QWidget(parent)
{

    this->setMouseTracking ( true );
    circleRadius = CIRCLE_RADIUS;
    numberOfLines = NUMBER_ROWS;
    numberOfCols = NUMBER_COLS;
}

QSize CVImageWidget::sizeHint() const
{
    return qimage.size();
}

QSize CVImageWidget::minimumSizeHint() const
{
    return qimage.size();
}

void CVImageWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
//    qDebug() << "CVImageWidget::mouseDoubleClickEvent";
}

void CVImageWidget::mouseMoveEvent(QMouseEvent *event)
{
//    qDebug() << "CVImageWidget::mouseMoveEvent" << event->x() << event->y();
    QRgb rgbPos = qimage.pixel ( event->x(), event->y() );
//    qDebug() << qRed ( rgbPos ) << qGreen ( rgbPos ) << qBlue ( rgbPos );
    emit newMousePos ( event->x(), event->y(), qRed ( rgbPos ), qGreen ( rgbPos ), qBlue ( rgbPos ) );
}

void CVImageWidget::mousePressEvent(QMouseEvent *event)
{
//    qDebug() << "CVImageWidget::mousePressEvent";

    int i, j;
    getStateOfClick ( i, j, event );
    if ( i != -1 )
    {
        if ( event->button() == Qt::MiddleButton )
        {
            if ( controlInterface.getEnvironment()->states [ i ][ j ]->isObstacle )
                controlInterface.getEnvironment()->setEmpty ( i, j );
            else
                controlInterface.getEnvironment()->setObstacle ( i, j );
        }
        else if ( event->button() == Qt::RightButton )
        {
            controlInterface.getEnvironment()->clearPath();
            controlInterface.getEnvironment()->setGoal ( i, j );
        }
    }
}

void CVImageWidget::mouseReleaseEvent(QMouseEvent * event)
{
    //    qDebug() << "CVImageWidget::mouseReleaseEvent";
}

int CVImageWidget::getInitX()
{
    return initX;
}

int CVImageWidget::getInitY()
{
    return initY;
}

int CVImageWidget::getNumberOfCols()
{
    return numberOfCols;
}

int CVImageWidget::getNumberOfLines()
{
    return numberOfLines;
}

int CVImageWidget::getCircleRadius()
{
    return circleRadius;
}

int CVImageWidget::getDistanceBetweenLinesX()
{
    return distanceBetweenLinesX;
}

int CVImageWidget::getDistanceBetweenLinesY()
{
    return distanceBetweenLinesY;
}

ControlInterface *CVImageWidget::getControlInterface()
{
    return &controlInterface;
}

void CVImageWidget::updateEnvironment()
{
    this->updateFinalX();
    this->updateFinalY();

    float xMiddle [ numberOfCols ];
    float yMiddle [ numberOfLines ];

    for ( int j = 0; j < numberOfCols; j++ )
    {
        xMiddle [ j ] =  distanceBetweenLinesX * j + initX + widthOfLine  / 2.0;
    }

    for ( int j = 0; j < numberOfLines; j++ )
    {
        yMiddle [ j ] = distanceBetweenLinesY * j + initY + widthOfLine / 2.0;
    }

    for ( int i = 0; i < numberOfLines; i++ )
    {
        for ( int j = 0; j < numberOfCols; j++ )
        {
            controlInterface.getEnvironment()->states [ i ][ j ]->yInit = yMiddle [ i ];
            controlInterface.getEnvironment()->states [ i ][ j ]->xInit = xMiddle [ j ];
        }
    }
}


void CVImageWidget::getStateOfClick(int &row, int &col, QMouseEvent *event)
{
    row = -1;
    col = -1;
    for ( int i = 0; i < numberOfLines; i++ )
    {
        for ( int j = 0; j < numberOfCols; j++ )
        {
            if ( sqrt ( pow ( controlInterface.getEnvironment()->states [ i ][ j ]->xInit - event->x(), 2 ) +
                    pow ( controlInterface.getEnvironment()->states [ i ][ j ]->yInit - event->y(), 2 ) ) < circleRadius )
            {
//                qDebug() << "clicou no " << i << j;
                row = i;
                col = j;
            }
        }
    }
}

void CVImageWidget::updateFinalX()
{
    finalX = ( this->getNumberOfCols() - 1 ) * this->getDistanceBetweenLinesX() + 2 * AUX_GUIDE_PATH;

}

void CVImageWidget::updateFinalY()
{
    finalY = ( this->getNumberOfLines() - 1 ) * this->getDistanceBetweenLinesY() + 2 * AUX_GUIDE_PATH;
}

void CVImageWidget::executeAStar()
{

    int robotNumber = -1;
    vector <char> seqCom = controlInterface.executeAStar( robotNumber );
    QString str;
    if ( robotNumber != -1 )
    {
        if ( robotNumber == RED_ROBOT_NUMBER )
            str.append ( "Robo vermelho:\n" );
        else if ( robotNumber == GREEN_ROBOT_NUMBER )
            str.append ( "Robo verde:\n" );
        else if ( robotNumber == BLUE_ROBOT_NUMBER )
            str.append ( "Robo azul:\n" );
        else if ( robotNumber == YELLOW_ROBOT_NUMBER )
            str.append ( "Robo amarelo:\n" );
        else if ( robotNumber == MAGENTA_ROBOT_NUMBER )
            str.append ( "Robo rosa:\n" );
        else if ( robotNumber == PURPLE_ROBOT_NUMBER )
            str.append ( "Robo roxo:\n" );
        else if ( robotNumber == ORANGE_ROBOT_NUMBER )
            str.append ( "Robo laranja:\n" );
        str.append ( QString::number( robotNumber ) );
        str.append ( '-' );
        for ( int i = 0; i < seqCom.size(); i++ )
        {
            str.append( QChar ( (char)seqCom.at ( i )) );
            str.append ( '-' );
        }
        //  introduz o numero do robo na mensagem
        seqCom.insert( seqCom.begin(), (char)robotNumber);
        //  realiza o envio pela serial de forma automática
        controlInterface.sendSerial ( seqCom );
    }
    else
    {
        str = QString ( "Nao ha caminho." );
    }

//    QMessageBox::information( this, "Menor Caminho", str );
}

void CVImageWidget::showImage(cv::Mat image)
{
    switch ( image.type() )
    {
        case CV_8UC1:
            cvtColor(image, tmp, CV_GRAY2RGB);
            break;
        case CV_8UC3:
            cvtColor(image, tmp, CV_BGR2RGB);
            break;
    }
    assert(tmp.isContinuous());
    qimage = QImage(tmp.data, tmp.cols, tmp.rows, tmp.cols*3, QImage::Format_RGB888);
    this->setFixedSize(image.cols, image.rows);
    controlInterface.detectRobots ( image );

//    for ( int k = 0; k < controlInterface.getRobots()->size(); k++ )
//    {
//        Robot *robot = controlInterface.getRobots()->at ( k );
//        if ( robot->getPosRowEnvironment () != -1 && robot->getPosColEnvironment() != -1 )
//        {
//            controlInterface.getEnvironment()->setEmpty ( robot->getPosRowEnvironment (), robot->getPosColEnvironment () );
////            qDebug() << "setando como empty " << robot->getPosRowEnvironment () << robot->getPosColEnvironment ();
//        }
//        robot->center = cv::Point2f ( -1, -1 ); // reseta a posição do robô
//        robot->setPosEnvironment ( QPoint ( -1, -1 ) );
//    }
//    for ( int k = 0; k < controlInterface.getRobots()->size(); k++ )
//    {
//        Robot *robot = controlInterface.getRobots()->at ( k );
//        if ( robot->getPosRowEnvironment () != -1 && robot->getPosColEnvironment() != -1 )
//        {
//            environment.setEmpty ( robot->getPosRowEnvironment (), robot->getPosColEnvironment () );
//            qDebug() << "setando como empty " << robot->getPosRowEnvironment () << robot->getPosColEnvironment ();
//        }
////            environment.states [ robot->getPosRowEnvironment () ][ robot->getPosColEnvironment () ]->isEmpty;
//    }

//    imageAnalyzis.detectDirections( image, cv::Scalar(lowBlue,lowGreen,lowRed), cv::Scalar(highBlue,highGreen,highRed),
//                                    robots );

    //  RESETAR OS ROBOS
    //                Robot *robot = controlInterface.getRobots()->at ( i );
    ////                robot->center = cv::Point2f ( -1, -1 ); // reseta a posição do robô

    //    cv::inRange(imgHsv, cv::Scalar ( 90, 210, 210  ), cv::Scalar ( 110, 255, 255 ), imgThreshBlue );
    //    cv::inRange(imgHsv, cv::Scalar ( 130, 120, 120 ), cv::Scalar ( 210, 205, 255  ), imgThreshRed );


    //*********************************************************************************

//    imageAnalyzis.detectRobots( image, cv::Scalar ( 90, 210, 210  ), cv::Scalar ( 110, 255, 255 ), controlInterface.getRobots()->at ( 0 ) );   //  azul
//    imageAnalyzis.detectRobots( image, cv::Scalar ( 130, 120, 120 ), cv::Scalar ( 210, 205, 255  ), controlInterface.getRobots()->at ( 1 ) );  //  verm
//    imageAnalyzis.detectRobots( image, cv::Scalar ( 55, 120, 100 ), cv::Scalar ( 100, 205, 190  ), controlInterface.getRobots()->at ( 2 ) );   //  verde

    //*********************************************************************************

//    for ( int i = 0; i < controlInterface.getRobots()->size(); i++ )
//    {
//        Robot *robot = robots.at ( i );
//        qDebug() << i << robot->center.x << robot->center.y << robot->direction << robot->direction * 180 / M_PI;

//    }



    for ( int i = 0; i < numberOfLines; i++ )
    {
        for ( int j = 0; j < numberOfCols; j++ )
        {
            cv::Point2f circleCenter ( controlInterface.getEnvironment()->states [ i ][ j ]->xInit, controlInterface.getEnvironment()->states [ i ][ j ]->yInit );
            for ( int k = 0; k < controlInterface.getRobots()->size(); k++ )
            {
                Robot *robot = controlInterface.getRobots()->at ( k );
                if ( robot->center.x != -1 && robot->center.y != -1
                     && robot->getDistanceTo ( circleCenter ) <= circleRadius )
                {
                    robot->setPosEnvironment ( QPoint ( j, i ) );
                    controlInterface.getEnvironment()->setRobot ( i, j, robot->getId() );
//                    qDebug() << "setando como robo " << robot->getPosRowEnvironment () << robot->getPosColEnvironment ();
//                    controlInterface.getEnvironment()->states [ i ][ j ]->isRobot = true;
//                    qDebug() << "robot " << k << " pos: " << i << j << controlInterface.getEnvironment()->setRobot ( i, j ) <<
//                                robot->getPosRowEnvironment () << robot->getPosColEnvironment ();
                }
            }


        }
    }
//    cv::imshow ( "image", image );
    repaint();
}

void CVImageWidget::getNewImage()
{
    cv::Mat image;
    if ( controlInterface.getImageProcessing()->readNewImage( image ) )
        this->showImage ( image );
    else
        qDebug() << "CVImageWidget::getNewImage() -> não conseguiu adquirir nova imagem";
}

void CVImageWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.drawImage(QPoint(0,0), qimage);

//    qDebug() << "CVImageWidget::paintEvent(QPaintEvent *)" <<
//                "\ndistanceBetweenLinesX" << distanceBetweenLinesX <<
//                "\ndistanceBetweenLinesY" << distanceBetweenLinesY <<
//                "\n initX" << initX <<
//                "\n initY" << initY <<
//                "\n finalX" << finalX <<
//                "\n finalY" << finalY <<
//                "\n widthOfLine" << widthOfLine <<
//                "\n numberOfCols" << numberOfCols <<
//                "\n numberOfLines" << numberOfLines;
    int aux = 30;
    for ( int j = 0; j < numberOfLines; j++ )
    {
        QRect rect ( initX - aux,  distanceBetweenLinesY * j + initY, finalX, widthOfLine );
        painter.fillRect( rect, QColor ( 0, 0, 0, controlInterface.getAlphaColor() )/*Qt::SolidPattern*/ );
    }

    for ( int j = 0; j < numberOfCols; j++ )
    {
        QRect rect ( distanceBetweenLinesX * j + initX, initY - aux, widthOfLine, finalY );
        painter.fillRect( rect, QColor ( 0, 0, 0, controlInterface.getAlphaColor() )/*Qt::SolidPattern*/ );
    }

    for ( int k = 0; k < controlInterface.getRobots()->size(); k++ )
    {
        Robot *robot = controlInterface.getRobots()->at ( k );
        QColor arrowColor = robot->getRobotColor();
//        if ( robot->getId() == RED_ROBOT_NUMBER )
//            arrowColor = RED_ROBOT;
//        else if ( robot->getId() == GREEN_ROBOT_NUMBER )
//            arrowColor = GREEN_ROBOT;
//        else if ( robot->getId() == BLUE_ROBOT_NUMBER )
//            arrowColor = BLUE_ROBOT;

        //        if ( robot->getPosRowEnvironment () != -1 )
        //        {
        //            if ( robot->getId() == ORANGE_ROBOT_NUMBER )
        //            {
        //                qDebug()    << controlInterface.getEnvironment()->states [ robot->posEnvironment.y() ][ robot->posEnvironment.x() ]->yInit
        //                            << controlInterface.getEnvironment()->states [ robot->posEnvironment.y() ][ robot->posEnvironment.x() ]->xInit;
        //            }
        //            painterArrow( painter,
        //                          (qreal)controlInterface.getEnvironment()->states [ robot->posEnvironment.y() ][ robot->posEnvironment.x() ]->xInit,
        //                          (qreal)controlInterface.getEnvironment()->states [ robot->posEnvironment.y() ][ robot->posEnvironment.x() ]->yInit,
        //                          (qreal)( CIRCLE_RADIUS * 2 * cos ( robot->direction ) ),
        //                          (qreal)( CIRCLE_RADIUS * 2 * sin ( robot->direction ) ), arrowColor );
        //        }
        if ( robot->getPosRowEnvironment () != -1 )
            painterArrow( painter, (qreal)robot->center.x, (qreal)robot->center.y,
                          (qreal)( MAX_DIST_CENTERS * 0.5 * cos ( robot->direction ) ),
                          (qreal)( MAX_DIST_CENTERS * 0.5 * sin ( robot->direction ) ), arrowColor );
//        if ( robot->getPosRowEnvironment () != -1 )
//            painterArrow( painter, (qreal)robot->center.x, (qreal)robot->center.y,
//                          (qreal)( CIRCLE_RADIUS * cos ( robot->direction ) ),
//                          (qreal)( CIRCLE_RADIUS * sin ( robot->direction ) ), arrowColor );
//        if ( robot->getPosRowEnvironment () != -1 )
//            qDebug() << "k = " << k << robot->direction << robot->direction * 180 / M_PI;
//            qDebug() << "robot" << robot->getPosRowEnvironment () << robot->getPosColEnvironment () <<
//                     controlInterface.getEnvironment()->states [ robot->getPosRowEnvironment () ][ robot->getPosColEnvironment () ]->isRobot;
    }


    painter.setRenderHint(QPainter::Antialiasing, true);
    for ( int i = 0; i < numberOfLines; i++ )
    {
        for ( int j = 0; j < numberOfCols; j++ )
        {
            QColor color;
            /*if ( controlInterface.getEnvironment()->states [ i ][ j ]->isStart )
                color = START_COLOR;
            else */
            if ( controlInterface.getEnvironment()->states [ i ][ j ]->isGoal )
                color = GOAL_COLOR;
            else if ( controlInterface.getEnvironment()->states [ i ][ j ]->isObstacle )
                color = OBSTACLE_COLOR;
            else if ( controlInterface.getEnvironment()->states [ i ][ j ]->isPath )
                color = PATH_COLOR;
            else if ( controlInterface.getEnvironment()->states [ i ][ j ]->isEmpty )
                color = EMPTY_COLOR;
            else if ( controlInterface.getEnvironment()->states [ i ][ j ]->isRobot)
            {
                for ( int k = RED_ROBOT_NUMBER; k <= ORANGE_ROBOT_NUMBER; k++ )
                {
                    if ( controlInterface.getEnvironment()->states [ i ][ j ]->idRobot == k )
                    {
                        Robot *robot = controlInterface.getRobotById ( controlInterface.getEnvironment()->states [ i ][ j ]->idRobot );
                        if ( robot != NULL )
                            color = robot->getRobotColor();
                        else
                            "void CVImageWidget::paintEvent(QPaintEvent *) -> ERRO: ROBOT == NULL";
                    }
                }
//                if ( controlInterface.getEnvironment()->states [ i ][ j ]->idRobot == BLUE_ROBOT_NUMBER )
//                {
//                    color = BLUE_ROBOT;
//                }
//                else if ( controlInterface.getEnvironment()->states [ i ][ j ]->idRobot == RED_ROBOT_NUMBER )
//                {
//                    color = RED_ROBOT;
//                }
//                else if ( controlInterface.getEnvironment()->states [ i ][ j ]->idRobot == GREEN_ROBOT_NUMBER )
//                {
//                    color = GREEN_ROBOT;
//                }
            }

//            QPen pen( Qt::black, 2 );
            QPen pen ( QColor ( 0, 0, 0, controlInterface.getAlphaColor() ), 2 );
            painter.setPen ( pen );
            QBrush brush ( color );
            painter.setBrush ( brush );
            QPointF center ( controlInterface.getEnvironment()->states [ i ][ j ]->xInit, controlInterface.getEnvironment()->states [ i ][ j ]->yInit );
            painter.drawEllipse ( center, circleRadius, circleRadius );
        }
    }

    painter.end();
}

void CVImageWidget::painterArrow(QPainter &painter, qreal x, qreal y, qreal u, qreal v, QColor arrowColor )
{
    QPen pen = painter.pen();
    QPen newPen ( arrowColor );
    painter.setPen ( newPen );
    QPointF pt1,pt2;
    double theta;

    if ( u == 0 )
        theta = M_PI/2;
    else
        theta = atan2 ( double ( v ), double ( u ) );

//    pt1.x() = x;
//    pt1.y() = y;
    pt1 = QPointF ( x, y );

//    pt2.x() = x + u;
//    pt2.y() = y + v;
    pt2 = QPointF ( x + u, y + v );

    painter.drawLine( pt1, pt2 );

    double size = 15;

    if ( theta == M_PI/2 && pt1.y() > pt2.y() )
    {
        pt1 = QPointF ( (size*cos(theta)-size*sin(theta)+pt2.x()), (size*sin(theta)+size*cos(theta)+pt2.y()) );
        painter.drawLine(pt1,pt2);

        pt1 = QPointF ( (size*cos(theta)+size*sin(theta)+pt2.x()), (size*sin(theta)-size*cos(theta)+pt2.y()) );
        painter.drawLine(pt1,pt2);
    }
    else
    {
        pt1= QPointF ( (-size*cos(theta)-size*sin(theta)+pt2.x()),(-size*sin(theta)+size*cos(theta)+pt2.y()) );
        painter.drawLine(pt1,pt2);

        pt1 = QPointF ( (-size*cos(theta)+size*sin(theta)+pt2.x()), (-size*sin(theta)-size*cos(theta)+pt2.y()));
        painter.drawLine ( pt1, pt2);
    }
    painter.setPen ( pen );
}

void CVImageWidget::setNumberOfLines( int value )
{
//    qDebug() << "CVImageWidget::setNumberOfLines(int)" << value;

    numberOfLines = value;
    controlInterface.setNumberOfLines ( numberOfLines );
    controlInterface.resetRobots();
    controlInterface.getEnvironment()->updateStates( numberOfLines, numberOfCols );

    this->updateEnvironment();
}

void CVImageWidget::setNumberOfCols(int value)
{
//    qDebug() << "CVImageWidget::setNumberOfCols(int)" << value;
    numberOfCols = value;
    controlInterface.setNumberOfCols ( numberOfCols );
    controlInterface.resetRobots();
    controlInterface.getEnvironment()->updateStates( numberOfLines, numberOfCols );
    this->updateEnvironment();

}

void CVImageWidget::setInitX(int value)
{
//    qDebug() << "CVImageWidget::setInitX(int)" << value;
    initX = value;
    this->updateEnvironment();
}

void CVImageWidget::setInitY(int value)
{
//    qDebug() << "CVImageWidget::setInitY(int)" << value;
    initY = value;
    this->updateEnvironment();
}

void CVImageWidget::setFinalX(int value)
{
//    qDebug() << "CVImageWidget::setFinalX(int)" << value;
    finalX = value;
    this->updateEnvironment();
}

void CVImageWidget::setFinalY(int value)
{
//    qDebug() << "CVImageWidget::setFinalY(int)" << value;
    finalY = value;
    this->updateEnvironment();
}

void CVImageWidget::setWidthOfLine(int value)
{
//    qDebug() << "CVImageWidget::setWidthOfLine(int)" << value;
    widthOfLine = value;
    this->updateEnvironment();
}

void CVImageWidget::setCircleRadius(int value)
{
//    qDebug() << "CVImageWidget::setCircleRadius(int)" << value;
    circleRadius = value;
}

void CVImageWidget::setDistanceBetweenLinesX(int value)
{
//    qDebug() << "CVImageWidget::setDistanceBetweenLinesX(int)" << value;
    distanceBetweenLinesX = value;
    this->updateEnvironment();
}

void CVImageWidget::setDistanceBetweenLinesY(int value)
{
//    qDebug() << "CVImageWidget::setDistanceBetweenLinesY(int)" << value;
    distanceBetweenLinesY = value;
    this->updateEnvironment();
}
