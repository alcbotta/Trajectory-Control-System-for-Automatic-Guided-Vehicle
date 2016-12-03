#include "CaptionWidget.h"
#include <QStaticText>
#include "Util.h"

CaptionWidget::CaptionWidget(QWidget *parent) :
    QWidget(parent)
{
}

void CaptionWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    int circleRadius = 20;
    int initX = 25;
    int initXStr = initX + 30;
    int initY = 25;
    QPen pen( Qt::black, 2 );
    painter.setPen ( pen );
    QBrush brush ( GOAL_COLOR );
    painter.setBrush ( brush );
    QPointF center ( initX, initY );
    painter.drawEllipse ( center, circleRadius, circleRadius );

    QStaticText stringLabel ( "Objetivo" );
    painter.drawStaticText ( initXStr, initY - circleRadius / 2, stringLabel );

    initY += 2 * circleRadius + 10;
    center = QPointF ( initX, initY );
    brush = QBrush ( OBSTACLE_COLOR );
    painter.setBrush ( brush );
    stringLabel = QStaticText ( "Obstaculo" );
    painter.drawEllipse ( center, circleRadius, circleRadius );
    painter.drawStaticText ( initXStr, initY - circleRadius / 2, stringLabel );

    initY += 2 * circleRadius + 10;
    center = QPointF ( initX, initY );
    brush = QBrush ( PATH_COLOR );
    painter.setBrush ( brush );
    stringLabel = QStaticText ( "Caminho" );
    painter.drawEllipse ( center, circleRadius, circleRadius );
    painter.drawStaticText ( initXStr, initY - circleRadius / 2, stringLabel );

    initY += 2 * circleRadius + 10;
    center = QPointF ( initX, initY );
    brush = QBrush ( EMPTY_COLOR );
    painter.setBrush ( brush );
    stringLabel = QStaticText ( "Estado vazio" );
    painter.drawEllipse ( center, circleRadius, circleRadius );
    painter.drawStaticText ( initXStr, initY - circleRadius / 2, stringLabel );

    painter.end();
}
