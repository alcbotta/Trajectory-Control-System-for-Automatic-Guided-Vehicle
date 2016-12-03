#ifndef UTIL_H
#define UTIL_H
#pragma once

#ifndef FORCE_UTIL_H
#define FORCE_UTIL_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QDebug>
#include <iostream>
#include <ctype.h>
#include <vector>
#include <queue>
#include <algorithm>
#include <QColor>
#include <string>
using namespace std;


#define NUMBER_ROBOTS       7
#define NUMBER_COLS         5
#define NUMBER_ROWS         5
#define MIN_RADIUS_ROBOT    7
#define MAX_RADIUS_ROBOT    30
#define MAX_DIST_CENTERS    100
#define CIRCLE_RADIUS       30

#define GOAL_COLOR          Qt::darkCyan
#define OBSTACLE_COLOR      Qt::black
#define PATH_COLOR          Qt::darkYellow
#define EMPTY_COLOR         Qt::transparent
#define BLUE_ROBOT          Qt::blue
#define RED_ROBOT           Qt::red
#define GREEN_ROBOT         Qt::green
#define YELLOW_ROBOT        Qt::yellow
#define MAGENTA_ROBOT       Qt::magenta
//  QT NÃO POSSUI A DEFINIÇÃO DAS CORES ROXO E LARANJA

//#define RED_ROBOT_NUMBER        0
//#define BLUE_ROBOT_NUMBER       1
//#define GREEN_ROBOT_NUMBER      2
//#define YELLOW_ROBOT_NUMBER     3
//#define MAGENTA_ROBOT_NUMBER    4
//#define PURPLE_ROBOT_NUMBER     5
//#define ORANGE_ROBOT_NUMBER     6

enum RobotsNumber
{
    RED_ROBOT_NUMBER,
    BLUE_ROBOT_NUMBER,
    GREEN_ROBOT_NUMBER,
    YELLOW_ROBOT_NUMBER,
    MAGENTA_ROBOT_NUMBER,
    PURPLE_ROBOT_NUMBER,
    ORANGE_ROBOT_NUMBER
 };

#define FUNDO_BRANCO

#ifdef FUNDO_BRANCO
//    #define RED_ROBOT_LOW_RED       80
//    #define RED_ROBOT_LOW_GREEN     0
//    #define RED_ROBOT_LOW_BLUE      0
//    #define RED_ROBOT_HIGH_RED      120
//    #define RED_ROBOT_HIGH_GREEN    30
//    #define RED_ROBOT_HIGH_BLUE     60

//    #define BLUE_ROBOT_LOW_RED      0
//    #define BLUE_ROBOT_LOW_GREEN    40
//    #define BLUE_ROBOT_LOW_BLUE     100
//    #define BLUE_ROBOT_HIGH_RED     20
//    #define BLUE_ROBOT_HIGH_GREEN   70
//    #define BLUE_ROBOT_HIGH_BLUE    140

//    #define GREEN_ROBOT_LOW_RED     0
//    #define GREEN_ROBOT_LOW_GREEN   20
//    #define GREEN_ROBOT_LOW_BLUE    0
//    #define GREEN_ROBOT_HIGH_RED    30
//    #define GREEN_ROBOT_HIGH_GREEN  60
//    #define GREEN_ROBOT_HIGH_BLUE   50

//    #define YELLOW_ROBOT_LOW_RED     125
//    #define YELLOW_ROBOT_LOW_GREEN   100
//    #define YELLOW_ROBOT_LOW_BLUE    0
//    #define YELLOW_ROBOT_HIGH_RED    205
//    #define YELLOW_ROBOT_HIGH_GREEN  170
//    #define YELLOW_ROBOT_HIGH_BLUE   10

//    #define MAGENTA_ROBOT_LOW_RED     100
//    #define MAGENTA_ROBOT_LOW_GREEN   0
//    #define MAGENTA_ROBOT_LOW_BLUE    30
//    #define MAGENTA_ROBOT_HIGH_RED    140
//    #define MAGENTA_ROBOT_HIGH_GREEN  40
//    #define MAGENTA_ROBOT_HIGH_BLUE   70

//    #define PURPLE_ROBOT_LOW_RED     20
//    #define PURPLE_ROBOT_LOW_GREEN   30
//    #define PURPLE_ROBOT_LOW_BLUE    80
//    #define PURPLE_ROBOT_HIGH_RED    60
//    #define PURPLE_ROBOT_HIGH_GREEN  80
//    #define PURPLE_ROBOT_HIGH_BLUE   130

//    #define ORANGE_ROBOT_LOW_RED     120
//    #define ORANGE_ROBOT_LOW_GREEN   20
//    #define ORANGE_ROBOT_LOW_BLUE    0
//    #define ORANGE_ROBOT_HIGH_RED    190
//    #define ORANGE_ROBOT_HIGH_GREEN  80
//    #define ORANGE_ROBOT_HIGH_BLUE   10

    #define RED_ROBOT_LOW_RED       244
    #define RED_ROBOT_LOW_GREEN     244
    #define RED_ROBOT_LOW_BLUE      244
    #define RED_ROBOT_HIGH_RED      245
    #define RED_ROBOT_HIGH_GREEN    245
    #define RED_ROBOT_HIGH_BLUE     245

    #define BLUE_ROBOT_LOW_RED      244
    #define BLUE_ROBOT_LOW_GREEN    244
    #define BLUE_ROBOT_LOW_BLUE     244
    #define BLUE_ROBOT_HIGH_RED     245
    #define BLUE_ROBOT_HIGH_GREEN   245
    #define BLUE_ROBOT_HIGH_BLUE    245

    #define GREEN_ROBOT_LOW_RED     244
    #define GREEN_ROBOT_LOW_GREEN   244
    #define GREEN_ROBOT_LOW_BLUE    244
    #define GREEN_ROBOT_HIGH_RED    245
    #define GREEN_ROBOT_HIGH_GREEN  245
    #define GREEN_ROBOT_HIGH_BLUE   245

    #define YELLOW_ROBOT_LOW_RED     100
    #define YELLOW_ROBOT_LOW_GREEN   80
    #define YELLOW_ROBOT_LOW_BLUE    0
    #define YELLOW_ROBOT_HIGH_RED    210
    #define YELLOW_ROBOT_HIGH_GREEN  180
    #define YELLOW_ROBOT_HIGH_BLUE   10

    #define MAGENTA_ROBOT_LOW_RED     244
    #define MAGENTA_ROBOT_LOW_GREEN   244
    #define MAGENTA_ROBOT_LOW_BLUE    244
    #define MAGENTA_ROBOT_HIGH_RED    245
    #define MAGENTA_ROBOT_HIGH_GREEN  245
    #define MAGENTA_ROBOT_HIGH_BLUE   245

    #define PURPLE_ROBOT_LOW_RED     244
    #define PURPLE_ROBOT_LOW_GREEN   244
    #define PURPLE_ROBOT_LOW_BLUE    244
    #define PURPLE_ROBOT_HIGH_RED    245
    #define PURPLE_ROBOT_HIGH_GREEN  245
    #define PURPLE_ROBOT_HIGH_BLUE   245

    #define ORANGE_ROBOT_LOW_RED     100
    #define ORANGE_ROBOT_LOW_GREEN   20
    #define ORANGE_ROBOT_LOW_BLUE    0
    #define ORANGE_ROBOT_HIGH_RED    200
    #define ORANGE_ROBOT_HIGH_GREEN  90
    #define ORANGE_ROBOT_HIGH_BLUE   15


    #define RED_ROBOT_LOW_H         0
    #define RED_ROBOT_LOW_S         91
    #define RED_ROBOT_HIGH_H        3
    #define RED_ROBOT_HIGH_S        100

    #define BLUE_ROBOT_LOW_H        190
    #define BLUE_ROBOT_LOW_S        85
    #define BLUE_ROBOT_HIGH_H       219
    #define BLUE_ROBOT_HIGH_S       100

    #define GREEN_ROBOT_LOW_H       36
    #define GREEN_ROBOT_LOW_S       41
    #define GREEN_ROBOT_HIGH_H      198
    #define GREEN_ROBOT_HIGH_S      99

    #define YELLOW_ROBOT_LOW_H      48
    #define YELLOW_ROBOT_LOW_S      98
    #define YELLOW_ROBOT_HIGH_H     54
    #define YELLOW_ROBOT_HIGH_S     100

    #define MAGENTA_ROBOT_LOW_H     322
    #define MAGENTA_ROBOT_LOW_S     60
    #define MAGENTA_ROBOT_HIGH_H    60
    #define MAGENTA_ROBOT_HIGH_S    100

    #define PURPLE_ROBOT_LOW_H      215
    #define PURPLE_ROBOT_LOW_S      40
    #define PURPLE_ROBOT_HIGH_H     250
    #define PURPLE_ROBOT_HIGH_S     70

    #define ORANGE_ROBOT_LOW_H      7
    #define ORANGE_ROBOT_LOW_S      99
    #define ORANGE_ROBOT_HIGH_H     20
    #define ORANGE_ROBOT_HIGH_S     100

#elif defined FUNDO_CINZA
    //  AMBIENTE CINZA
    #define RED_ROBOT_LOW_BLUE      0
    #define RED_ROBOT_LOW_GREEN     190
    #define RED_ROBOT_LOW_RED       200
    #define RED_ROBOT_HIGH_BLUE     10
    #define RED_ROBOT_HIGH_GREEN    255

    #define RED_ROBOT_HIGH_RED      255

    #define BLUE_ROBOT_LOW_BLUE     90
    #define BLUE_ROBOT_LOW_GREEN    150
    #define BLUE_ROBOT_LOW_RED      155
    #define BLUE_ROBOT_HIGH_BLUE    110
    #define BLUE_ROBOT_HIGH_GREEN   200
    #define BLUE_ROBOT_HIGH_RED     255

    #define GREEN_ROBOT_LOW_BLUE    30
    #define GREEN_ROBOT_LOW_GREEN   70
    #define GREEN_ROBOT_LOW_RED     90
    #define GREEN_ROBOT_HIGH_BLUE   70
    #define GREEN_ROBOT_HIGH_GREEN  190
    #define GREEN_ROBOT_HIGH_RED    210

#else
    //  DEFAULT: SEM NADA
    #define RED_ROBOT_LOW_BLUE      130
    #define RED_ROBOT_LOW_GREEN     120
    #define RED_ROBOT_LOW_RED       120
    #define RED_ROBOT_HIGH_BLUE     210
    #define RED_ROBOT_HIGH_GREEN    205
    #define RED_ROBOT_HIGH_RED      255
    #define BLUE_ROBOT_LOW_BLUE     90
    #define BLUE_ROBOT_LOW_GREEN    210
    #define BLUE_ROBOT_LOW_RED      210
    #define BLUE_ROBOT_HIGH_BLUE    110
    #define BLUE_ROBOT_HIGH_GREEN   255
    #define BLUE_ROBOT_HIGH_RED     255

    #define GREEN_ROBOT_LOW_BLUE    55
    #define GREEN_ROBOT_LOW_GREEN   120
    #define GREEN_ROBOT_LOW_RED     100
    #define GREEN_ROBOT_HIGH_BLUE   100
    #define GREEN_ROBOT_HIGH_GREEN  205
    #define GREEN_ROBOT_HIGH_RED    200

    #define RED_ROBOT_LOW_H         312
    #define RED_ROBOT_LOW_S         29
    #define RED_ROBOT_HIGH_H        360
    #define RED_ROBOT_HIGH_S        70

    #define BLUE_ROBOT_LOW_H        192
    #define BLUE_ROBOT_LOW_S        60
    #define BLUE_ROBOT_HIGH_H       223
    #define BLUE_ROBOT_HIGH_S       100

    #define GREEN_ROBOT_LOW_H       128
    #define GREEN_ROBOT_LOW_S       52
    #define GREEN_ROBOT_HIGH_H      190
    #define GREEN_ROBOT_HIGH_S      81
#endif

//#define RED_ROBOT_LOW_H         160
//#define RED_ROBOT_LOW_S         125
//#define RED_ROBOT_HIGH_H        180
//#define RED_ROBOT_HIGH_S        177

//#define BLUE_ROBOT_LOW_H        70
//#define BLUE_ROBOT_LOW_S        203
//#define BLUE_ROBOT_HIGH_H       120
//#define BLUE_ROBOT_HIGH_S       255

//#define GREEN_ROBOT_LOW_H       64
//#define GREEN_ROBOT_LOW_S       154
//#define GREEN_ROBOT_HIGH_H      95
//#define GREEN_ROBOT_HIGH_S      211

double m_map(double x, double in_min, double in_max, double out_min, double out_max);

//  estrutura auxiliar para representar um circulo
struct MyCircle
{
    cv::Point2f center;
    float radius;
};

//  estrutura auxiliar para fazer a ordenação de um vetor de acordo com o tamanho do raio dos elementos do mesmo
typedef struct auxSort
{
    bool operator() (std::vector<cv::Point> c1, std::vector<cv::Point> c2)
    {
        cv::Point2f center;
        float radius1, radius2;
        cv::minEnclosingCircle(c1,center,radius1);
        cv::minEnclosingCircle(c2,center,radius2);
        return radius1 > radius2;
    }
}auxSort;

//  estrutura auxiliar para fazer a ordenação de um vetor de acordo com a distância do centros dos elementos do mesmo
//  em relação a um ponto comum
typedef struct auxSort2
{
    cv::Point2f mainCenter;
    bool operator() (MyCircle *p1, MyCircle *p2)
    {
        double dist1 = sqrt ( pow ( mainCenter.x - p1->center.x, 2  ) + pow ( mainCenter.y - p1->center.y, 2  ) );
        double dist2 = sqrt ( pow ( mainCenter.x - p2->center.x, 2  ) + pow ( mainCenter.y - p2->center.y, 2  ) );
        return dist1 > dist2;
    }
}auxSort2;

//  estrutura auxiliar para fazer a ordenação de um vetor de acordo com o tamanho do raio dos elementos do mesmo
typedef struct auxSort3
{
    bool operator() (MyCircle *p1, MyCircle *p2)
    {
        return p1->radius > p2->radius;
    }
}auxSort3;

typedef struct auxSort4
{
    bool operator() (MyCircle *p1, MyCircle *p2)
    {
        return p1->radius > p2->radius;
    }
}auxSort4;


#endif // FORCE_UTIL_H
#endif // UTIL_H
