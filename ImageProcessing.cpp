#include "ImageProcessing.h"
#include "Util.h"
#include <QPainter>
#include "Robot.h"

ImageProcessing::ImageProcessing(QObject *parent) :
    QObject(parent)
{
    cap.open ( 0 );
//    cap.open ( "video1-1.ogv" );
    if ( !cap.isOpened() )
    {
        qDebug() << "CVImageWidget::CVImageWidget -> falha ao tentar abrir camera 0";
        exit ( 0 );
//        cap.open ( 0 );
//        if ( !cap.isOpened() )
//        {
//            qDebug() << "CVImageWidget::CVImageWidget -> falha ao tentar abrir camera 0 -> o programa sera finalizado";
//            exit ( 0 );
//        }
    }

    systemColorRgb = true;
}

bool ImageProcessing::readNewImage( cv::Mat &image )
{

#ifdef DEBUG_SYSTEM_COLOR
    image = cv::imread ( "./imgs/image.png" );
//    image = cv::imread ( "/home/odin/Desktop/UTFPR/VisaoComputacional/Aula2/image.png" );
#else
    if ( !cap.read(image) )
    {
        qDebug() << "CVImageWidget::getNewImage -> falha ao tentar ler nova imagem";
        return false;
    }
#endif


    return true;
}

/*
  desenha uma flecha na matriz image
  */
void ImageProcessing::drawArrow(cv::Mat image, int x, int y, int u, int v, cv::Scalar color, int size, int thickness)
{
    cv::Point pt1,pt2;
    double theta;

    if ( u == 0 )
        theta = M_PI/2;
    else
        theta = atan2 ( double ( v ), double ( u ) );

    pt1.x = x;
    pt1.y = y;

    pt2.x = x + u;
    pt2.y = y + v;

    cv::line ( image, pt1, pt2, color, thickness, 8 );

    size = (int)( size * 0.707 );

    if ( theta == M_PI/2 && pt1.y > pt2.y )
    {
        pt1.x = (int)(size*cos(theta)-size*sin(theta)+pt2.x);
        pt1.y = (int)(size*sin(theta)+size*cos(theta)+pt2.y);
        cv::line (image,pt1,pt2,color,thickness,8);

        pt1.x=(int)(size*cos(theta)+size*sin(theta)+pt2.x);
        pt1.y=(int)(size*sin(theta)-size*cos(theta)+pt2.y);
        cv::line(image,pt1,pt2,color,thickness,8);
    }
    else
    {
        pt1.x=(int)(-size*cos(theta)-size*sin(theta)+pt2.x);
        pt1.y=(int)(-size*sin(theta)+size*cos(theta)+pt2.y);
        cv::line(image,pt1,pt2,color,thickness,8);

        pt1.x=(int)(-size*cos(theta)+size*sin(theta)+pt2.x);
        pt1.y=(int)(-size*sin(theta)-size*cos(theta)+pt2.y);
        cv::line ( image, pt1, pt2, color, thickness, 8 );
    }
}

void ImageProcessing::detectRobots(cv::Mat image, cv::Scalar lowRange, cv::Scalar highRange,
           Robot *robot,
           int minRadiusRobot, int maxRadiusRobot, int maxDistCenters, cv::Mat *results )
{

    cv::Mat frameBlur;
    cv::GaussianBlur(image, frameBlur, cv::Size ( 5, 5 ), 1 );
    cv::Mat imgThresh;
    cv::Mat imgHsv;/* ( frameBlur.rows, frameBlur.cols, CV_8UC3 );*/
    cv::cvtColor( frameBlur, imgHsv, CV_BGR2HSV );
    if ( !systemColorRgb )
    {
        int from_to[] = {0,0, 1,1};
        cv::Mat hs(frameBlur.size(), CV_8UC2);
        cv::mixChannels(&imgHsv, 1, &hs, 1, from_to, 2);
        cv::Scalar newLowRange = lowRange;
        newLowRange [ 0 ] /= 2;
        newLowRange [ 1 ] = m_map ( newLowRange [ 1 ], 0, 100, 0, 255 );

        cv::Scalar newHighRange = highRange;
        newHighRange [ 0 ] /= 2;
        newHighRange [ 1 ] = m_map ( newHighRange [ 1 ], 0, 100, 0, 255 );
        cv::inRange(hs, newLowRange, newHighRange, imgThresh );
    }
    else
    {
        cv::inRange(frameBlur, lowRange, highRange, imgThresh );
    }

//    cv::GaussianBlur(imgThresh, imgThresh, cv::Size ( 3, 3 ), 1 );


    cv::Mat imageErosion;
    cv::Mat imageDilation;

    cv::Mat st_elem = getStructuringElement ( cv::MORPH_RECT, cv::Size( 6, 6 ) );
    cv::erode ( imgThresh, imageErosion, st_elem );
    st_elem = getStructuringElement ( cv::MORPH_RECT, cv::Size( 5, 5 ) );
    cv::dilate ( imageErosion, imageDilation, st_elem );

    cv::Mat matContours;
    imageDilation.copyTo ( matContours );

    std::vector<std::vector<cv::Point> > contours;
    std::vector< MyCircle * > detectedCircles;

    cv::findContours(matContours,
        contours, // vector que conterá os contornos encontrados
        CV_RETR_LIST,
        CV_CHAIN_APPROX_NONE);

//    qDebug() << "contours.size()" << contours.size();
    for ( int i = 0; i < contours.size(); i++ )
    {
        MyCircle *mc = new MyCircle;
        cv::minEnclosingCircle(cv::Mat(contours.at ( i )),mc->center,mc->radius);
        if ( mc->radius > minRadiusRobot  && mc->radius < maxRadiusRobot )
            detectedCircles.push_back( mc );
    }
//    qDebug() << detectedCircles.size();

    if ( detectedCircles.size() >= 2 )
    {
        //  ordena os circulos por tamanho (decrescente) de raio
        auxSort4 auxsort4;
        std::sort ( detectedCircles.begin(), detectedCircles.end(), auxsort4 );

        //  ordena os circulos por distancia ao circulo de maior raio (crescente)
        auxSort2 auxsort2;
        vector <MyCircle *> auxDetectedCircles = detectedCircles;
        auxsort2.mainCenter = detectedCircles.at ( 0 )->center; //  pega o maior circulo

        // ordena um vetor auxiliar com base na distancia ao centro de maior raio
        std::sort ( auxDetectedCircles.begin(), auxDetectedCircles.end(), auxsort2 );


//        for ( int i = 0; i < detectedCircles.size(); i++ )
//            cv::circle ( image, detectedCircles.at ( i )->center, detectedCircles.at ( i )->radius, cv::Scalar ( 255, 255, 255 ) );

        int pos1 = 0;
        int pos2 = 0;

        if ( sqrt ( pow ( detectedCircles.at( pos1 )->center.x -
                          auxDetectedCircles.at ( pos2 )->center.x, 2 ) +
                    pow ( detectedCircles.at( pos1 )->center.y -
                          auxDetectedCircles.at ( pos2 )->center.y, 2 ) ) <
             maxDistCenters )
        {

            cv::Point2f auxPoint;
            auxPoint.x = ( detectedCircles.at ( pos1 )->center.x +
                           auxDetectedCircles.at ( pos2 )->center.x ) / 2;
            auxPoint.y = ( detectedCircles.at ( pos1 )->center.y +
                           auxDetectedCircles.at ( pos2 )->center.y ) / 2;

//            robot->center = detectedCircles.at ( pos1 )->center; // forma 1
            robot->center = auxPoint; // forma 2
            robot->direction = atan2 ( auxDetectedCircles.at( pos2 )->center.y - detectedCircles.at ( pos1 )->center.y,
                                       auxDetectedCircles.at( pos2 )->center.x - detectedCircles.at ( pos1 )->center.x );
        }
    }

    if ( results != NULL )
    {
        results [ 0 ] = image;
        results [ 1 ] = imgHsv;
        results [ 2 ] = imgThresh;
        results [ 3 ] = imageErosion;
        results [ 4 ] = imageDilation;
        results [ 5 ] = frameBlur;
    }
}

string ImageProcessing::getImgType(int imgTypeInt)
{
    int numImgTypes = 35; // 7 base types, with five channel options each (none or C1, ..., C4)

    int enum_ints[] =       {CV_8U,  CV_8UC1,  CV_8UC2,  CV_8UC3,  CV_8UC4,
                             CV_8S,  CV_8SC1,  CV_8SC2,  CV_8SC3,  CV_8SC4,
                             CV_16U, CV_16UC1, CV_16UC2, CV_16UC3, CV_16UC4,
                             CV_16S, CV_16SC1, CV_16SC2, CV_16SC3, CV_16SC4,
                             CV_32S, CV_32SC1, CV_32SC2, CV_32SC3, CV_32SC4,
                             CV_32F, CV_32FC1, CV_32FC2, CV_32FC3, CV_32FC4,
                             CV_64F, CV_64FC1, CV_64FC2, CV_64FC3, CV_64FC4};

    string enum_strings[] = {"CV_8U",  "CV_8UC1",  "CV_8UC2",  "CV_8UC3",  "CV_8UC4",
                             "CV_8S",  "CV_8SC1",  "CV_8SC2",  "CV_8SC3",  "CV_8SC4",
                             "CV_16U", "CV_16UC1", "CV_16UC2", "CV_16UC3", "CV_16UC4",
                             "CV_16S", "CV_16SC1", "CV_16SC2", "CV_16SC3", "CV_16SC4",
                             "CV_32S", "CV_32SC1", "CV_32SC2", "CV_32SC3", "CV_32SC4",
                             "CV_32F", "CV_32FC1", "CV_32FC2", "CV_32FC3", "CV_32FC4",
                             "CV_64F", "CV_64FC1", "CV_64FC2", "CV_64FC3", "CV_64FC4"};

    for(int i=0; i<numImgTypes; i++)
    {
        if(imgTypeInt == enum_ints[i]) return enum_strings[i];
    }
    return "unknown image type";
}

void ImageProcessing::setSystemColorRgb(bool sysColor)
{
    systemColorRgb = sysColor;
}

bool ImageProcessing::getSystemColorRgb()
{
    return systemColorRgb;
}
