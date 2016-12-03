#ifndef CONTROLINTERFACE_H
#define CONTROLINTERFACE_H

#include <QObject>
#include <opencv2/opencv.hpp>
#include "ImageProcessing.h"
#include "Environment.h"
#include "Robot.h"
#include "mapa.h"
#include <QSettings>
#include "qextserialport.h"

#define START_C 0xAA
#define ESP_C   0xAB
#define END_C   0xFF

class ControlInterface : public QObject
{
    Q_OBJECT
public:
    explicit ControlInterface(QObject *parent = 0);
    void setNumberOfLines ( int nLines );
    void setNumberOfCols ( int nCols );
    ImageProcessing *getImageProcessing ();
    Environment *getEnvironment ();
    vector <Robot *> *getRobots ();
    vector<char> executeAStar(int &robotNumber );
    void resetRobots ();

    int getMinRadiusRobots ();
    int getMaxRadiusRobots ();
    int getMaxDistCenters ();

    void detectRobots (cv::Mat &image);

//    bool sendDataSerialPort ( vector <char> bytesToSend );
    bool sendDataSerialPort ( QByteArray &bytesToSend );

    void readPacket ();

    Robot *getRobotById ( int robotId );
    Robot *getRobotByComboBoxNumber ( int comboBoxNumber );

    int getAlphaColor ();

signals:
    void receiveResponse ();
public slots:
    void saveSegmentation ();
    void updateSelectedRobot ( int value );
    void setShowSegmentation ( int value);
    void setMinRadiusRobots ( int minRadius );
    void setMaxRadiusRobots ( int maxRadius );
    void setMaxDistCenters ( int maxDist );

    void sendSerial ();
    void sendSerial ( vector < char > bytesToSend );
    //  Slots para leitura da serial
    void onReadyRead();
    void onDsrChanged(bool status);

    void restartEnvironment ();

    void setAlphaColor ( int value );

    
private:
    ImageProcessing imageProcessing;
    Environment environment;
    std::vector <Robot *> robots;
    int numberOfLines;
    int numberOfCols;
    int minRadiusRobot;
    int maxRadiusRobot;
    int maxDistCenters;
    bool showSegmentation;
    int selectedRobot;
    bool saveSegmentationOfImage;

    QextSerialPort *port;
    vector <unsigned char> bytesReceivedSerial;
    bool openSerialPort ( const QString &portName );

    int alphaColor;

    vector<point_t> *auxExecuteAStar( Robot *robot );
    vector<char> generateSeqMov( vector<point_t>* shortestPath, Robot *nearest );
};

#endif // CONTROLINTERFACE_H
