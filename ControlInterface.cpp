#include "ControlInterface.h"
#include <cstdlib>
#include <ctime>
#include <utility>      // std::pair, std::make_pair
#include <string>       // std::string
#include <iostream>     // std::cout

int contador = 0;
#define MSG_SIZE    5

ControlInterface::ControlInterface(QObject *parent) :
    QObject(parent)
{
    srand ( time ( NULL ) );
    showSegmentation = false;
    selectedRobot = RED_ROBOT_NUMBER;
    saveSegmentationOfImage = false;
    //  inicialização dos robôs
//    for ( int i = 0; i < NUMBER_ROBOTS; i++ )
//    {
//        robots.push_back ( new Robot );
//    }
//    robots.push_back ( new Robot ( RED_ROBOT, RED_ROBOT_NUMBER ) );
//    robots.push_back ( new Robot ( BLUE_ROBOT, BLUE_ROBOT_NUMBER ) );
//    robots.push_back ( new Robot ( GREEN_ROBOT, GREEN_ROBOT_NUMBER ) );
//    robots.push_back ( new Robot ( YELLOW_ROBOT, YELLOW_ROBOT_NUMBER ) );
//    robots.push_back ( new Robot ( MAGENTA_ROBOT, MAGENTA_ROBOT_NUMBER ) );
//    robots.push_back ( new Robot ( QColor ( 130, 0, 150 ), PURPLE_ROBOT_NUMBER ) );   //  roxo = purple
//    robots.push_back ( new Robot ( QColor ( 255, 120, 0 ), ORANGE_ROBOT_NUMBER ) ); //  laranja

    alphaColor = 100;

    robots.push_back ( new Robot ( QColor ( 255,0,0, alphaColor )/*RED_ROBOT*/, RED_ROBOT_NUMBER ) );
    robots.push_back ( new Robot ( QColor ( 0,0,255, alphaColor )/*BLUE_ROBOT*/, BLUE_ROBOT_NUMBER ) );
    robots.push_back ( new Robot ( QColor ( 0,255,0, alphaColor )/*GREEN_ROBOT*/, GREEN_ROBOT_NUMBER ) );
    robots.push_back ( new Robot ( QColor ( 255,255,0, alphaColor )/*YELLOW_ROBOT*/, YELLOW_ROBOT_NUMBER ) );
    robots.push_back ( new Robot ( QColor ( 242,22,82, alphaColor )/*MAGENTA_ROBOT*/, MAGENTA_ROBOT_NUMBER ) );
    robots.push_back ( new Robot ( QColor ( 130, 0, 150, alphaColor ), PURPLE_ROBOT_NUMBER ) );   //  roxo = purple
    robots.push_back ( new Robot ( QColor ( 255, 120, 0, alphaColor ), ORANGE_ROBOT_NUMBER ) ); //  laranja
    port = NULL;
    openSerialPort ( "/dev/ttyACM0" );
}

void ControlInterface::setNumberOfLines(int nLines)
{
    numberOfLines = nLines;
}

void ControlInterface::setNumberOfCols(int nCols)
{
    numberOfCols = nCols;
}

ImageProcessing *ControlInterface::getImageProcessing()
{
    return &imageProcessing;
}

Environment *ControlInterface::getEnvironment()
{
    return &environment;
}

vector<Robot *> *ControlInterface::getRobots()
{
    return &robots;
}

vector<char> ControlInterface::executeAStar( int &robotNumber )
{
    vector <char> seqCom;
    if (  environment.goal == NULL )
    {
        return seqCom;
    }

    int row, col;
    environment.getGoal ( row, col );
    point_t goal = { row, col };

    vector < pair < Robot *, pair < vector<point_t>*, vector <char> > > > robotPathSeq;
    for ( int k = 0; k < robots.size(); k++ )
    {
        Robot *robot = robots.at ( k );
        if ( robot->getPosColEnvironment() == -1 && robot->getPosColEnvironment() == -1 )
            continue;
        vector<point_t>* res = auxExecuteAStar ( robot );
        if ( res != NULL )
        {
            res->insert( res->begin(), goal );
            vector <char> resSeq = generateSeqMov( res, robot );
            robotPathSeq.push_back ( make_pair( robot, make_pair ( res, resSeq ) ) );
        }
    }

    vector<point_t>* shortestPath = NULL;
    Robot *nearest = NULL;

//    qDebug() << robotPathSeq.size();
    for ( int k = 0; k < robotPathSeq.size(); k++ )
    {
        pair < vector<point_t>*, vector <char> > aux = robotPathSeq.at( k ).second;
//        qDebug() << aux.first->size() << aux.second.size();
        if  ( nearest == NULL )
        {
            nearest = robotPathSeq.at( k ).first;
            shortestPath = aux.first;
            seqCom = aux.second;
        }
        else
        {

            if ( aux.first->size() < shortestPath->size() )
            {
                nearest = robotPathSeq.at( k ).first;
                shortestPath = aux.first;
                seqCom = aux.second;
            }
            else if ( aux.first->size() == shortestPath->size() )
            {
                /*
                  se o tamanho do caminho for igual, compara o número de ações a serem executadas, a fim
                  de escolher o com o menor número de ações
                  */
                if ( aux.second.size() < seqCom.size() )
                {
                    nearest = robotPathSeq.at( k ).first;
                    shortestPath = aux.first;
                    seqCom = aux.second;
                }
            }
        }
    }

    if ( shortestPath == NULL )
    {
        return seqCom;
    }

    for ( int i = shortestPath->size() - 1; i > 0 ; i-- )
    {
        point_t p = shortestPath->at ( i );
        environment.states [ p.x ][ p.y ]->isPath = true;
    }
    robotNumber = nearest->getId();

    return seqCom;
}

void ControlInterface::resetRobots()
{
//    qDebug() << "void ControlInterface::resetRobots()";
    //  reseta todos os robôs
    for ( int k = 0; k < robots.size(); k++ )
    {
        Robot *robot = robots.at ( k );
//        qDebug() << "robot: " << robot->getId() << robot->getPosColEnvironment() << robot->getPosRowEnvironment();
        if ( robot->getPosRowEnvironment () != -1 && robot->getPosColEnvironment() != -1 )
        {
            environment.setEmpty ( robot->getPosRowEnvironment (), robot->getPosColEnvironment () );
//            qDebug() << "setando como empty " << robot->getPosRowEnvironment () << robot->getPosColEnvironment ();
        }
        robot->center = cv::Point2f ( -1, -1 ); // reseta a posição do robô
        robot->setPosEnvironment ( QPoint ( -1, -1 ) );
//        qDebug() << "\tnovo robot: " << robot->getId() << robot->getPosColEnvironment() << robot->getPosRowEnvironment();
    }
}


void ControlInterface::setMinRadiusRobots(int minRadius)
{
    minRadiusRobot = minRadius;
}

void ControlInterface::setMaxRadiusRobots(int maxRadius)
{
    maxRadiusRobot = maxRadius;
}

void ControlInterface::setMaxDistCenters(int maxDist)
{
    maxDistCenters = maxDist;
}

void ControlInterface::sendSerial ( vector < char > bytesToSend )
{
    QByteArray bytesToSendSerial;
    bytesToSendSerial.push_back ( START_C );
    bytesToSendSerial.push_back ( ESP_C );
    bytesToSendSerial.push_back ( bytesToSend.size() );
    qDebug() << "numero do robo: " << (int)bytesToSend.at ( 0 );
    qDebug() << "tamanho do pacote de dados:" << bytesToSend.size();
    for ( int i = 0; i < bytesToSend.size(); i++ )
    {
        bytesToSendSerial.push_back ( bytesToSend.at ( i ) );
        qDebug() << bytesToSend.at ( i );
    }
    bytesToSendSerial.push_back ( ESP_C );
    bytesToSendSerial.push_back ( END_C );
    qDebug() << ( this->sendDataSerialPort ( bytesToSendSerial ) ? "enviou com sucesso" : "erro" );
}

void ControlInterface::sendSerial()
{
    qDebug() << "enviando comando pela serial";

//    int numberOfCommands = rand () % 10 + 3;
////    char msg [ numberOfCommands ];
//    vector <char> msg;
//    msg.push_back( (char)(numberOfCommands) );
//    msg.push_back( (char)(rand () % NUMBER_ROBOTS) );
//    qDebug() << 0 << "\t" << (int)msg.at ( 0 );
//    qDebug() << 1 << "\t" << (int)msg.at ( 1 );
//    for ( int i = 2; i < numberOfCommands; i++ )
//    {
//        msg.push_back ( (char)(rand () % 26 + 65 ) );
//        qDebug() << i << "\t" << msg.at ( i );
//    }

//    this->sendDataSerialPort ( msg );
    QByteArray bytesToSendSerial;
    vector <char> teste;
    //teste.push_back ( (char)7 );
    teste.push_back ( START_C );
    teste.push_back ( ESP_C );
    teste.push_back ( 3 );
    teste.push_back ( 'E' );
    teste.push_back ( 'F' );
    teste.push_back ( 'F' );
    teste.push_back ( ESP_C );
    teste.push_back ( END_C );

    for ( int i = 0; i < teste.size(); i++ )
    {
        bytesToSendSerial.push_back( (char)teste.at ( i ) );
        qDebug() << ((char)teste.at ( i ));
    }

//    qDebug() << ( this->sendDataSerialPort ( teste ) ? "enviou com sucesso" : "erro" );
    qDebug() << ( this->sendDataSerialPort ( bytesToSendSerial ) ? "enviou com sucesso" : "erro" ) << contador++;

//    qDebug() << msg;
}

void ControlInterface::onReadyRead()
{
//    return;
//    qDebug() << "bytesReceivedSerial.size()" << bytesReceivedSerial.size();
    QByteArray bytes;
    int bytesAval = port->bytesAvailable();
    if ( bytesAval >= 3 )
    {
        bytes.resize ( bytesAval );
        port->read ( bytes.data(), bytes.size() );
        QString str ( bytes );
        if ( str == "fim" )
        {
            qDebug() << "Recebeu resposta positiva";
            emit receiveResponse ();
        }
    }
//    qDebug() << "bytes read:" << bytes.size();
//    qDebug() << "bytes:" << bytes;
//    qDebug() << "recebi:";
//    for ( int i = 0; i < bytes.size(); i++ )
//    {
//        qDebug() << bytes.at ( i );
//        bytesReceivedSerial.push_back ( bytes.at ( i ) );
//    }

//    int countAux = 0;
//    while ( bytesReceivedSerial.size() >= MSG_SIZE && countAux++ < 100 )
//    {
//        qDebug() << "bytesReceivedSerial.size()" << bytesReceivedSerial.size();
//        readPacket();
//    }
//    readPacket();

//    if ( bytesReceivedSerial.size() >= MSG_SIZE )
//        onReadyRead ();

}

void ControlInterface::onDsrChanged(bool status)
{
    if (status)
        qDebug() << "device was turned on";
    else
        qDebug() << "device was turned off";
}

void ControlInterface::restartEnvironment()
{
    environment.restartEnvironment();
}

void ControlInterface::setAlphaColor(int value)
{
    alphaColor = value;
    for ( int k = 0; k < robots.size(); k++ )
    {
        Robot *robot = robots.at ( k );
        QColor color = robot->getRobotColor();
        color.setAlpha ( alphaColor );
        robot->setRobotColor ( color );
    }
}

//bool ControlInterface::sendDataSerialPort ( vector<char> bytesToSend )
//{
//    char str [ bytesToSend.size() + 1 ];
//    for ( int i = 0; i < bytesToSend.size(); i++ )
//        str [ i ] = bytesToSend.at ( i );
//    str [ bytesToSend.size() ] = '\0';
//    QByteArray bytes ( str, bytesToSend.size() + 1 );

//    return port->write ( bytes ) > 0 ? true : false;
//}

bool ControlInterface::sendDataSerialPort ( QByteArray &bytesToSend )
{
    return ( port->write ( bytesToSend ) > 0 ? true : false );
}

void ControlInterface::readPacket()
{
    qDebug() << "readPacket()";
    int posFound = -1;
    int pktSize = -1;
//    qDebug() << bytesReceivedSerial [ 0 ] << START_C;
    for ( int i = 0; i < bytesReceivedSerial.size(); i++ )
    {
        qDebug() << bytesReceivedSerial [ i ] ;
        if (  bytesReceivedSerial [ i ] == START_C &&
        i + 1 < bytesReceivedSerial.size() &&
        bytesReceivedSerial [ i + 1 ] == ESP_C )
        {

            int dataSize =  bytesReceivedSerial[ i + 2 ];
//                qDebug() << "dataSize" << dataSize;
            vector <char> dataBuffer;
            if ( i + dataSize + MSG_SIZE <= bytesReceivedSerial.size() )
            {
//                    qDebug() << "\tentrei";
                if (  bytesReceivedSerial [ i + dataSize + 3 ] == ESP_C &&
                bytesReceivedSerial [ i + dataSize + 4 ] == END_C )
                {
                    for ( int j = 0; j < dataSize; j++ )
                    {
//                            qDebug() << "\t\t" << j;
                        dataBuffer.push_back( bytesReceivedSerial [ i + j + 3 ] );
//                            qDebug() << "\t\t\t" <<
//                                        (j == 0 ? (int)dataBuffer.at ( j ) : (char)dataBuffer.at ( j ));
                    }
//                        qDebug() << "dataBuffer.size()" << dataBuffer.size() << dataSize;
//                    cout << (int)dataBuffer.at ( 0 ) << endl;
//                    cout << "\t";
                    for ( int j = 0; j < dataBuffer.size(); j++ )
                        cout << dataBuffer.at ( j );
//                    cout << endl;
                    pktSize = MSG_SIZE + dataSize;
//                        qDebug() << "pktSize" << pktSize;
                    posFound = i;
                    break;
                }
            }
            else
                break;
        }
    }


    if ( posFound > -1 )
    {
//        qDebug() << "apagando:" << posFound << posFound + pktSize;
//        bytesReceivedSerial.erase ( bytesReceivedSerial.begin() + posFound,
//                                    bytesReceivedSerial.begin() + posFound + pktSize );
        bytesReceivedSerial.erase ( bytesReceivedSerial.begin(),
                                    bytesReceivedSerial.begin() + posFound + pktSize );
    }
}



bool ControlInterface::openSerialPort(const QString &portName)
{
    port = new QextSerialPort ( portName, QextSerialPort::EventDriven );
    port->setBaudRate(BAUD9600);
    port->setFlowControl(FLOW_OFF);
    port->setParity(PAR_NONE);
    port->setDataBits(DATA_8);
    port->setStopBits(STOP_1);

    if (port->open(QIODevice::ReadWrite) == true)
    {
        connect(port, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
        connect(port, SIGNAL(dsrChanged(bool)), this, SLOT(onDsrChanged(bool)));
        if (!(port->lineStatus() & LS_DSR))
            qDebug() << "warning: device is not turned on";
        qDebug() << "listening for data on" << port->portName();
    }
    else
    {
        qDebug() << "bool ControlInterface::openSerialPort(const QString &portName) -> " <<
                    "device failed to open:" << port->errorString();
    }
    return true;
}

int ControlInterface::getMinRadiusRobots()
{
    return minRadiusRobot;
}

int ControlInterface::getMaxRadiusRobots()
{
    return maxRadiusRobot;
}

int ControlInterface::getMaxDistCenters()
{
    return maxDistCenters;
}

void ControlInterface::detectRobots( cv::Mat &image )
{
    resetRobots ();
    cv::Mat results [ NUMBER_OF_RESULTS ];

//    qDebug() << showSegmentation << selectedRobot;
//    cv::destroyAllWindows();
//    qDebug() << "minRadiusRobot" << minRadiusRobot <<
//                "maxRadiusRobot" << maxRadiusRobot <<
//                "maxDistCenters" << maxDistCenters;
//    qDebug() << redRobotLowRange [ 0 ] << redRobotLowRange [ 1 ] << redRobotLowRange [ 2 ];

    for ( int i = RED_ROBOT_NUMBER; i <= ORANGE_ROBOT_NUMBER; i++ )
    {
        Robot *robot = robots.at ( i );
        imageProcessing.detectRobots(   image,
                                        imageProcessing.getSystemColorRgb() ? robot->getLowRangeRgb() : robot->getLowRangeHsv(),
                                        imageProcessing.getSystemColorRgb() ? robot->getHighRangeRgb() : robot->getHighRangeHsv(),
                                        robot, minRadiusRobot, maxRadiusRobot, maxDistCenters,
                                        ( showSegmentation && selectedRobot == i ) ? results : NULL );  //  verm
    }
//    Robot *robot = robots.at ( 0 );
//    imageProcessing.detectRobots(   image,
//                                    imageProcessing.getSystemColorRgb() ? robot->getLowRangeRgb() : robot->getLowRangeHsv(),
//                                    imageProcessing.getSystemColorRgb() ? robot->getHighRangeRgb() : robot->getHighRangeHsv(),
//                                    robot, minRadiusRobot, maxRadiusRobot, maxDistCenters,
//                                    ( showSegmentation && selectedRobot == RED_ROBOT_NUMBER ) ? results : NULL );  //  verm

//    robot = robots.at ( 1 );
//    imageProcessing.detectRobots( image,
//                                  imageProcessing.getSystemColorRgb() ? robot->getLowRangeRgb() : robot->getLowRangeHsv(),
//                                  imageProcessing.getSystemColorRgb() ? robot->getHighRangeRgb() : robot->getHighRangeHsv(),
//                                  robot, minRadiusRobot, maxRadiusRobot, maxDistCenters,
//                                  ( showSegmentation && selectedRobot == BLUE_ROBOT_NUMBER ) ? results : NULL  );   //  azul

//    robot = robots.at ( 2 );
//    imageProcessing.detectRobots( image,
//                                  imageProcessing.getSystemColorRgb() ? robot->getLowRangeRgb() : robot->getLowRangeHsv(),
//                                  imageProcessing.getSystemColorRgb() ? robot->getHighRangeRgb() : robot->getHighRangeHsv(),
//                                  robot, minRadiusRobot, maxRadiusRobot, maxDistCenters,
//                                  ( showSegmentation && selectedRobot == GREEN_ROBOT_NUMBER ) ? results : NULL  );   //  verde
    if ( showSegmentation )
    {
        cv::imshow ( "image", results [ 0 ] );
        cv::imshow ( "imgHsv", results [ 1 ] );
        cv::imshow ( "imgThresh", results [ 2 ] );
        cv::imshow ( "imageErosion", results [ 3 ] );
        cv::imshow ( "imageDilation", results [ 4 ] );
        cv::imshow ( "blur", results [ 5 ] );

        if ( saveSegmentationOfImage )
        {
            saveSegmentationOfImage = false;
            cv::imwrite ( "./imgs/image.png", results [ 0 ] );
            cv::imwrite ( "./imgs/imgHsv.png", results [ 1 ] );
            cv::imwrite ( "./imgs/imgThresh.png", results [ 2 ] );
            cv::imwrite ( "./imgs/imageErosion.png", results [ 3 ] );
            cv::imwrite ( "./imgs/imageDilation.png", results [ 4 ] );
            cv::imwrite ( "./imgs/blur.png", results [ 5 ] );
        }
    }
//    imageProcessing.detectRobots( image, cv::Scalar ( 130, 120, 120 ), cv::Scalar ( 210, 205, 255  ),
//                                  robots.at ( 0 ), minRadiusRobot, maxRadiusRobot, maxDistCenters );  //  verm
//    imageProcessing.detectRobots( image, cv::Scalar ( 90, 210, 210  ), cv::Scalar ( 110, 255, 255 ),
//                                  robots.at ( 1 ), minRadiusRobot, maxRadiusRobot, maxDistCenters );   //  azul
//    imageProcessing.detectRobots( image, cv::Scalar ( 55, 120, 100 ), cv::Scalar ( 100, 205, 190  ),
//                                  robots.at ( 2 ), minRadiusRobot, maxRadiusRobot, maxDistCenters );   //  verde

//    for ( int i = 0; i < numberOfLines; i++ )
//    {
//        for ( int j = 0; j < numberOfCols; j++ )
//        {
//            cv::Point2f circleCenter ( environment.states [ i ][ j ]->xInit, environment.states [ i ][ j ]->yInit );
//            for ( int k = 0; k < robots.size(); k++ )
//            {
//                Robot *robot = robot->at ( k );
//                if ( robot->center.x != -1 && robot->center.y != -1
//                     && robot->getDistanceTo ( circleCenter ) <= circleRadius )
//                {
//                    robot->setPosEnvironment ( QPoint ( j, i ) );
//                    environment.setRobot ( i, j, robot->getId() );
//                }
//            }
//        }
    //    }
}

void ControlInterface::saveSegmentation()
{
    if ( showSegmentation )
    {
        saveSegmentationOfImage = true;
    }
}

void ControlInterface::updateSelectedRobot(int value)
{
//    qDebug() << "ControlInterface::updateSelectedRobot(int value)" << value;
//    selectedRobot = value;
    //  a ordem do valor de value deve ser igual ao que consta no comboBoxSelectRobot
    switch ( value )
    {
        case 0:
            selectedRobot = RED_ROBOT_NUMBER;
            break;
        case 1:
            selectedRobot = BLUE_ROBOT_NUMBER;
            break;
        case 2:
            selectedRobot = GREEN_ROBOT_NUMBER;
            break;
        case 3:
            selectedRobot = YELLOW_ROBOT_NUMBER;
            break;
        case 4:
            selectedRobot = MAGENTA_ROBOT_NUMBER;
            break;
        case 5:
            selectedRobot = PURPLE_ROBOT_NUMBER;
            break;
        case 6:
            selectedRobot = ORANGE_ROBOT_NUMBER;
            break;
    }
}

vector<point_t> *ControlInterface::auxExecuteAStar(Robot *robot)
{
//    qDebug() << "vector<point_t> *ControlInterface::auxExecuteAStar(Robot *robot)";
    point_t dim = { numberOfLines, numberOfCols };
    Mapa* mapa = new Mapa(dim);
    vector <point_t> barreiras;

//    environment.getStart ( row, col );
    point_t inicio = { robot->getPosRowEnvironment(), robot->getPosColEnvironment() };
    int row, col;
    environment.getGoal ( row, col );
    point_t fim = { row, col };

//    qDebug() << "\nnumberOfLines" << numberOfLines <<
//                "\nnumberOfCols" << numberOfCols <<
//                "\ninicio.x" << inicio.x <<
//                "\ninicio.y" << inicio.y <<
//                "\ngoal row" << row <<
//                "\ngoal col" << col <<
//                "\nrobot" << robot <<
//                "\nrobot" << robot->getId();
//    switch ( robot->getId() )
//    {
//    case RED_ROBOT_NUMBER:
//        qDebug() << "robo VERMELHO";
//        break;
//    case BLUE_ROBOT_NUMBER:
//        qDebug() << "robo AZUL";
//        break;
//    case GREEN_ROBOT_NUMBER:
//        qDebug() << "robo VERDE";
//        break;
//    }

    for ( int i = 0; i < numberOfLines; i++ )
    {
        for ( int j = 0; j < numberOfCols; j++ )
        {
//            qDebug() << i << j;
//            if ( environment.states [ i ][ j ]->isObstacle )
            if ( !environment.states [ i ][ j ]->isEmpty && ( i != robot->getPosRowEnvironment() ||
                                                                j != robot->getPosColEnvironment() ) )
            {
                point_t p = { i, j };
                barreiras.push_back(p);
            }
        }
    }
    mapa->setDimensao(dim);
    mapa->setInicio(inicio);
    mapa->setObjetivo(fim);
    mapa->setObstaculos(&barreiras);
    return mapa->a_star();
}

vector<char> ControlInterface::generateSeqMov(vector<point_t> *shortestPath, Robot *nearest)
{
    vector <char> seqCom;
    point_t p = shortestPath->at ( shortestPath->size() -1 );
    double robotDirectionDeg = nearest->direction * 180 / M_PI;
    double firstMovDeg = atan2 ( p.x - nearest->getPosRowEnvironment(), p.y - nearest->getPosColEnvironment() ) *
            180 / M_PI;

    if ( robotDirectionDeg < 0 )
        robotDirectionDeg += 360;
    if ( firstMovDeg < 0 )
        firstMovDeg += 360;

//    qDebug() << "ANTIGO " << robotDirectionDeg << firstMovDeg;

    if ( robotDirectionDeg <= 45 || robotDirectionDeg > 315 )
        robotDirectionDeg = 0;
    else if ( robotDirectionDeg > 45 && robotDirectionDeg <= 135 )
        robotDirectionDeg = 90;
    else if ( robotDirectionDeg > 135 && robotDirectionDeg <= 225 )
        robotDirectionDeg = 180;
    else if ( robotDirectionDeg > 225 && robotDirectionDeg <= 315 )
        robotDirectionDeg = 270;

//    qDebug() << "novo robotDirectionDeg" << robotDirectionDeg;
    switch ( (int)(robotDirectionDeg - firstMovDeg) )
    {
    case -90:
    case 270:
        seqCom.push_back ( 'D' );
        break;
    case 90:
    case -270:
        seqCom.push_back ( 'E' );
        break;
    case 180:
    case -180:
        seqCom.push_back ( 'D' );
        seqCom.push_back ( 'D' );
        break;
    }
    seqCom.push_back ( 'F' );

//    qDebug() << "antes de comecar o loop" << robotDirectionDeg << firstMovDeg <<
//                nearest->getPosRowEnvironment() << nearest->getPosColEnvironment();
    robotDirectionDeg = firstMovDeg;
    point_t newState = {p.x, p.y};

    for ( int i = shortestPath->size() - 2; i > 0 ; i-- )
    {
        point_t np = shortestPath->at ( i );
        firstMovDeg = atan2 ( np.x - newState.x, np.y - newState.y ) * 180 / M_PI;

        if ( firstMovDeg < 0 )
            firstMovDeg += 360;
//        qDebug() << "no loop i: " << i <<
//                    "\nrobotDirectionDeg " << robotDirectionDeg <<
//                    "newState.x" << newState.x <<
//                    "newState.y" << newState.y <<
//                    "firstMovDeg" << firstMovDeg << "\n";

        switch ( (int)(robotDirectionDeg - firstMovDeg) )
        {
        case -90:
        case 270:
            seqCom.push_back ( 'D' );
            break;
        case 90:
        case -270:
            seqCom.push_back ( 'E' );
            break;
        }
        seqCom.push_back ( 'F' );

        newState = {np.x, np.y};
        robotDirectionDeg = firstMovDeg;
    }

//    QString str;
//    for ( int i = 0; i < seqCom.size(); i++ )
//    {
//        str.append( QChar ( (char)seqCom.at ( i )) );
//        str.append ( '-' );
////        qDebug() << (char)seqCom.at ( i ) << "-";
//    }
//    QMessageBox::information( this, "Path", str );
    return seqCom;
}

void ControlInterface::setShowSegmentation(int value )
{
    if ( value == Qt::Checked )
        showSegmentation = true;
    else if ( value == Qt::Unchecked )
    {
        showSegmentation = false;
        cv::destroyAllWindows();
    }

    //    qDebug() << "CVImageWidget::setShowSegmentation(int value )" << value << showSegmentation;
}

Robot *ControlInterface::getRobotById(int robotId )
{
//    return robots.at ( robotNumber );
    for ( int k = 0; k < robots.size(); k++ )
    {
        Robot *robot = robots.at ( k );
        if ( robot->getId() == robotId )
            return robot;
    }
    return NULL;
}

Robot *ControlInterface::getRobotByComboBoxNumber(int comboBoxNumber)
{
    if ( comboBoxNumber >= robots.size() )
    {
        qDebug() << "Robot *ControlInterface::getRobotByComboBoxNumber(int comboBoxNumber) -> ERRO###############";
        return NULL;
    }
    else
        return robots.at ( comboBoxNumber );
}

int ControlInterface::getAlphaColor()
{
    return alphaColor;
}
