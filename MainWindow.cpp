#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "Robot.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    settings ( "Oficinas 3", "Projeto" )
{
    ui->setupUi(this);
    this->showMaximized();

    ui->widgetShowVideo->getControlInterface()->setAlphaColor ( ui->spinBoxAlphaColor->value() );

    ui->pushButtonSendSerial->setVisible ( false );

    connect ( ui->widgetShowVideo, SIGNAL ( newMousePos ( int, int, int, int, int ) ),
              this, SLOT ( mouseMoveImageWidget ( int, int, int, int, int ) ) );

//    connect ( this, SIGNAL ( destroy( bool, bool ) ), this, SLOT ( windowDestroyed () ) );
    connect ( this, SIGNAL ( destroyed () ), this, SLOT ( windowDestroyed () ) );

    connect ( ui->pushButtonRestartEnvironment, SIGNAL ( clicked ( bool ) ),
              ui->widgetShowVideo->getControlInterface(), SLOT ( restartEnvironment () ) );

    connect ( ui->radioButtonHsv, SIGNAL ( clicked ( bool ) ), this, SLOT ( typeOfSegmentation () ) );
    connect ( ui->radioButtonRgb, SIGNAL ( clicked ( bool ) ), this, SLOT ( typeOfSegmentation () ) );

    ui->spinBoxLowH->setEnabled ( false );
    ui->spinBoxLowS->setEnabled ( false );
    ui->spinBoxHighH->setEnabled ( false );
    ui->spinBoxHighS->setEnabled ( false );

//    qDebug() << ui->widgetShowVideo->width() << ui->widgetShowVideo->height();
    ui->spinBoxFinalX->setMaximum( ui->widgetShowVideo->width() );
    ui->spinBoxFinalY->setMaximum( ui->widgetShowVideo->height() );
//    ui->spinBoxNumberOfCols->setMinimum( NUMBER_COLS );
//    ui->spinBoxNumberOfLines->setMinimum( NUMBER_ROWS );
//    ui->widgetShowVideo->initEnvironment();

    connect ( ui->pushButtonCallAStar, SIGNAL ( clicked ( bool ) ),
              ui->widgetShowVideo, SLOT ( executeAStar ( ) ) );

    connect ( ui->pushButtonCallAStar, SIGNAL ( clicked ( bool ) ),
              this, SLOT ( blockExecuteAStar ( ) ) );

    connect ( ui->widgetShowVideo->getControlInterface(), SIGNAL ( receiveResponse () ),
              this, SLOT ( unblockExecuteAStar() ) );

    connect ( ui->pushButtonSendSerial, SIGNAL ( clicked ( bool ) ),
              ui->widgetShowVideo->getControlInterface(), SLOT ( sendSerial ( ) ) );

    connect ( ui->pushButtonSaveSegmentation, SIGNAL ( clicked ( bool ) ),
              ui->widgetShowVideo->getControlInterface(), SLOT ( saveSegmentation ( ) ) );

    //  Eventos da aba de Guide-path:

    connect ( ui->spinBoxAlphaColor, SIGNAL ( valueChanged ( int ) ),
              ui->widgetShowVideo->getControlInterface(), SLOT ( setAlphaColor ( int ) ) );

    connect ( ui->spinBoxNumberOfLines, SIGNAL ( valueChanged ( int ) ), ui->widgetShowVideo, SLOT ( setNumberOfLines ( int ) ) );
    connect ( ui->spinBoxNumberOfCols, SIGNAL ( valueChanged ( int ) ), ui->widgetShowVideo, SLOT ( setNumberOfCols ( int ) ) );


//    connect ( ui->spinBoxNumberOfLines, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setNumberOfLines ( int ) ) );
//    connect ( ui->spinBoxNumberOfCols, SIGNAL ( valueChanged ( int ) ), ui->widgetShowVideo, SLOT ( setNumberOfCols ( int ) ) );

    connect ( ui->spinBoxInitX, SIGNAL ( valueChanged ( int ) ), ui->widgetShowVideo, SLOT ( setInitX ( int ) ) );
    connect ( ui->spinBoxInitY, SIGNAL ( valueChanged ( int ) ), ui->widgetShowVideo, SLOT ( setInitY ( int ) ) );
    connect ( ui->spinBoxFinalX, SIGNAL ( valueChanged ( int ) ), ui->widgetShowVideo, SLOT ( setFinalX ( int ) ) );
    connect ( ui->spinBoxFinalY, SIGNAL ( valueChanged ( int ) ), ui->widgetShowVideo, SLOT ( setFinalY ( int ) ) );
    connect ( ui->spinBoxWidthOfLine, SIGNAL ( valueChanged ( int ) ), ui->widgetShowVideo, SLOT ( setWidthOfLine ( int ) ) );
    connect ( ui->spinBoxCircleRadius, SIGNAL ( valueChanged ( int ) ), ui->widgetShowVideo, SLOT ( setCircleRadius ( int ) ) );
    connect ( ui->spinBoxDistanceBetweenLinesX, SIGNAL ( valueChanged ( int ) ),
              ui->widgetShowVideo, SLOT ( setDistanceBetweenLinesX ( int ) ) );
    connect ( ui->spinBoxDistanceBetweenLinesY, SIGNAL ( valueChanged ( int ) ),
              ui->widgetShowVideo, SLOT ( setDistanceBetweenLinesY ( int ) ) );

    //  Eventos da aba de Segmentação:

    connect ( ui->comboBoxSelectRobot, SIGNAL ( currentIndexChanged ( int ) ),
              this, SLOT ( changeCurrentIndexSelectRobot ( int ) ) );

    connect ( ui->spinBoxLowBlue, SIGNAL ( valueChanged ( int ) ), this, SLOT ( changeColorRange ( ) ) );
    connect ( ui->spinBoxLowGreen, SIGNAL ( valueChanged ( int ) ), this, SLOT ( changeColorRange ( ) ) );
    connect ( ui->spinBoxLowRed, SIGNAL ( valueChanged ( int ) ), this, SLOT ( changeColorRange ( ) ) );
    connect ( ui->spinBoxHighBlue, SIGNAL ( valueChanged ( int ) ), this, SLOT ( changeColorRange ( ) ) );
    connect ( ui->spinBoxHighGreen, SIGNAL ( valueChanged ( int ) ), this, SLOT ( changeColorRange ( ) ) );
    connect ( ui->spinBoxHighRed, SIGNAL ( valueChanged ( int ) ), this, SLOT ( changeColorRange ( ) ) );

    connect ( ui->spinBoxHighH, SIGNAL ( valueChanged ( int ) ), this, SLOT ( changeColorRange ( ) ) );
    connect ( ui->spinBoxHighS, SIGNAL ( valueChanged ( int ) ), this, SLOT ( changeColorRange ( ) ) );
    connect ( ui->spinBoxLowH, SIGNAL ( valueChanged ( int ) ), this, SLOT ( changeColorRange ( ) ) );
    connect ( ui->spinBoxLowS, SIGNAL ( valueChanged ( int ) ), this, SLOT ( changeColorRange ( ) ) );

    connect ( ui->comboBoxSelectRobot, SIGNAL ( currentIndexChanged ( int ) ),
               ui->widgetShowVideo->getControlInterface(), SLOT ( updateSelectedRobot ( int ) ) );
    connect ( ui->checkBoxShowSegmentation, SIGNAL ( stateChanged ( int ) ),
            ui->widgetShowVideo->getControlInterface(), SLOT ( setShowSegmentation ( int ) ) );

    //  Eventos da aba de Identificação:
    connect ( ui->spinBoxRobotsRadiusMin, SIGNAL ( valueChanged ( int ) ),
              ui->widgetShowVideo->getControlInterface(), SLOT ( setMinRadiusRobots ( int ) ) );
    connect ( ui->spinBoxRobotsRadiusMax, SIGNAL ( valueChanged ( int ) ),
              ui->widgetShowVideo->getControlInterface(), SLOT ( setMaxRadiusRobots ( int ) ) );
    connect ( ui->spinBoxDistanceOfIdenticators, SIGNAL ( valueChanged ( int ) ),
              ui->widgetShowVideo->getControlInterface(), SLOT ( setMaxDistCenters ( int ) ) );



    this->updateSettings ();
    ui->widgetShowVideo->updateEnvironment();

    timerReadNewFrame = new QTimer(this);
    connect(timerReadNewFrame, SIGNAL(timeout()), ui->widgetShowVideo, SLOT(getNewImage()));
    timerReadNewFrame->start(1);

//    ui->widgetShowVideo->updateSettings( settings );

//    this->changeColorRange ();
}

MainWindow::~MainWindow()
{
//    cv::destroyAllWindows();
    delete ui;
}



void MainWindow::updateSettings()
{
    ui->spinBoxCircleRadius->setValue ( settings.value ( "gui/circleRadius", CIRCLE_RADIUS ).toInt() );
    ui->widgetShowVideo->setCircleRadius( settings.value ( "gui/circleRadius", CIRCLE_RADIUS ).toInt() );

    ui->spinBoxNumberOfLines->setValue ( settings.value ( "gui/numberOfLines", NUMBER_ROWS ).toInt() );
    ui->widgetShowVideo->setNumberOfLines ( settings.value ( "gui/numberOfLines", NUMBER_ROWS ).toInt() );
    ui->widgetShowVideo->getControlInterface()->setNumberOfLines ( settings.value ( "gui/numberOfLines", NUMBER_ROWS ).toInt() );

    ui->spinBoxNumberOfCols->setValue ( settings.value ( "gui/numberOfCols", NUMBER_COLS ).toInt() );
    ui->widgetShowVideo->setNumberOfCols ( settings.value ( "gui/numberOfCols", NUMBER_COLS ).toInt() );
    ui->widgetShowVideo->getControlInterface()->setNumberOfCols ( settings.value ( "gui/numberOfCols", NUMBER_COLS ).toInt() );


    int auxX = 37;
    int auxY = 36;

//    int aux = ui->widgetShowVideo->getCircleRadius ();
    ui->spinBoxInitX->setValue ( settings.value ( "gui/initX", auxX ).toInt() );
    ui->widgetShowVideo->setInitX ( settings.value ( "gui/initX", auxX ).toInt() );

    ui->spinBoxInitY->setValue ( settings.value ( "gui/initY", auxY ).toInt() );
    ui->widgetShowVideo->setInitY ( settings.value ( "gui/initY", auxY ).toInt() );

    ui->spinBoxDistanceBetweenLinesX->setValue ( settings.value (
        "gui/distaceBetweenLinesX", ui->widgetShowVideo->width() / ui->widgetShowVideo->getNumberOfCols() ).toInt() );
    ui->widgetShowVideo->setDistanceBetweenLinesX ( settings.value (
        "gui/distaceBetweenLinesX", ui->widgetShowVideo->width() / ui->widgetShowVideo->getNumberOfCols() ).toInt() );

    ui->spinBoxDistanceBetweenLinesY->setValue( settings.value (
        "gui/distaceBetweenLinesY", ui->widgetShowVideo->height() / ui->widgetShowVideo->getNumberOfLines() ).toInt() );
    ui->widgetShowVideo->setDistanceBetweenLinesY ( settings.value (
        "gui/distaceBetweenLinesY", ui->widgetShowVideo->height() / ui->widgetShowVideo->getNumberOfLines() ).toInt() );


    ui->spinBoxFinalX->setValue ( settings.value ( "gui/finalX", ui->widgetShowVideo->getInitX() +
        ( ui->widgetShowVideo->getNumberOfCols() - 1 ) *
        ui->widgetShowVideo->getDistanceBetweenLinesX() + 2 * AUX_GUIDE_PATH ).toInt() );
    ui->widgetShowVideo->setFinalX ( settings.value ( "gui/finalX", ui->widgetShowVideo->getInitX() +
        ( ui->widgetShowVideo->getNumberOfCols() - 1 ) *
        ui->widgetShowVideo->getDistanceBetweenLinesX() + 2 * AUX_GUIDE_PATH ).toInt() );

    ui->spinBoxFinalY->setValue ( settings.value ( "gui/finalY", ui->widgetShowVideo->getInitY() +
        ( ui->widgetShowVideo->getNumberOfLines() - 1 ) *
        ui->widgetShowVideo->getDistanceBetweenLinesY() + 2 * AUX_GUIDE_PATH ).toInt() );
    ui->widgetShowVideo->setFinalY ( settings.value ( "gui/finalY", ui->widgetShowVideo->getInitY() +
        ( ui->widgetShowVideo->getNumberOfLines() - 1 ) *
        ui->widgetShowVideo->getDistanceBetweenLinesY() + 2 * AUX_GUIDE_PATH ).toInt() );

    ui->spinBoxWidthOfLine->setValue ( settings.value ( "gui/widthOfLine", 10 ).toInt() );
    ui->widgetShowVideo->setWidthOfLine ( settings.value ( "gui/widthOfLine", 10 ).toInt() );

    ui->spinBoxDistanceOfIdenticators->setValue ( settings.value (
                                                      "gui/maxDistCenters", MAX_DIST_CENTERS ).toInt() );
    ui->widgetShowVideo->getControlInterface()->setMaxDistCenters ( settings.value (
                                                    "gui/maxDistCenters", MAX_DIST_CENTERS ).toInt() );

    ui->spinBoxRobotsRadiusMax->setValue ( settings.value ( "gui/maxRadiusRobots", MAX_RADIUS_ROBOT ).toInt() );
    ui->widgetShowVideo->getControlInterface()->setMaxRadiusRobots (
                settings.value ( "gui/maxRadiusRobots", MAX_RADIUS_ROBOT ).toInt() );

    ui->spinBoxRobotsRadiusMin->setValue ( settings.value ( "gui/minRadiusRobots", MIN_RADIUS_ROBOT ).toInt() );
    ui->widgetShowVideo->getControlInterface()->setMinRadiusRobots (
                settings.value ( "gui/minRadiusRobots", MIN_RADIUS_ROBOT ).toInt() );



    ui->spinBoxLowBlue->setValue ( settings.value ( "gui/redRobotLowBlue", RED_ROBOT_LOW_BLUE ).toInt() );
    ui->spinBoxLowGreen->setValue ( settings.value ( "gui/redRobotLowGreen", RED_ROBOT_LOW_GREEN ).toInt() );
    ui->spinBoxLowRed->setValue ( settings.value ( "gui/redRobotLowRed", RED_ROBOT_LOW_RED ).toInt() );
    ui->spinBoxHighBlue->setValue ( settings.value ( "gui/redRobotHighBlue", RED_ROBOT_HIGH_BLUE).toInt() );
    ui->spinBoxHighGreen->setValue ( settings.value ( "gui/redRobotHighGreen", RED_ROBOT_HIGH_GREEN ).toInt() );
    ui->spinBoxHighRed->setValue ( settings.value ( "gui/redRobotHighRed", RED_ROBOT_HIGH_RED ).toInt() );

    ui->spinBoxLowH->setValue ( settings.value ( "gui/redRobotLowH", RED_ROBOT_LOW_H ).toInt() );
    ui->spinBoxLowS->setValue ( settings.value ( "gui/redRobotLowS", RED_ROBOT_LOW_S ).toInt() );
    ui->spinBoxHighH->setValue ( settings.value ( "gui/redRobotHighH", RED_ROBOT_HIGH_H ).toInt() );
    ui->spinBoxHighS->setValue ( settings.value ( "gui/redRobotHighS", RED_ROBOT_HIGH_S ).toInt() );

    cv::Scalar redRobotLowRange ( settings.value ( "gui/redRobotLowBlue", RED_ROBOT_LOW_BLUE ).toInt(),
                                  settings.value ( "gui/redRobotLowGreen", RED_ROBOT_LOW_GREEN ).toInt(),
                                  settings.value ( "gui/redRobotLowRed", RED_ROBOT_LOW_RED ).toInt() );
    cv::Scalar redRobotHighRange ( settings.value ( "gui/redRobotHighBlue", RED_ROBOT_HIGH_BLUE).toInt(),
                                   settings.value ( "gui/redRobotHighGreen", RED_ROBOT_HIGH_GREEN ).toInt(),
                                   settings.value ( "gui/redRobotHighRed", RED_ROBOT_HIGH_RED ).toInt() );

    cv::Scalar redRobotLowRangeHsv ( settings.value ( "gui/redRobotLowH", RED_ROBOT_LOW_H ).toInt(),
                                     settings.value ( "gui/redRobotLowS", RED_ROBOT_LOW_S ).toInt() );

    cv::Scalar redRobotHighRangeHsv ( settings.value ( "gui/redRobotHighH", RED_ROBOT_HIGH_H ).toInt(),
                                     settings.value ( "gui/redRobotHighS", RED_ROBOT_HIGH_S ).toInt() );

//    ui->widgetShowVideo->getControlInterface()->setRedRobotRange ( redRobotLowRange, redRobotHighRange );
    Robot *robot = ui->widgetShowVideo->getControlInterface()->getRobots()->at ( 0 );
    robot->setRangeRgb ( redRobotLowRange, redRobotHighRange );
    robot->setRangeHsv ( redRobotLowRangeHsv, redRobotHighRangeHsv );

    cv::Scalar blueRobotLowRange ( settings.value ( "gui/blueRobotLowBlue", BLUE_ROBOT_LOW_BLUE ).toInt(),
                                  settings.value ( "gui/blueRobotLowGreen", BLUE_ROBOT_LOW_GREEN ).toInt(),
                                  settings.value ( "gui/blueRobotLowRed", BLUE_ROBOT_LOW_RED ).toInt() );
    cv::Scalar blueRobotHighRange ( settings.value ( "gui/blueRobotHighBlue", BLUE_ROBOT_HIGH_BLUE).toInt(),
                                   settings.value ( "gui/blueRobotHighGreen", BLUE_ROBOT_HIGH_GREEN ).toInt(),
                                   settings.value ( "gui/blueRobotHighRed", BLUE_ROBOT_HIGH_RED ).toInt() );

    cv::Scalar blueRobotLowRangeHsv ( settings.value ( "gui/blueRobotLowH", BLUE_ROBOT_LOW_H ).toInt(),
                                     settings.value ( "gui/blueRobotLowS", BLUE_ROBOT_LOW_S ).toInt() );

    cv::Scalar blueRobotHighRangeHsv ( settings.value ( "gui/blueRobotHighH", BLUE_ROBOT_HIGH_H ).toInt(),
                                     settings.value ( "gui/blueRobotHighS", BLUE_ROBOT_HIGH_S ).toInt() );
//    ui->widgetShowVideo->getControlInterface()->setBlueRobotRange ( blueRobotLowRange, blueRobotHighRange );
    robot = ui->widgetShowVideo->getControlInterface()->getRobots()->at ( 1 );
    robot->setRangeRgb ( blueRobotLowRange, blueRobotHighRange );
    robot->setRangeHsv ( blueRobotLowRangeHsv, blueRobotHighRangeHsv );

    cv::Scalar greenRobotLowRange ( settings.value ( "gui/greenRobotLowBlue", GREEN_ROBOT_LOW_BLUE ).toInt(),
                                  settings.value ( "gui/greenRobotLowGreen", GREEN_ROBOT_LOW_GREEN ).toInt(),
                                  settings.value ( "gui/greenRobotLowRed", GREEN_ROBOT_LOW_RED ).toInt() );
    cv::Scalar greenRobotHighRange ( settings.value ( "gui/greenRobotHighBlue", GREEN_ROBOT_HIGH_BLUE).toInt(),
                                   settings.value ( "gui/greenRobotHighGreen", GREEN_ROBOT_HIGH_GREEN ).toInt(),
                                   settings.value ( "gui/greenRobotHighRed", GREEN_ROBOT_HIGH_RED ).toInt() );

    cv::Scalar greenRobotLowRangeHsv ( settings.value ( "gui/greenRobotLowH", GREEN_ROBOT_LOW_H ).toInt(),
                                     settings.value ( "gui/greenRobotLowS", GREEN_ROBOT_LOW_S ).toInt() );

    cv::Scalar greenRobotHighRangeHsv ( settings.value ( "gui/greenRobotHighH", GREEN_ROBOT_HIGH_H ).toInt(),
                                     settings.value ( "gui/greenRobotHighS", GREEN_ROBOT_HIGH_S ).toInt() );
//    ui->widgetShowVideo->getControlInterface()->setGreenRobotRange ( greenRobotLowRange, greenRobotHighRange );
    robot = ui->widgetShowVideo->getControlInterface()->getRobots()->at ( 2 );
    robot->setRangeRgb ( greenRobotLowRange, greenRobotHighRange );
    robot->setRangeHsv ( greenRobotLowRangeHsv, greenRobotHighRangeHsv );


    cv::Scalar yellowRobotLowRange ( settings.value ( "gui/yellowRobotLowBlue", YELLOW_ROBOT_LOW_BLUE ).toInt(),
                                  settings.value ( "gui/yellowRobotLowGreen", YELLOW_ROBOT_LOW_GREEN ).toInt(),
                                  settings.value ( "gui/yellowRobotLowRed", YELLOW_ROBOT_LOW_RED ).toInt() );
    cv::Scalar yellowRobotHighRange ( settings.value ( "gui/yellowRobotHighBlue", YELLOW_ROBOT_HIGH_BLUE).toInt(),
                                   settings.value ( "gui/yellowRobotHighGreen", YELLOW_ROBOT_HIGH_GREEN ).toInt(),
                                   settings.value ( "gui/yellowRobotHighRed", YELLOW_ROBOT_HIGH_RED ).toInt() );

    cv::Scalar yellowRobotLowRangeHsv ( settings.value ( "gui/yellowRobotLowH", YELLOW_ROBOT_LOW_H ).toInt(),
                                     settings.value ( "gui/yellowRobotLowS", YELLOW_ROBOT_LOW_S ).toInt() );

    cv::Scalar yellowRobotHighRangeHsv ( settings.value ( "gui/yellowRobotHighH", YELLOW_ROBOT_HIGH_H ).toInt(),
                                     settings.value ( "gui/yellowRobotHighS", YELLOW_ROBOT_HIGH_S ).toInt() );
    robot = ui->widgetShowVideo->getControlInterface()->getRobots()->at ( 3 );
    robot->setRangeRgb ( yellowRobotLowRange, yellowRobotHighRange );
    robot->setRangeHsv ( yellowRobotLowRangeHsv, yellowRobotHighRangeHsv );


    cv::Scalar magentaRobotLowRange ( settings.value ( "gui/magentaRobotLowBlue", MAGENTA_ROBOT_LOW_BLUE ).toInt(),
                                  settings.value ( "gui/magentaRobotLowGreen", MAGENTA_ROBOT_LOW_GREEN ).toInt(),
                                  settings.value ( "gui/magentaRobotLowRed", MAGENTA_ROBOT_LOW_RED ).toInt() );
    cv::Scalar magentaRobotHighRange ( settings.value ( "gui/magentaRobotHighBlue", MAGENTA_ROBOT_HIGH_BLUE).toInt(),
                                   settings.value ( "gui/magentaRobotHighGreen", MAGENTA_ROBOT_HIGH_GREEN ).toInt(),
                                   settings.value ( "gui/magentaRobotHighRed", MAGENTA_ROBOT_HIGH_RED ).toInt() );

    cv::Scalar magentaRobotLowRangeHsv ( settings.value ( "gui/magentaRobotLowH", MAGENTA_ROBOT_LOW_H ).toInt(),
                                     settings.value ( "gui/magentaRobotLowS", MAGENTA_ROBOT_LOW_S ).toInt() );

    cv::Scalar magentaRobotHighRangeHsv ( settings.value ( "gui/magentaRobotHighH", MAGENTA_ROBOT_HIGH_H ).toInt(),
                                     settings.value ( "gui/magentaRobotHighS", MAGENTA_ROBOT_HIGH_S ).toInt() );
    robot = ui->widgetShowVideo->getControlInterface()->getRobots()->at ( 4 );
    robot->setRangeRgb ( magentaRobotLowRange, magentaRobotHighRange );
    robot->setRangeHsv ( magentaRobotLowRangeHsv, magentaRobotHighRangeHsv );


    cv::Scalar purpleRobotLowRange ( settings.value ( "gui/purpleRobotLowBlue", PURPLE_ROBOT_LOW_BLUE ).toInt(),
                                  settings.value ( "gui/purpleRobotLowGreen", PURPLE_ROBOT_LOW_GREEN ).toInt(),
                                  settings.value ( "gui/purpleRobotLowRed", PURPLE_ROBOT_LOW_RED ).toInt() );
    cv::Scalar purpleRobotHighRange ( settings.value ( "gui/purpleRobotHighBlue", PURPLE_ROBOT_HIGH_BLUE).toInt(),
                                   settings.value ( "gui/purpleRobotHighGreen", PURPLE_ROBOT_HIGH_GREEN ).toInt(),
                                   settings.value ( "gui/purpleRobotHighRed", PURPLE_ROBOT_HIGH_RED ).toInt() );

    cv::Scalar purpleRobotLowRangeHsv ( settings.value ( "gui/purpleRobotLowH", PURPLE_ROBOT_LOW_H ).toInt(),
                                     settings.value ( "gui/purpleRobotLowS", PURPLE_ROBOT_LOW_S ).toInt() );

    cv::Scalar purpleRobotHighRangeHsv ( settings.value ( "gui/purpleRobotHighH", PURPLE_ROBOT_HIGH_H ).toInt(),
                                     settings.value ( "gui/purpleRobotHighS", PURPLE_ROBOT_HIGH_S ).toInt() );
    robot = ui->widgetShowVideo->getControlInterface()->getRobots()->at ( 5 );
    robot->setRangeRgb ( purpleRobotLowRange, purpleRobotHighRange );
    robot->setRangeHsv ( purpleRobotLowRangeHsv, purpleRobotHighRangeHsv );


    cv::Scalar orangeRobotLowRange ( settings.value ( "gui/orangeRobotLowBlue", ORANGE_ROBOT_LOW_BLUE ).toInt(),
                                  settings.value ( "gui/orangeRobotLowGreen", ORANGE_ROBOT_LOW_GREEN ).toInt(),
                                  settings.value ( "gui/orangeRobotLowRed", ORANGE_ROBOT_LOW_RED ).toInt() );
    cv::Scalar orangeRobotHighRange ( settings.value ( "gui/orangeRobotHighBlue", ORANGE_ROBOT_HIGH_BLUE).toInt(),
                                   settings.value ( "gui/orangeRobotHighGreen", ORANGE_ROBOT_HIGH_GREEN ).toInt(),
                                   settings.value ( "gui/orangeRobotHighRed", ORANGE_ROBOT_HIGH_RED ).toInt() );

    cv::Scalar orangeRobotLowRangeHsv ( settings.value ( "gui/orangeRobotLowH", ORANGE_ROBOT_LOW_H ).toInt(),
                                     settings.value ( "gui/orangeRobotLowS", ORANGE_ROBOT_LOW_S ).toInt() );

    cv::Scalar orangeRobotHighRangeHsv ( settings.value ( "gui/purpleRobotHighH", ORANGE_ROBOT_HIGH_H ).toInt(),
                                     settings.value ( "gui/purpleRobotHighS", ORANGE_ROBOT_HIGH_S ).toInt() );
    robot = ui->widgetShowVideo->getControlInterface()->getRobots()->at ( 6 );
    robot->setRangeRgb ( orangeRobotLowRange, orangeRobotHighRange );
    robot->setRangeHsv ( orangeRobotLowRangeHsv, orangeRobotHighRangeHsv );
}

void MainWindow::blockExecuteAStar()
{
//    ui->pushButtonCallAStar->setEnabled ( false );
}

void MainWindow::unblockExecuteAStar()
{
    ui->pushButtonCallAStar->setEnabled ( true );
}

void MainWindow::windowDestroyed()
{
    cv::destroyAllWindows();
}



void MainWindow::changeColorRange()
{
//    int rangeValues [ 6 ];
//    rangeValues [ 0 ] = ui->spinBoxLowBlue->value();
//    rangeValues [ 1 ] = ui->spinBoxLowGreen->value();
//    rangeValues [ 2 ] = ui->spinBoxLowRed->value();
//    rangeValues [ 3 ] = ui->spinBoxHighBlue->value();
//    rangeValues [ 4 ] = ui->spinBoxHighGreen->value();
//    rangeValues [ 5 ] = ui->spinBoxHighRed->value();

//    qDebug() << "MainWindow::changeColorRange" << ui->comboBoxSelectRobot->currentIndex();
    cv::Scalar lowRangeRgb, highRangeRgb, lowRangeHsv, highRangeHsv;
    lowRangeRgb = cv::Scalar ( ui->spinBoxLowBlue->value(), ui->spinBoxLowGreen->value(), ui->spinBoxLowRed->value() );
    highRangeRgb = cv::Scalar ( ui->spinBoxHighBlue->value(), ui->spinBoxHighGreen->value(), ui->spinBoxHighRed->value() );
    lowRangeHsv = cv::Scalar ( ui->spinBoxLowH->value(), ui->spinBoxLowS->value() );
    highRangeHsv = cv::Scalar ( ui->spinBoxHighH->value(), ui->spinBoxHighS->value() );
    Robot *robot = ui->widgetShowVideo->getControlInterface()->getRobotByComboBoxNumber ( ui->comboBoxSelectRobot->currentIndex() );
//    switch (ui->comboBoxSelectRobot->currentIndex())
//    {
//        case RED_ROBOT_NUMBER:
//            robot = ui->widgetShowVideo->getControlInterface()->getRobots()->at ( 0 );
//            break;
//        case BLUE_ROBOT_NUMBER:
//            robot = ui->widgetShowVideo->getControlInterface()->getRobots()->at ( 1 );
//            break;
//        case GREEN_ROBOT_NUMBER:
//            robot = ui->widgetShowVideo->getControlInterface()->getRobots()->at ( 2 );
//            break;
//    }
    if ( robot != NULL )
    {

        robot->setRangeRgb ( lowRangeRgb, highRangeRgb );
        robot->setRangeHsv ( lowRangeHsv, highRangeHsv );
    }
    else
    {
        qDebug() << "# Erro void MainWindow::changeColorRange() -> robot == NULL";
    }

    //    ui->widgetShowVideo->updateRange ( rangeValues );
}

void MainWindow::changeCurrentIndexSelectRobot ( int value )
{
//    qDebug() << "MainWindow::changeCurrentIndexSelectRobot(int)" << value;

//    switch (ui->comboBoxSelectRobot->currentIndex())
//    {
//    case RED_ROBOT_NUMBER:
//        lowRange = ui->widgetShowVideo->getControlInterface()->getRedRobotLowRangeR();
//        highRange = ui->widgetShowVideo->getControlInterface()->getRedRobotHighRange();
//        break;
//    case BLUE_ROBOT_NUMBER:
//        lowRange = ui->widgetShowVideo->getControlInterface()->getBlueRobotLowRange();
//        highRange = ui->widgetShowVideo->getControlInterface()->getBlueRobotHighRange();
//        break;
//    case GREEN_ROBOT_NUMBER:
//        lowRange = ui->widgetShowVideo->getControlInterface()->getGreenRobotLowRange();
//        highRange = ui->widgetShowVideo->getControlInterface()->getGreenRobotHighRange();
//        break;
//    }
    Robot *robot = ui->widgetShowVideo->getControlInterface()->getRobotByComboBoxNumber ( ui->comboBoxSelectRobot->currentIndex() );
//    Robot *robot = NULL;
//    switch (ui->comboBoxSelectRobot->currentIndex())
//    {
//    case RED_ROBOT_NUMBER:
//        robot = ui->widgetShowVideo->getControlInterface()->getRobots()->at ( 0 );
//        break;
//    case BLUE_ROBOT_NUMBER:
//        robot = ui->widgetShowVideo->getControlInterface()->getRobots()->at ( 1 );
//        break;
//    case GREEN_ROBOT_NUMBER:
//        robot = ui->widgetShowVideo->getControlInterface()->getRobots()->at ( 2 );
//        break;
//    }
    if ( robot != NULL )
    {
        cv::Scalar lowRangeRgb = robot->getLowRangeRgb();
        cv::Scalar highRangeRgb = robot->getHighRangeRgb();

        cv::Scalar lowRangeHsv = robot->getLowRangeHsv();
        cv::Scalar highRangeHsv = robot->getHighRangeHsv();

        ui->spinBoxLowBlue->setValue (  lowRangeRgb[ 0 ] );
        ui->spinBoxLowGreen->setValue ( lowRangeRgb [ 1 ] );
        ui->spinBoxLowRed->setValue ( lowRangeRgb [ 2 ] );
        ui->spinBoxHighBlue->setValue ( highRangeRgb [ 0 ] );
        ui->spinBoxHighGreen->setValue ( highRangeRgb [ 1 ] );
        ui->spinBoxHighRed->setValue ( highRangeRgb [ 2 ] );

        ui->spinBoxLowH->setValue ( lowRangeHsv [ 0 ] );
        ui->spinBoxLowS->setValue ( lowRangeHsv [ 1 ] );
        ui->spinBoxHighH->setValue ( highRangeHsv [ 0 ] );
        ui->spinBoxHighS->setValue ( highRangeHsv [ 1 ] );
    }
    else
    {
        qDebug() << "# Erro void MainWindow::changeCurrentIndexSelectRobot ( int value ) -> robot == NULL";
    }

//    if ( ui->widgetShowVideo->getControlInterface()->getImageProcessing()->getSystemColorRgb() )
//    {
//        ui->spinBoxLowBlue->setValue ( lowRange [ 0 ] );
//        ui->spinBoxLowGreen->setValue ( lowRange [ 1 ] );
//        ui->spinBoxLowRed->setValue ( lowRange [ 2 ] );
//        ui->spinBoxHighBlue->setValue ( highRange [ 0 ] );
//        ui->spinBoxHighGreen->setValue ( highRange [ 1 ] );
//        ui->spinBoxHighRed->setValue ( highRange [ 2 ] );
//    }
//    else
//    {
//        ui->spinBoxLowH->setValue ( lowRange [ 0 ] );
//        ui->spinBoxLowS->setValue ( lowRange [ 1 ] );
//        ui->spinBoxHighH->setValue ( highRange [ 0 ] );
//        ui->spinBoxHighS->setValue ( highRange [ 1 ] );
//    }
}

void MainWindow::changeRobotsRadiusMin(int value)
{
    qDebug() << "MainWindow::changeRobotsRadiusMin(int)" << value;
}

void MainWindow::changeRobotsRadiusMax(int value)
{
    qDebug() << "MainWindow::changeRobotsRadiusMax(int)" << value;
}

void MainWindow::changeDistanceOfIdenticators(int value)
{
    qDebug() << "MainWindow::changeDistanceOfIdenticators(int)" << value;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent ( event );
    cv::destroyAllWindows();
}

void MainWindow::typeOfSegmentation()
{
//    qDebug() << "typeOfSegmentation";
    if ( ui->radioButtonHsv->isChecked() )
    {
        ui->widgetShowVideo->getControlInterface()->getImageProcessing()->setSystemColorRgb ( false );
        ui->spinBoxLowBlue->setEnabled ( false );
        ui->spinBoxLowGreen->setEnabled ( false );
        ui->spinBoxLowRed->setEnabled ( false );
        ui->spinBoxHighBlue->setEnabled ( false );
        ui->spinBoxHighGreen->setEnabled ( false );
        ui->spinBoxHighRed->setEnabled ( false );

        ui->spinBoxLowH->setEnabled ( true );
        ui->spinBoxLowS->setEnabled ( true );
        ui->spinBoxHighH->setEnabled ( true );
        ui->spinBoxHighS->setEnabled ( true );
    }
    else if ( ui->radioButtonRgb->isChecked() )
    {
        ui->widgetShowVideo->getControlInterface()->getImageProcessing()->setSystemColorRgb ( true );
        ui->spinBoxLowH->setEnabled ( false );
        ui->spinBoxLowS->setEnabled ( false );
        ui->spinBoxHighH->setEnabled ( false );
        ui->spinBoxHighS->setEnabled ( false );

        ui->spinBoxLowBlue->setEnabled ( true );
        ui->spinBoxLowGreen->setEnabled ( true );
        ui->spinBoxLowRed->setEnabled ( true );
        ui->spinBoxHighBlue->setEnabled ( true );
        ui->spinBoxHighGreen->setEnabled ( true );
        ui->spinBoxHighRed->setEnabled ( true );
    }
//    qDebug() << "system color" << ui->widgetShowVideo->getControlInterface()->getImageProcessing()->getSystemColorRgb();
    changeColorRange ();
}

void MainWindow::mouseMoveImageWidget(int x, int y, int r, int g, int b)
{
    ui->labelXPosValue->setText ( QString::number ( x ) );
    ui->labelYPosValue->setText ( QString::number ( y ) );

//    qDebug() << "void MainWindow::mouseMoveImageWidget(int r, int g, int b)" << r << g << b;
    ui->labelRedValue->setText ( QString::number ( r ) );
    ui->labelGreenValue->setText ( QString::number ( g ) );
    ui->labelBlueValue->setText ( QString::number ( b ) );

    double nr = m_map ( r, 0.0, 255.0, 0.0, 1.0 );
    double ng = m_map ( g, 0.0, 255.0, 0.0, 1.0 );;
    double nb = m_map ( b, 0.0, 255.0, 0.0, 1.0 );;

    double maxValue = max ( max ( nr , ng ), nb );
    double minValue = min ( min ( nr , ng ), nb );

    double h, s;
    if ( maxValue == minValue )
        h = 0;
    else if ( maxValue == nr )
    {
        if ( ng >= nb )
            h = 60 * ( ng - nb ) / ( maxValue - minValue );
        else
            h = 60 * ( ng - nb ) / ( maxValue - minValue ) + 360;
    }
    else if ( maxValue == ng )
        h = 60 * ( nb - nr ) / ( maxValue - minValue ) + 120;
    else // maxValue == b
        h = 60 * ( nr - ng ) / ( maxValue - minValue ) + 240;

    s = ( maxValue - minValue ) / maxValue * 100;

//    qDebug() << maxValue << minValue << h << s;

    ui->labelHueValue->setText ( QString::number ( (int)h ) );
    ui->labelSatValue->setText ( QString::number ( (int)s ) );
    ui->labelValValue->setText ( QString::number ( (int)(maxValue * 100) )/*v*/ );
}
