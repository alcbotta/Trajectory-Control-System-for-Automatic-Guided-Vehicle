#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSettings>
#include <QTimer>
#include <QCloseEvent>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void updateSettings ();

public slots:

    void blockExecuteAStar ();
    void unblockExecuteAStar ();

    void windowDestroyed ();
    //  Eventos da aba de Segmentação:
    void changeColorRange ( );
    void changeCurrentIndexSelectRobot ( int value );

    //  Eventos da aba de Identificação:
    void changeRobotsRadiusMin ( int value );
    void changeRobotsRadiusMax ( int value );
    void changeDistanceOfIdenticators ( int value );
    void closeEvent(QCloseEvent *event);


    void typeOfSegmentation ();

    void mouseMoveImageWidget ( int x, int y, int r, int g, int b );
    
private:
    Ui::MainWindow *ui;
    QSettings settings;
    QTimer *timerReadNewFrame;
};

#endif // MAINWINDOW_H
