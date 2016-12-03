#include <QApplication>
#include "Util.h"
#include "MainWindow.h"
#include "ControlInterface.h"
using namespace cv;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
