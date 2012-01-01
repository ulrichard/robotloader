// Chopper
#include "main.h"
#include "mainwindow.h"
// Qt
#include <QtGui/QApplication>

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    RACSQTMain mainwindow;

    mainwindow.show();

    return app.exec();
}
