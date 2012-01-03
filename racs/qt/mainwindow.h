#pragma once

// racsqt
#include "ui_mainwindow.h"
// Qt
#include <QtGui/QMainWindow>
// boost
#include <boost/thread.hpp>



class ProximityRadar;
class QCheckBox;
class QPlainTextEdit;
class Communication;
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
class RACSQTMain : public QMainWindow
{
    Q_OBJECT;
public:
    RACSQTMain();
    virtual ~RACSQTMain();

private slots:
//    void enableRanger(int state);
//    void enableLogging(int state);
//    void enableControl(int state);

private:
//    void sendCommands();

	Ui_MainWindow 	  ui;  
    QCheckBox         *cbLog;
    QPlainTextEdit    *plainTextEdit;
    Communication     *sercomm;

//    boost::thread      inputListener;
};
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A


