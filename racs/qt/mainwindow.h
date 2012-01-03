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
class RACSQTMain : public QMainWindow, private Ui_MainWindow
{
    Q_OBJECT;
public:
    RACSQTMain();
    virtual ~RACSQTMain();

private slots:
    void sliderChanged(int val);

private:
    Communication     *sercomm_;

//    boost::thread      inputListener;
};
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A


