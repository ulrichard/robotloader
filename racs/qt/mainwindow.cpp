
// racsqt
#include "mainwindow.h"
#include "Communication.h"
// Qt
#include <QtGui/QCheckBox>
#include <QtGui/QPlainTextEdit>
#include <QMessageBox>
//boost
#include <boost/tuple/tuple.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/ref.hpp>
#include <boost/timer.hpp>

namespace bfs = boost::filesystem;

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
struct TextAdder
{
    TextAdder(QPlainTextEdit &edit) : edit_(edit) { }

    void operator()(const boost::any &val)
    {
        edit_.appendPlainText(QString(boost::any_cast<std::string>(val).c_str()));
        edit_.appendPlainText("\n");
    }
private:
    QPlainTextEdit &edit_;
};
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
RACSQTMain::RACSQTMain()
{
	ui.setupUi(this);

    setCentralWidget(ui.centralwidget);

	try
	{
		sercomm = new Communication("/dev/ttyUSB0");
		sercomm->addListener(Communication::LST_TEXT,    TextAdder(*ui.plainTextEdit));
	}
	catch(std::exception &ex)
	{
		std::stringstream sstr;
		sstr << "Could not establish a bluetooth connection to the robot arm on \"/dev/ttyUSB0\" with the following message:\n" << ex.what();
		QMessageBox::critical(this, "No connection to the robot arm", sstr.str().c_str(), QMessageBox::Ok);
	}

//    connect(cbLog,     SIGNAL(stateChanged(int)), this, SLOT(enableLogging(int)));

    TextAdder ta(*ui.plainTextEdit);
    ta(boost::any(std::string("Welcome")));

}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
/*
void RACSQTMain::sendCommands()
{
    if(!doSendControl_)
        return;

    std::stringstream sstr;

    // front back
    static size_t lastFB = 90;
    const size_t fb = joyslis->AxisVal(1) / 365 + 90;
//    if(fb != lastFB)
        sstr << "fb" << fb << "\n";
    lastFB = fb;

    // left right
    static size_t lastLR = 90;
    const size_t lr = joyslis->AxisVal(0) / 365 + 90;
//    if(lr != lastLR)
        sstr << "lr" << lr << "\n";
    lastLR = lr;

    // rotor throttle (motor speed)
    static size_t lastMS = 0;
    const size_t ms = std::max<int>(0, 256 - (joyslis->AxisVal(3) / 256 + 128));
//    if(ms != lastMS)
        sstr << "ms" << ms << "\n";
    lastMS = ms;

    if(sstr.str().length())
        sercomm->sendCommand(sstr.str());
}
*/
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
RACSQTMain::~RACSQTMain()
{
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A

