
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
#include <boost/lexical_cast.hpp>

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
    : QMainWindow()
	, sercomm_(NULL)
{
	setupUi(this);
    setCentralWidget(centralwidget);

	for(size_t i=0; i<20; ++i)
		if(bfs::exists("/dev/ttyUSB" + boost::lexical_cast<std::string>(i)))
			cbSerialPort->addItem(("/dev/ttyUSB" + boost::lexical_cast<std::string>(i)).c_str());


    // todo : pass the servo number as parameter to the slot

    connect(sli1_Gripper,  SIGNAL(sliderMoved(int)), this, SLOT(sliderChanged(int)));
    connect(sli2_Hand,     SIGNAL(sliderMoved(int)), this, SLOT(sliderChanged(int)));
    connect(sli3_Wrist,    SIGNAL(sliderMoved(int)), this, SLOT(sliderChanged(int)));
    connect(sli4_Ellbow,   SIGNAL(sliderMoved(int)), this, SLOT(sliderChanged(int)));
    connect(sli5_Shoulder, SIGNAL(sliderMoved(int)), this, SLOT(sliderChanged(int)));
    connect(sli6_BaseRot,  SIGNAL(sliderMoved(int)), this, SLOT(sliderChanged(int)));

	connect(cbSerialConnected, SIGNAL(stateChanged(int)), this, SLOT(SerialConnect(int)));
	connect(cbRTS,             SIGNAL(stateChanged(int)), this, SLOT(ChangeRTS(int)));

    TextAdder ta(*plainTextEdit);
    ta(boost::any(std::string("Welcome")));

}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
RACSQTMain::~RACSQTMain()
{
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
void RACSQTMain::SerialConnect(int val)
{
	if(val && NULL == sercomm_)
	{
		std::cout << "Connecting to " << cbSerialPort->getText() << std::endl;
		try
		{
			sercomm_ = new Communication(cbSerialPort->getText());
			sercomm_->addListener(Communication::LST_TEXT, TextAdder(*plainTextEdit));

			sercomm_->enableRTS(true);
			boost::this_thread::sleep(boost::posix_time::millisec(200));
			sercomm_->enableRTS(false);
		}
		catch(std::exception &ex)
		{
			std::stringstream sstr;
			sstr << "Could not establish a serial connection to the robot arm on \"/dev/ttyUSB0\" with the following message:\n" << ex.what();
			QMessageBox::critical(this, "No connection to the robot arm", sstr.str().c_str(), QMessageBox::Ok);
		}
	}
	else if(!val && NULL != sercomm_)
	{
		std::cout << "Disconnecting serial connection" << std::endl;
		delete sercomm_;
		sercomm_ = NULL;
	}
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
void RACSQTMain::ChangeRTS(int val)
{
	if(NULL != sercomm_)
		sercomm_->enableRTS(0 != val);
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
void RACSQTMain::sliderChanged(int val)
{
	if(NULL != sercomm_)
	{
		std::cout << "Serial not connected" << std::endl;
		return;
	}

    std::stringstream sstr;

    sstr << "1:"
         << std::showpos << std::setw(3) << std::setfill('0') << val << std::endl;

    if(sstr.str().length())
        sercomm_->sendCommand(sstr.str());
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

