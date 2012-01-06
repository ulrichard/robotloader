
// racsqt
#include "mainwindow.h"
#include "Communication.h"
#include "SignalHandler.h"
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
#include <boost/bind.hpp>
#include <boost/format.hpp>

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
bool connect1(QObject * sender, const char* signal, const boost::function<void(int)>& f)
{
    // Note: Not a leak as sender will delete the handler when destructed
    return QObject::connect(sender, signal, new SignalHandler1(sender, f), SLOT(handleSignal(int)));
}
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

    // connect the signals
    connect1(sli1_Gripper,      SIGNAL(sliderMoved(int)),   boost::bind(&RACSQTMain::sliderChanged, this, 1, ::_1));
    connect1(sli2_Hand,         SIGNAL(sliderMoved(int)),   boost::bind(&RACSQTMain::sliderChanged, this, 2, ::_1));
    connect1(sli3_Wrist,        SIGNAL(sliderMoved(int)),   boost::bind(&RACSQTMain::sliderChanged, this, 3, ::_1));
    connect1(sli4_Ellbow,       SIGNAL(sliderMoved(int)),   boost::bind(&RACSQTMain::sliderChanged, this, 4, ::_1));
    connect1(sli5_Shoulder,     SIGNAL(sliderMoved(int)),   boost::bind(&RACSQTMain::sliderChanged, this, 5, ::_1));
    connect1(sli6_BaseRot,      SIGNAL(sliderMoved(int)),   boost::bind(&RACSQTMain::sliderChanged, this, 6, ::_1));
	connect(cbSerialConnected,  SIGNAL(toggled(bool)),      this, SLOT(SerialConnect(bool)));
	connect(pbReset,            SIGNAL(clicked()),          this, SLOT(ResetRobot()));

    TextAdder ta(*plainTextEdit);
    ta(boost::any(std::string("Welcome")));

}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
RACSQTMain::~RACSQTMain()
{
    delete sercomm_;
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
void RACSQTMain::SerialConnect(bool val)
{
	if(val && NULL == sercomm_)
	{
		std::cout << "Connecting to " << cbSerialPort->currentText().toStdString() << std::endl;
		try
		{
			sercomm_ = new Communication(cbSerialPort->currentText().toStdString(), 38400);
			sercomm_->addListener(Communication::LST_TEXT, TextAdder(*plainTextEdit));
			ResetRobot();
		}
		catch(std::exception &ex)
		{
			std::stringstream sstr;
			sstr << "Could not establish a serial connection to the robot arm on \"/" << cbSerialPort->currentText().toStdString()
                 << "\" with the following message:\n" << ex.what();
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
void RACSQTMain::ResetRobot()
{
	if(NULL != sercomm_)
	{
	    sercomm_->setRTS(true);
	    boost::this_thread::sleep(boost::posix_time::millisec(1000));
	    sercomm_->setRTS(false);
	}
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
void RACSQTMain::sliderChanged(size_t servo, int val)
{
	if(NULL == sercomm_)
	{
		std::cout << "Serial not connected" << std::endl;
		return;
	}

    const std::string msg(boost::str(boost::format("(%1i:%+3.3i)") % servo % val));
    if(msg.length())
        sercomm_->sendCommand(msg);
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A

