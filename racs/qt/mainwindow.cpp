
// racsqt
#include "mainwindow.h"
#include "Communication.h"
#include "SignalHandler.h"
// libconfig
#include <libconfig.h++>
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
    QSlider* const sliders[6]   = {sli1_Gripper, sli2_Hand, sli3_Wrist, sli4_Ellbow, sli5_Shoulder, sli6_BaseRot};
    QSpinBox* const spinners[6] = {spb1_Gripper, spb2_Hand, spb3_Wrist, spb4_Ellbow, spb5_Shoulder, spb6_BaseRot};
    for(size_t i=0; i<6; ++i)
    {
        connect1(sliders[i],   SIGNAL(sliderMoved(int)),  boost::bind(&RACSQTMain::sliderChanged, this, i+1, ::_1));
        connect1(spinners[i],  SIGNAL(valueChanged(int)), boost::bind(&RACSQTMain::sliderChanged, this, i+1, ::_1));
        connect(sliders[i],    SIGNAL(sliderMoved(int)),  spinners[i], SLOT(setValue(int)));
        connect(spinners[i],   SIGNAL(valueChanged(int)), sliders[i],  SLOT(setValue(int)));
    }
	connect(cbSerialConnected, SIGNAL(toggled(bool)),     this, SLOT(SerialConnect(bool)));
	connect(pbReset,           SIGNAL(clicked()),         this, SLOT(ResetRobot()));

    TextAdder ta(*plainTextEdit);
    ta(boost::any(std::string("Welcome")));

    // read the last positions
    const bfs::path cfgfile(bfs::path(getenv("HOME")) / ".racsqt.cfg");
    if(bfs::exists(cfgfile))
    {
        libconfig::Config cfg;
        cfg.readFile(cfgfile.string().c_str());
        for(size_t i=0; i<6; ++i)
        {
            const int val = cfg.lookup("servo" + boost::lexical_cast<std::string>(i+1));
            sliders[i]->setValue(val);
            spinners[i]->setValue(val);
        }
    }

}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
RACSQTMain::~RACSQTMain()
{
    SerialConnect(false);
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
	    try
		{
		    // write the current positions
		    const bfs::path cfgfile(bfs::path(getenv("HOME")) / ".racsqt.cfg");
		    libconfig::Config cfg;
		    QSlider* const sliders[6]   = {sli1_Gripper, sli2_Hand, sli3_Wrist, sli4_Ellbow, sli5_Shoulder, sli6_BaseRot};
            for(size_t i=0; i<6; ++i)
                cfg.getRoot().add("servo" + boost::lexical_cast<std::string>(i+1), libconfig::Setting::TypeInt) = sliders[i]->value();
		    cfg.writeFile(cfgfile.string().c_str());


            std::cout << "Disconnecting serial connection" << std::endl;
            delete sercomm_;
		}
		catch(std::exception &ex)
		{
            std::stringstream sstr;
			sstr << "Failed to disconnect the serial connection with the following message:\n" << ex.what();
			QMessageBox::critical(this, "serial disconnect", sstr.str().c_str(), QMessageBox::Ok);
		}
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

	    // send the position for each servo
	    QSlider* const sliders[6]   = {sli1_Gripper, sli2_Hand, sli3_Wrist, sli4_Ellbow, sli5_Shoulder, sli6_BaseRot};
        for(size_t i=0; i<6; ++i)
            sliderChanged(i+1, sliders[i]->value());
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

    const std::string msg(boost::str(boost::format("(%d:%+04d)") % servo % val));
    if(msg.length())
        sercomm_->sendCommand(msg);
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A

