#ifndef COMMUNICATION_H_INCLUDED
#define COMMUNICATION_H_INCLUDED

//boost
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
//#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/function.hpp>
#include <boost/any.hpp>
// std lib
#include <string>
#include <deque>
#include <iostream>


/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
namespace // anonymous
{
class serial_client
{
public:
        serial_client(boost::asio::io_service& io_service, boost::function<void(std::string)> msg_received, unsigned int baud, const std::string& device)
                : active_(true),
                  io_service_(io_service),
                  serialPort(io_service, device),
                  msg_received_(msg_received)
        {
                if(not serialPort.is_open())
                    throw std::runtime_error("Failed to open serial port\n");

                boost::asio::serial_port_base::baud_rate baud_option(baud);
                serialPort.set_option(baud_option); // set the baud rate after the port has been opened
                serialPort.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
                serialPort.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                serialPort.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
                serialPort.set_option(boost::asio::serial_port_base::character_size(8));
                read_start();
        }
/*
        void write(const char msg) // pass the write data to the do_write function via the io service in the other thread
        {
                io_service_.post(boost::bind(&serial_client::do_write, this, msg));
        }
*/
        void write_str(const std::string& msg) // pass the write data to the do_write function via the io service in the other thread
        {
                io_service_.post(boost::bind(&serial_client::do_write_str, this, msg));
        }

        void close() // call the do_close function via the io service in the other thread
        {
                io_service_.post(boost::bind(&serial_client::do_close, this, boost::system::error_code()));
        }

        bool active() // return true if the socket is still active
        {
                return active_;
        }

private:
        static const int max_read_length = 512; // maximum amount of data to read in one operation

        void read_start(void)
        { // Start an asynchronous read and call read_complete when it completes or fails
                serialPort.async_read_some(boost::asio::buffer(read_msg_, max_read_length),
                        boost::bind(&serial_client::read_complete,
                                this,
                                boost::asio::placeholders::error,
                                boost::asio::placeholders::bytes_transferred));
        }

        void read_complete(const boost::system::error_code& error, size_t bytes_transferred)
        { // the asynchronous read operation has now completed or failed and returned an error
                if (!error)
                { // read completed, so process the data
                        read_msg_[std::min<size_t>(bytes_transferred, max_read_length-1)] = '\0';
                        msg_received_(read_msg_);
                        //cout.write(read_msg_, bytes_transferred); // echo to standard output
                        read_start(); // start waiting for another asynchronous read again
                }
                else
                        do_close(error);
        }
/*
        void do_write(const char msg)
        { // callback to handle write call from outside this class
                bool write_in_progress = !write_msgs_.empty(); // is there anything currently being written?
                write_msgs_.push_back(msg); // store in write buffer
                if (!write_in_progress) // if nothing is currently being written, then start
                        write_start();
        }
*/
        void do_write_str(const std::string &msg)
        { // callback to handle write call from outside this class
                const bool write_in_progress = !write_msgs_.empty(); // is there anything currently being written?
                std::copy(msg.begin(), msg.end(), std::back_inserter(write_msgs_));
                if(!write_in_progress) // if nothing is currently being written, then start
                        write_start();
        }

        void write_start(void)
        { // Start an asynchronous write and call write_complete when it completes or fails
                boost::asio::async_write(serialPort,
                        boost::asio::buffer(&write_msgs_.front(), write_msgs_.size()),
                        boost::bind(&serial_client::write_complete,
                                this,
                                boost::asio::placeholders::error));
        }

        void write_complete(const boost::system::error_code& error)
        { // the asynchronous read operation has now completed or failed and returned an error
                if (!error)
                { // write completed, so send next write data
                        write_msgs_.clear(); // remove the completed data
                        if(!write_msgs_.empty()) // if there is anthing left to be written
                                write_start(); // then start sending the next item in the buffer
                }
                else
                        do_close(error);
        }

        void do_close(const boost::system::error_code& error)
        { // something has gone wrong, so close the socket & make this object inactive
                if (error == boost::asio::error::operation_aborted) // if this call is the result of a timer cancel()
                        return; // ignore it because the connection cancelled the timer
                if (error)
                        throw std::runtime_error(std::string("Error: ") + error.message()); // show the error message
                else
                        throw std::runtime_error("Error: Connection did not succeed.");
                //cout << "Press Enter to exit\n";
                serialPort.close();
                active_ = false;
        }

private:
        bool active_; // remains true while this object is still operating
        boost::asio::io_service& io_service_; // the main IO service that runs this connection
        boost::asio::serial_port serialPort; // the serial port this instance is connected to
        char read_msg_[max_read_length]; // data read from the socket
        std::deque<char> write_msgs_; // buffered write data
        boost::function<void(std::string)> msg_received_;
};
} // anonymous namespace
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
class Communication
{
public:
    Communication(const std::string &device = "/dev/rfcomm0", size_t baud = 115200);
    virtual ~Communication();

    enum LISTENER_TYPE
    {                   // content of the boost::any that's passed with the event :
        LST_TEXT,       // std::string
        LST_RANGER,     // std::pair<size_t, size_t>
        LST_HEIGHT,     // size_t
        LST_COMPASS,    // size_t
        LST_ACCEL,      // boost::tuple<size_t, size_t, size_t>
        LST_BARO,       // size_t
        LST_MOTOR       // std::pair<size_t, size_t>
    };
    typedef std::multimap<LISTENER_TYPE, boost::function<void(boost::any)> > ListenerCont;

    void addListener(const LISTENER_TYPE typ, boost::function<void(boost::any)> func)
    {   listeners_.insert(std::make_pair(typ, func));   }
    void sendCommand(const std::string &cmd);

private:
    void DispatchMsg(std::string msg);

    boost::asio::io_service io_service;
    serial_client           sercli;
    boost::thread           thrd;
    ListenerCont            listeners_;
    boost::mutex            send_mtx_;
};
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A


#endif // COMMUNICATION_H_INCLUDED
