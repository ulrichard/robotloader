
// Chopper
#include "Communication.h"
// boost
#include <boost/tuple/tuple.hpp>
#include <boost/thread/locks.hpp>
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_attribute.hpp>
#include <boost/spirit/include/classic_symbols.hpp>
#include <boost/spirit/include/phoenix1_actor.hpp>
#include <boost/spirit/include/phoenix1_statements.hpp>
#include <boost/lexical_cast.hpp>

using namespace boost::spirit::classic;
using namespace phoenix;

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
Communication::Communication(const std::string &device, size_t baud)
    : sercli(io_service, boost::bind(&Communication::DispatchMsg, this, ::_1), baud, device),
      thrd(boost::bind(&boost::asio::io_service::run, &io_service))
{

}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
Communication::~Communication()
{
    sercli.close(); // close the minicom client connection
    thrd.join(); // wait for the IO service thread to close
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
void Communication::sendCommand(const std::string &cmd)
{
    boost::lock_guard<boost::mutex> lg(send_mtx_);

 //   std::cout << "Sending : " << cmd << std::endl;

    if(!sercli.active())
		throw std::runtime_error("serial client not active when trying to send message!");
    sercli.write_str(cmd);

    std::cout << "Sent : " << cmd << std::endl;
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
template<class T>
struct ParserEventAssign
{
    ParserEventAssign(Communication::LISTENER_TYPE typ, Communication::ListenerCont &listeners)
        : typ_(typ), numval_(0), listeners_(listeners), v1_(dummy_), v2_(dummy_), v3_(dummy_) { }
    ParserEventAssign(Communication::LISTENER_TYPE typ, Communication::ListenerCont &listeners, T &v1)
        : typ_(typ), numval_(1), listeners_(listeners), v1_(v1), v2_(v1), v3_(v1) { }
    ParserEventAssign(Communication::LISTENER_TYPE typ, Communication::ListenerCont &listeners, T &v1, T &v2)
        : typ_(typ), numval_(2), listeners_(listeners), v1_(v1), v2_(v2), v3_(v2) { }
    ParserEventAssign(Communication::LISTENER_TYPE typ, Communication::ListenerCont &listeners, T &v1, T &v2, T &v3)
        : typ_(typ), numval_(3), listeners_(listeners), v1_(v1), v2_(v2), v3_(v3) { }

    template<class iterT>
    void operator()(iterT begin, iterT end) const
    {
        if(listeners_.count(typ_))
            for(Communication::ListenerCont::const_iterator it = listeners_.lower_bound(typ_); it != listeners_.upper_bound(typ_); ++it)
                switch(numval_)
                {
                case 0:
                    it->second(boost::any(boost::lexical_cast<T>(std::string(begin, end))));
                    break;
                case 1:
                    it->second(boost::any(static_cast<T>(v1_)));
                    break;
                case 2:
                    it->second(boost::any(std::pair<T, T>(v1_, v2_)));
                    break;
                case 3:
                    it->second(boost::any(boost::tuple<T, T, T>(v1_, v2_, v3_)));
                    break;
                default:
                    assert(false);
                }
    }

    void operator()(const T &val)
    {
        assert(numval_ == 0);
        if(listeners_.count(typ_))
            for(Communication::ListenerCont::const_iterator it = listeners_.lower_bound(typ_); it != listeners_.upper_bound(typ_); ++it)
                it->second(boost::any(static_cast<T>(val)));

    }
private:
    const Communication::LISTENER_TYPE typ_;
    const size_t numval_;
    Communication::ListenerCont &listeners_;
    T dummy_;
    T &v1_, &v2_, &v3_;
};
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A
void Communication::DispatchMsg(std::string msg)
{
    static std::string buf = "";

    buf += msg;
    size_t nlpos = buf.find('\n');
    if(nlpos == std::string::npos)
        return;
    msg = buf.substr(0, nlpos);
    buf = buf.substr(nlpos + 1);

    std::cout << "Received : \"" << msg << "\"" << std::endl;

	typedef rule<> rule_t;

    size_t curr_angle, curr_distval, curr_height, curr_comp, accel_x, accel_y, accel_z, curr_baro;


    rule_t rngPair   = ("RA" >> uint_p[assign_a(curr_angle)] >> "RV" >> uint_p[assign_a(curr_distval)])
                        [ParserEventAssign<size_t>(LST_RANGER, listeners_, curr_angle, curr_distval)];
    rule_t heighGnd  = ("AG" >> uint_p[assign_a(curr_height)])[ParserEventAssign<size_t>(LST_HEIGHT, listeners_, curr_height)];
    rule_t compassHd = ("CH" >> uint_p[assign_a(curr_comp)])[ParserEventAssign<size_t>(LST_COMPASS, listeners_, curr_comp)];
    rule_t accelFull = ("AX" >> uint_p[assign_a(accel_x)] >> "AY" >> uint_p[assign_a(accel_y)] >> "AZ" >> uint_p[assign_a(accel_z)])
                        [ParserEventAssign<size_t>(LST_ACCEL, listeners_, accel_x, accel_y, accel_z)];
    rule_t baroHeig  = ("BH" >> uint_p[assign_a(curr_baro)])[ParserEventAssign<size_t>(LST_BARO, listeners_, curr_baro)];

    rule_t sentence = (*(rngPair | heighGnd | compassHd | accelFull | baroHeig))[ParserEventAssign<std::string>(LST_TEXT, listeners_)] >> *blank_p;

    parse_info<> info = parse(msg.c_str(), sentence, space_p);
//    if(!info.hit || info.length < msg.length() - 1)
//        throw std::runtime_error("failed to parse sentence : " + msg);
}
/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8/////////9/////////A




