#pragma once

// http://uint32t.blogspot.com/2008/11/using-boost-bind-and-boost-function.html

// Qt
#include <QObject>
// boost
#include <boost/function.hpp>


struct SignalHandler0 : QObject
{
private:
  Q_OBJECT
public:
  SignalHandler0(QObject * parent,
                 boost::function<void(void)> const & f):
    QObject(parent), // parent will delete this object when destructed
    m_f(f) {}

public slots:
  void
  handleSignal()
  {
    try
    {
      m_f();
    }
    catch(...)
    {
      // Cannot throw exceptions from signals.
      assert(false);
    }
  }
private:
  boost::function<void(void)> m_f;
};


struct SignalHandler1 : QObject
{
private:
  Q_OBJECT
public:
  SignalHandler1(QObject * parent,
                 boost::function<void(int)> const & f):
    QObject(parent), // parent will delete this object when destructed
    m_f(f) {}

public slots:
  void
  handleSignal(int val)
  {
    try
    {
      m_f(val);
    }
    catch(...)
    {
      // Cannot throw exceptions from signals.
      assert(false);
    }
  }
private:
  boost::function<void(int)> m_f;
};




