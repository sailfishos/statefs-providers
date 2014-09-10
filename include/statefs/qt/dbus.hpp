#ifndef _STATEFS_QT_DBUS_HPP_
#define _STATEFS_QT_DBUS_HPP_

#include <QDBusPendingReply>
#include <QDBusServiceWatcher>
#include <QDebug>

#include <stdexcept>
#include <tuple>
#include <memory>

namespace statefs { namespace qt {

template <typename T>
QDBusPendingReply<T> sync(QDBusPendingReply<T> &&reply)
{
    QDBusPendingCallWatcher watcher(reply);
    watcher.waitForFinished();
    if (!watcher.isFinished())
        throw std::logic_error("D-Bus request is not executed");
    return reply;
}

template <typename OnValue, typename T>
bool callback_or_error(QDBusPendingReply<T> const &reply, OnValue on_value)
{
    if (reply.isError()) {
        auto err = reply.error();
        qWarning() << "D-Bus request error " << err.name()
                   << ": " << err.message();
        return false;
    }

    on_value(reply.value());
    return true;
}

template <typename T, typename OnValue>
void async(QObject *parent, QDBusPendingCallWatcher *watcher, OnValue on_value)
{
    parent->connect(watcher, &QDBusPendingCallWatcher::finished
                  , [on_value](QDBusPendingCallWatcher *w) {
                        QDBusPendingReply<T> reply = *w;
                        callback_or_error(reply, on_value);
                        w->deleteLater();
                    });
}

template <typename T, typename OnValue>
void async(QObject *parent, QDBusPendingCall const &call, OnValue &&on_value)
{
    auto watcher = new QDBusPendingCallWatcher(call, parent);
    async<T>(parent, watcher, std::forward<OnValue>(on_value));
}

template <typename OnValue, typename T>
void async(QObject *parent, QDBusPendingReply<T> &&reply, OnValue on_value)
{
    auto watcher = new QDBusPendingCallWatcher(reply, parent);
    async<T>(parent, watcher, std::forward<OnValue>(on_value));
}

template <typename OnValue, typename T>
bool sync(QDBusPendingReply<T> reply, OnValue on_value)
{
    QDBusPendingCallWatcher watcher(reply);
    watcher.waitForFinished();
    if (!watcher.isFinished()) {
        qWarning() << "D-Bus request is not executed";
        return false;
    }
    return callback_or_error(reply, on_value);
}

template <size_t Pos, typename T>
struct TupleDBus
{
    static void output(QDBusArgument &argument, T const &src)
    {
        argument << std::get<std::tuple_size<T>::value - Pos>(src);
        TupleDBus<Pos - 1, T>::output(argument, src);
    }

    static void input(QDBusArgument const &argument, T &dst)
    {
        argument >> std::get<std::tuple_size<T>::value - Pos>(dst);
        TupleDBus<Pos - 1, T>::input(argument, dst);
    }
};

template <typename T>
struct TupleDBus<0, T>
{
    static void output(QDBusArgument &, T const &)
    {
        // do nothing, after the last element
    }

    static void input(QDBusArgument const &, T &)
    {
        // do nothing, after the last element
    }
};


class ServiceWatch : public QObject
{
    Q_OBJECT;
public:
    ServiceWatch(QDBusConnection &bus, QString const &service)
        : bus_(bus), service_(service)
    {}

    virtual ~ServiceWatch() {}

    template <typename RegT, typename UnregT>
    void init(RegT onRegister, UnregT onUnregister)
    {
        if (watcher_)
            return;

        watcher_.reset(new QDBusServiceWatcher(service_, bus_));
        // , QDBusServiceWatcher::WatchForRegistration));

        auto cb = [onRegister, onUnregister]
            (const QString & serviceName
             , const QString & oldOwner
             , const QString & newOwner) {
            try {
                if (newOwner == "") {
                    qDebug() << serviceName << " is unregistered";
                    onUnregister();
                } else {
                    qDebug() << serviceName << " is registered";
                    onRegister();
                }
            } catch(std::exception const &e) {
                qWarning() << "Exception " << e.what() << " handling "
                << serviceName << " un/registration";
            }
        };
        connect(watcher_.get(), &QDBusServiceWatcher::serviceOwnerChanged
                , cb);
    }

private:
    QDBusConnection &bus_;
    QString service_;
    std::unique_ptr<QDBusServiceWatcher> watcher_;
};

}}

template <typename ... Args>
QDBusArgument & operator << (QDBusArgument &argument, std::tuple<Args...> const &src)
{
    using statefs::qt::TupleDBus;
    typedef std::tuple<Args...> tuple_type;
    argument.beginStructure();
    TupleDBus<std::tuple_size<tuple_type>::value
              , tuple_type>::output(argument, src);
    argument.endStructure();
    return argument;
}

template <typename ... Args>
QDBusArgument const& operator >> (QDBusArgument const &argument, std::tuple<Args...> &dst)
{
    using statefs::qt::TupleDBus;
    typedef std::tuple<Args...> tuple_type;
    argument.beginStructure();
    TupleDBus<std::tuple_size<tuple_type>::value
              , tuple_type>::input(argument, dst);
    argument.endStructure();
    return argument;
}

#endif // _STATEFS_QT_DBUS_HPP_
