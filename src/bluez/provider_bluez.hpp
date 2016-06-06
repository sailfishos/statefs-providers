#ifndef _STATEFS_PRIVATE_BLUEZ_HPP_
#define _STATEFS_PRIVATE_BLUEZ_HPP_

#include <statefs/provider.hpp>
#include <statefs/property.hpp>
#include <statefs/qt/ns.hpp>
#include <set>

#include <qtaround/dbus.hpp>

// BluezQt
#include <manager.h>
#include <adapter.h>
#include <device.h>

#include <QtDBus>
#include <QObject>

namespace statefs { namespace bluez {

using qtaround::dbus::ServiceWatch;

class BlueZ;

class Bridge : public QObject, public statefs::qt::PropertiesSource
{
    Q_OBJECT
public:

    Bridge(BlueZ *, QDBusConnection &);

    virtual ~Bridge() {}

    virtual void init();

private slots:
    void usableAdapterChanged(BluezQt::AdapterPtr adapter);
    void addDevice(BluezQt::DevicePtr device);
    void removeDevice(BluezQt::DevicePtr device);

private:
    QDBusConnection &bus_;
    std::unique_ptr<BluezQt::Manager> manager_;
    BluezQt::AdapterPtr adapter_;
    std::map<QDBusObjectPath, BluezQt::DevicePtr> devices_;
    std::set<QDBusObjectPath> connected_;
    ServiceWatch watch_;
};

class BlueZ : public statefs::qt::Namespace
{
public:

    BlueZ(QDBusConnection &bus, statefs_provider_mode);

private:
    friend class Bridge;
    void reset_properties();
    statefs::qt::DefaultProperties defaults_;
};

}}

#endif // _STATEFS_PRIVATE_BLUEZ_HPP_
