/*
 * StateFS UPower provider
 *
 * Copyright (C) 2013 Jolla Ltd.
 * Contact: Denis Zalevskiy <denis.zalevskiy@jollamobile.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 */

#include "provider_upower.hpp"

#include <cor/util.hpp>
#include <statefs/qt/dbus.hpp>

#include <math.h>
#include <iostream>


namespace statefs { namespace upower {

using statefs::qt::Namespace;
using statefs::qt::PropertiesSource;
using statefs::qt::sync;

static char const *service_name = "org.freedesktop.UPower"; 

Bridge::Bridge(PowerNs *ns, QDBusConnection &bus)
    : PropertiesSource(ns)
    , bus_(bus)
    , watch_(bus, service_name)
    , default_values_{87.0, true, false, 878787, 0, UnknownState}
    , last_values_(default_values_)
{
}

void Bridge::update_all_props()
{
    using namespace std::placeholders;
    auto set = std::bind(&PropertiesSource::updateProperty, this, _1, _2);
    auto actions = std::make_tuple
        ([&set](double v) {
            set("ChargePercentage", round(v));
            set("Capacity", v);
        }, [&set](bool v) {
            set("OnBattery", v);
            if (v) set("TimeUntilFull", 0);
        }, [&set](bool v) { set("LowBattery", v);
        }, [&set](qlonglong v) { set("TimeUntilLow", v);
        }, [&set](qlonglong v) { set("TimeUntilFull", v);
        }, [&set](DeviceState v) {
            bool is_charging = (v == Charging || v == FullyCharged);
            set("IsCharging", is_charging);
            if (!is_charging || v == FullyCharged)
                set("TimeUntilFull", 0);
        });

    if (device_ && manager_) {
        Properties props_now { device_->percentage()
                , manager_->onBattery(), manager_->onLowBattery()
                , device_->timeToEmpty(), device_->timeToFull()
                , (DeviceState)device_->state() };
        cor::copy_apply_if_changed(last_values_, props_now, actions);
    } else if (manager_) {
        Properties props_now { 99, manager_->onBattery()
                , manager_->onLowBattery(), 12345, 0, UnknownState };
        cor::copy_apply_if_changed(last_values_, props_now, actions);
    } else {
        cor::copy_apply_if_changed(last_values_, default_values_, actions);
    }
}

bool Bridge::try_get_battery(QString const &path)
{
    auto is_battery = [](std::unique_ptr<Device> const &p) {
        return (p->nativePath() == "battery");
    };
    std::unique_ptr<Device> device(new Device(service_name, path, bus_));
    if (!is_battery(device))
        return false;

    device_ = std::move(device);
    device_path_ = path;
    if (device_) {
        connect(device_.get(), &Device::Changed
                , this, &Bridge::update_all_props);
    } else {
        qWarning() << "No battery found";
    }
    update_all_props();
    return true;
}

void Bridge::init_manager()
{
    manager_.reset(new Manager(service_name, "/org/freedesktop/UPower", bus_));
    auto find_battery = [this](QList<QDBusObjectPath> const &devices) {
        qDebug() << "found " << devices.size() << " upower device(s)";
        std::find_if(devices.begin(), devices.end()
                     , [this](QDBusObjectPath const &p) {
                         return try_get_battery(p.path());
                     });
    };

    qDebug() << "Enumerating upower devices";
    async(this, manager_->EnumerateDevices(), find_battery);
    connect(manager_.get(), &Manager::Changed
            , this, &Bridge::update_all_props);
    using namespace std::placeholders;
    connect(manager_.get(), &Manager::DeviceAdded
            , this, &Bridge::try_get_battery);
    connect(manager_.get(), &Manager::DeviceRemoved
            , [this](QString const &path) {
                if (path == device_path_) {
                    reset_device();
                }
            });
}

void Bridge::reset_device()
{
    device_.reset();
    device_path_ = "";
    update_all_props();
}

void Bridge::init()
{
    auto reset_manager = [this]() {
        reset_device();
        manager_.reset();
        update_all_props();
    };
    watch_.init([this]() { init_manager(); }, reset_manager);
    init_manager();
}

PowerNs::PowerNs(QDBusConnection &bus)
    : Namespace("Battery", std::unique_ptr<PropertiesSource>
                (new Bridge(this, bus)))
    , defaults_{
    { "ChargePercentage", "87" }
    , { "Capacity", "87" }
    , { "OnBattery", "1" }
    , { "LowBattery", "0" }
    , { "TimeUntilLow", "878787" }
    , { "TimeUntilFull", "0" }
    , { "IsCharging", "0" }}
{
    for (auto v : defaults_)
        addProperty(v.first, v.second);
    src_->init();
}

class Provider;
static Provider *provider = nullptr;

class Provider : public statefs::AProvider
{
public:
    Provider(statefs_server *server)
        : AProvider("upower", server)
        , bus_(QDBusConnection::systemBus())
    {
        auto ns = std::make_shared<PowerNs>(bus_);
        insert(std::static_pointer_cast<statefs::ANode>(ns));
    }
    virtual ~Provider() {}

    virtual void release() {
        if (this == provider) {
            delete provider;
            provider = nullptr;
        }
    }

private:
    QDBusConnection bus_;
};

static inline Provider *init_provider(statefs_server *server)
{
    if (provider)
        throw std::logic_error("provider ptr is already set");
    provider = new Provider(server);
    return provider;
}

}}

EXTERN_C struct statefs_provider * statefs_provider_get
(struct statefs_server *server)
{
    return statefs::upower::init_provider(server);
}
