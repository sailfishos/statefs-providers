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
#include <cor/error.hpp>
#include <qtaround/dbus.hpp>

#include <math.h>
#include <iostream>


namespace statefs { namespace upower {

using statefs::qt::Namespace;
using statefs::qt::PropertiesSource;
using statefs::qt::make_proper_source;
using qtaround::dbus::async;

static char const *service_name = "org.freedesktop.UPower";

typedef Bridge::Prop Prop;

const QMap<QString, Prop> Bridge::state_ids_{
    {"Percentage", Prop::Percentage}, {"OnBattery", Prop::OnBattery}
    , {"OnLowBattery", Prop::LowBattery}, {"TimeToEmpty", Prop::TimeToEmpty}
    , {"TimeToFull", Prop::TimeToFull}, {"State", Prop::State}};

const Bridge::state_type Bridge::default_state_{
    {87.0, true, false, 878787, 0, UnknownState}};

Bridge::Bridge(PowerNs *ns, QDBusConnection &bus)
    : PropertiesSource(ns)
    , bus_(bus)
    , watch_(bus, service_name)
    , last_state_(default_state_)
    , new_state_(default_state_)
    , actions_(construct_actions())
{

}

Bridge::actions_type Bridge::construct_actions()
{
    auto processState = [this](Prop, QVariant const &v) {
        char const *state = "unknown";
        bool is_charging = false;
        auto state_id = v.toUInt();

        switch (state_id) {
        case UnknownState:
            // do nothing
            break;
        case Charging:
            is_charging = true;
            state = "charging";
            break;
        case Discharging:
            state = "discharging";
            break;
        case Empty:
            state = "empty";
            break;
        case FullyCharged:
            state = "full";
            break;
        case PendingCharge:
            // or maybe should be unknown
            state = "charging";
            break;
        case PendingDischarge:
            // or maybe should be unknown
            state = "discharging";
            break;
        default:
            qWarning() << "Unknown upower state" << state_id;
            break;
        }
        updateProperty("IsCharging", is_charging);
        updateProperty("State", state);
    };

    actions_type res = {{
            [this](Prop, QVariant const &v) {
                updateProperty("ChargePercentage", round(v.toDouble()));
                updateProperty("Capacity", v);
            }, [this](Prop, QVariant const &v) {
                    updateProperty("OnBattery", v);
                    if (v.toBool())
                        updateProperty("TimeUntilFull", 0);
            }, [this](Prop, QVariant const &v) { updateProperty("LowBattery", v);
            }, [this](Prop, QVariant const &v) { updateProperty("TimeUntilLow", v);
            }, [this](Prop, QVariant const &v) { updateProperty("TimeUntilFull", v);
            }, processState
        }};
        return res;
}

void Bridge::update_all_props()
{
    auto update = [this]() {
        auto changed_count = 0;
        for (size_t i = 0; i != propCount; ++i) {
            auto const &now = new_state_[i];
            if (now.isValid() && (now != last_state_[i])) {
                ++changed_count;
                actions_[i](static_cast<Prop>(i), now);
            }
        }
        if (changed_count)
            std::copy(new_state_.begin(), new_state_.end(), last_state_.begin());
    };

    auto setProp = [this](QString const &n, QVariant const &v) {
        auto it = state_ids_.find(n);
        if (it == state_ids_.end()) {
            qWarning() << "Unknown property" << n << v;
            return;
        }
        new_state_[static_cast<size_t>(*it)] = v;
    };

    auto onDevProps = [setProp, update](QVariantMap const &kv) {
        for (QString name : {"Percentage", "TimeToEmpty", "TimeToFull", "State"})
            setProp(name, kv[name]);

        update();
    };

    auto onMgrProps = [setProp, this, update, onDevProps](QVariantMap const &kv) {
        for (QString name : {"OnBattery", "OnLowBattery"})
            setProp(name, kv[name]);

        if (device_props_)
            async(this, device_props_->GetAll(Device::staticInterfaceName())
                  , onDevProps);
        else
            update();
    };

    auto getAll = [this, update, onMgrProps, onDevProps]() {
        if (manager_props_)
            async(this, manager_props_->GetAll(Manager::staticInterfaceName())
                  , onMgrProps);
        else if (device_props_)
            async(this, device_props_->GetAll(Device::staticInterfaceName())
                  , onDevProps);
        else
            update();
    };

    cor::error_trace_msg_nothrow("Updating upower props", getAll);
}

bool Bridge::try_get_battery(QString const &path)
{
    bool found = false;
    auto getBattery = [this, &found, &path]() {
        auto is_battery = [](std::unique_ptr<Device> const &p) {
            return (p->nativePath() == "battery");
        };
        std::unique_ptr<Device> device(new Device(service_name, path, bus_));
        if (!is_battery(device))
            return;

        device_ = std::move(device);
        device_path_ = path;
        if (device_) {
            device_props_.reset(new Properties(service_name, path, bus_));
            connect(device_.get(), &Device::Changed
                    , this, &Bridge::update_all_props);
            found = true;
        } else {
            qWarning() << "No battery found";
        }
    };
    cor::error_trace_msg_nothrow("Checking is device battery", getBattery);
    if (found)
        update_all_props();
    return found;
}

void Bridge::init_manager()
{
    manager_.reset(new Manager(service_name, "/org/freedesktop/UPower", bus_));
    manager_props_.reset(new Properties(service_name, "/org/freedesktop/UPower", bus_));
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

PowerNs::PowerNs(QDBusConnection &bus, statefs_provider_mode mode)
    : Namespace("Battery", make_proper_source<Bridge>(mode, this, bus))
    , defaults_{
    { "ChargePercentage", "87" }
    , { "Capacity", "87" }
    , { "OnBattery", "1" }
    , { "LowBattery", "0" }
    , { "TimeUntilLow", "878787" }
    , { "TimeUntilFull", "0" }
    , { "IsCharging", "0" }
    , { "State", "unknown" }}
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
        auto ns = std::make_shared<PowerNs>
            (bus_, server ? server->mode : statefs_provider_mode_run);
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
