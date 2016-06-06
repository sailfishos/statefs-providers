/*
 * StateFS BlueZ provider
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

#include "provider_bluez.hpp"
#include <iostream>
#include <functional>
#include <cor/util.hpp>

#include <initmanagerjob.h>

#include <QDebug>

namespace statefs { namespace bluez {

using statefs::qt::Namespace;
using statefs::qt::PropertiesSource;
using statefs::qt::make_proper_source;
using qtaround::dbus::async;

static char const *service_name = "org.bluez";

Bridge::Bridge(BlueZ *ns, QDBusConnection &bus)
    : PropertiesSource(ns)
    , bus_(bus)
    , watch_(bus, service_name)
{
}

void Bridge::init()
{
    auto setup_manager = [this]() {
        manager_.reset(new BluezQt::Manager);
        connect(manager_.get(), &BluezQt::Manager::usableAdapterChanged
                , this, &Bridge::usableAdapterChanged);
        BluezQt::InitManagerJob *job = manager_->init();
        job->start();
        connect(job, &BluezQt::InitManagerJob::result
                , [this](BluezQt::InitManagerJob *j) {
                    if (j->error()) {
                        qWarning() << "BluezQt::Manager::init() error:" << j->errorText();
                    }
                });
    };
    auto reset_manager = [this]() {
        manager_.reset();
        static_cast<BlueZ*>(target_)->reset_properties();
    };
    watch_.init(setup_manager, reset_manager);
    setup_manager();
}

void Bridge::usableAdapterChanged(BluezQt::AdapterPtr adapter)
{
    qDebug() << "Found default bluetooth adapter" << adapter->ubi();

    adapter_ = adapter;

    connect(adapter_.data(), &BluezQt::Adapter::poweredChanged
            , [this](bool powered) {
                updateProperty("Powered", powered);
            });
    connect(adapter_.data(), &BluezQt::Adapter::discoverableChanged
            , [this](bool discoverable) {
                updateProperty("Discoverable", discoverable);
            });
    connect(adapter_.data(), &BluezQt::Adapter::deviceRemoved
            , this, &Bridge::removeDevice);
    connect(adapter_.data(), &BluezQt::Adapter::deviceAdded
            , this, &Bridge::addDevice);

    updateProperty("Address", adapter->address());
    updateProperty("Powered", adapter->isPowered());
    updateProperty("Discoverable", adapter->isDiscoverable());

    foreach (BluezQt::DevicePtr device, adapter->devices()) {
        addDevice(device);
    }
}

void Bridge::addDevice(BluezQt::DevicePtr device)
{
    removeDevice(device);

    QDBusObjectPath deviceId(device->ubi());

    connect(device.data(), &BluezQt::Device::connectedChanged
        , [this,deviceId](bool connected) {
            if (connected) {
                connected_.insert(deviceId);
            } else {
                connected_.erase(deviceId);
            }
            updateProperty("Connected", connected_.size() > 0);
        });

    if (device->isConnected()) {
        connected_.insert(deviceId);
    } else {
        connected_.erase(deviceId);
    }

    devices_.insert(std::make_pair(deviceId, std::move(device)));
}

void Bridge::removeDevice(BluezQt::DevicePtr device)
{
    auto it = devices_.find(QDBusObjectPath(device->ubi()));
    if (it != devices_.end())
        devices_.erase(it);
    if (connected_.erase(QDBusObjectPath(device->ubi())))
        updateProperty("Connected", connected_.size() > 0);
}

BlueZ::BlueZ(QDBusConnection &bus, statefs_provider_mode mode)
    : Namespace("Bluetooth", make_proper_source<Bridge>(mode, this, bus))
    , defaults_({
            { "Powered", "0" }
            , { "Discoverable", "0" }
            , { "Connected", "0" }
            , { "Address", "00:00:00:00:00:00" }})
{
    addProperty("Enabled", "0", "Powered");
    addProperty("Visible", "0", "Discoverable");
    addProperty("Connected", "0");
    addProperty("Address", "00:00:00:00:00:00");
    src_->init();
}

void BlueZ::reset_properties()
{
    setProperties(defaults_);
}


class Provider : public statefs::AProvider
{
public:
    Provider(statefs_server *server)
        : AProvider("bluez", server)
        , bus_(QDBusConnection::systemBus())
    {
        auto ns = std::make_shared<BlueZ>
            (bus_, server ? server->mode : statefs_provider_mode_run);
        insert(std::static_pointer_cast<statefs::ANode>(ns));
    }
    virtual ~Provider() {}

    virtual void release() {
        delete this;
    }

private:
    QDBusConnection bus_;
};

static Provider *provider = nullptr;

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
    return statefs::bluez::init_provider(server);
}
