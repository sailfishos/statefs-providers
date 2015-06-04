/*
 * StateFS MCE provider
 *
 * Copyright (C) 2013-2015 Jolla Ltd.
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

#include "provider_mce.hpp"
#include <math.h>
#include <iostream>
#include <qtaround/dbus.hpp>


#include <mce/dbus-names.h> // from mce-dev
#include <mce/mode-names.h> // from mce-dev

namespace statefs { namespace mce {

using statefs::qt::Namespace;
using statefs::qt::PropertiesSource;
using statefs::qt::make_proper_source;
using qtaround::dbus::sync;
using qtaround::dbus::async;

static char const *service_name = "com.nokia.mce";

Bridge::Bridge(MceNs *ns, QDBusConnection &bus)
    : PropertiesSource(ns)
    , bus_(bus)
    , watch_(new ServiceWatch(bus, service_name))
{
}

void Bridge::init_request()
{
    MceNs *ns = static_cast<MceNs*>(target_);

    auto on_psm = [this](bool v) {
        updateProperty("PowerSaveMode", v);
    };
    auto on_display = [ns](QString const& v) {
        qDebug() << "Display:" << v;
        ns->set(Property::Blanked, v == "off");
    };

    auto on_radio = [this, ns](unsigned v) {
        updateProperty("OfflineMode", (v & MCE_RADIO_STATE_CELLULAR) == 0);
        updateProperty("WlanEnabled", (v & MCE_RADIO_STATE_WLAN) != 0);
        updateProperty("InternetEnabled", (v & MCE_RADIO_STATE_MASTER) != 0);
    };
    auto on_keyboard = [ns](QString const &v) {
        auto is_available = (v == "available");
        ns->set(Property::KeyboardPresent, is_available);
        ns->set(Property::KeyboardOpen, is_available);
    };

    signal_.reset(new MceSignal(service_name, "/com/nokia/mce/signal", bus_));
    connect(signal_.get(), &MceSignal::psm_state_ind, on_psm);
    connect(signal_.get(), &MceSignal::display_status_ind, on_display);
    connect(signal_.get(), &MceSignal::radio_states_ind, on_radio);
    connect(signal_.get(), &MceSignal::keyboard_available_state_ind, on_keyboard);

    request_.reset(new MceRequest(service_name, "/com/nokia/mce/request", bus_));
    async(this, request_->get_psm_state(), on_psm);
    async(this, request_->get_display_status(), on_display);
    async(this, request_->get_radio_states(), on_radio);
    async(this, request_->keyboard_available_state_req(), on_keyboard);
}

void Bridge::init()
{
    auto reset_all = [this]() {
        signal_.reset();
        request_.reset();
    };
    watch_->init([this]() { init_request(); }, reset_all);
    init_request();
}


MceNs::MceNs(QDBusConnection &bus
             , std::shared_ptr<ScreenNs> const &screen
             , std::shared_ptr<KeyboardNs> const &keyboard
             , statefs_provider_mode mode)
    : Namespace("System", make_proper_source<Bridge>(mode, this, bus))
    , screen_(screen)
    , keyboard_(keyboard)
    , defaults_({{"PowerSaveMode", "0"}
            , {"OfflineMode", "0"}
            , {"InternetEnabled", "1"}
            , {"WlanEnabled", "0"}})
{
    setters_[cor::enum_index(Property::Blanked)] = screen_->set_blanked_;
    setters_[cor::enum_index(Property::KeyboardPresent)] = keyboard_->set_present_;
    setters_[cor::enum_index(Property::KeyboardOpen)] = keyboard_->set_open_;

    for (auto v : defaults_)
        addProperty(v.first, v.second);
    src_->init();
}

void MceNs::set(Property id, bool v)
{
    setters_[cor::enum_index(id)](v ? "1" : "0");
}

ScreenNs::ScreenNs()
    : Namespace("Screen")
{
    auto d = statefs::Discrete("Blanked", "0");
    auto prop = statefs::create(d);
    *this << prop;
    set_blanked_ = setter(prop);
}

void MceNs::reset_properties()
{
    setProperties(defaults_);
}

KeyboardNs::KeyboardNs()
    : Namespace("maemo_InternalKeyboard")
{
    auto present = statefs::create(statefs::Discrete("Present", "0"));
    *this << present;
    set_present_ = setter(present);

    auto open = statefs::create(statefs::Discrete("Open", "0"));
    *this << open;
    set_open_ = setter(open);
}

class Provider;
static Provider *provider = nullptr;

class Provider : public statefs::AProvider
{
public:
    Provider(statefs_server *server)
        : AProvider("mce", server)
        , bus_(QDBusConnection::systemBus())
    {
        auto screen_ns = std::make_shared<ScreenNs>();
        auto keyboard_ns = std::make_shared<KeyboardNs>();
        auto ns = std::make_shared<MceNs>
            (bus_, screen_ns, keyboard_ns, server
             ? server->mode : statefs_provider_mode_run);
        insert(std::static_pointer_cast<statefs::ANode>(ns));
        insert(std::static_pointer_cast<statefs::ANode>(screen_ns));
        insert(std::static_pointer_cast<statefs::ANode>(keyboard_ns));
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
    return statefs::mce::init_provider(server);
}
