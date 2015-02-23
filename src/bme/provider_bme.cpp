/*
 * StateFS bme provider
 *
 * Copyright (C) 2013 0x7DD.
 * Contact: Andrey Kozhevnikov <coderusinbox@gmail.com>
 *
 * Copyright (C) 2013, 2015 Jolla Ltd.
 * Contact: Denis Zalevskiy <denis.zalevskiy@jollamobile.com>
 *
 * Copyright (C) 2010 Nokia Corporation.
 * Contact: Marius Vollmer marius.vollmer@nokia.com
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

#include "provider_bme.hpp"

#define NANOSECS_PER_MIN (60 * 1000 * 1000LL)

namespace statefs { namespace bme {

using std::make_tuple;

template <typename T>
static inline std::string statefs_attr(T const &v)
{
    return std::to_string(v);
}

static inline std::string statefs_attr(bool v)
{
    return std::to_string(v ? 1 : 0);
}

const BatteryNs::info_type BatteryNs::info = {{
    make_tuple("ChargePercentage", "62")
    , make_tuple("ChargeBars", "5")
    , make_tuple("OnBattery", "1")
    , make_tuple("LowBattery", "0")
    , make_tuple("TimeUntilLow", "3600")
    , make_tuple("TimeUntilFull", "0")
    , make_tuple("IsCharging", "0")
    , make_tuple("State", "unknown")
}};

BatteryNs::BatteryNs() : Namespace("Battery")
{
    for (size_t i = 0; i < prop_count; ++i) {
        char const *name;
        char const *defval;
        std::tie(name, defval) = info[i];
        auto prop = statefs::create(statefs::Discrete{name, defval});
        setters_[i] = setter(prop);
        *this << prop;
    }

    memset(ufds, 0 , sizeof(ufds));

    exiting = false;

    exit_handler = eventfd(0, 0);
    if (exit_handler >= 0) {
        ufds[1].fd = exit_handler;
        ufds[1].events = POLLIN;

        initialize_bme();

        start_listening();
    }
}

BatteryNs::~BatteryNs()
{
    eventfd_write(exit_handler, 1);
    if (listener.joinable())
        listener.join();
    if (xchg != BME_XCHG_INVAL) {
        bme_xchg_close(xchg);
    }
}

void BatteryNs::initialize_bme()
{
    xchg = bme_xchg_open();
    if (xchg == BME_XCHG_INVAL) {
        return;
    }
    fcntl(bme_xchg_inotify_desc(xchg), F_SETFD, FD_CLOEXEC);

    readBatteryValues();

    desc = bme_xchg_inotify_desc(xchg);
    if (desc >= 0) {
        ufds[0].fd = desc;
        ufds[0].events = POLLIN;
    }
    else {
        exiting = true;
    }
}

void BatteryNs::set(Prop id, std::string const &v)
{
    setters_[static_cast<size_t>(id)](v);
}

void BatteryNs::start_listening()
{
    std::cerr << "listening\n";
    listener = std::thread([this]() {
        int rv = 0;

        while (!exiting)
        {
            rv = poll(ufds, 2, -1);
            if (rv < 0) {
                std::cerr << "poll error\n";

                ::usleep(10000000);
                initialize_bme();
            } else if (rv == 0) {
                std::cerr << "Timeout!\n";
            } else {
                if (ufds[1].revents & POLLIN) { // exit event
                    break;
                }
                if (ufds[0].revents & POLLIN) { // bme event
                    onBMEEvent();
                }
            }
        }
    });
    std::cerr << "continue execution\n";
}

void BatteryNs::onBMEEvent()
{
    std::cerr << "onBMEEvent\n";
    inotify_event ev;
    int rc;
    rc = bme_xchg_inotify_read(xchg, &ev);
    if (rc < 0) {
        std::cerr << "can't read bmeipc xchg inotify event\n";
        return;
    }

    // XXX: should we read the .bmeevt file and only act on relevant events?

    if ((ev.mask & IN_DELETE_SELF) || (ev.mask & IN_MOVE_SELF)) {
        cleanProviderSource();
        initProviderSource();
    } else if (!(ev.mask & IN_IGNORED)) {
        readBatteryValues();
    }
}

bool BatteryNs::readBatteryValues()
{
    std::cerr << "readBatteryValues\n";
    bme_stat_t st;
    int sd = -1;

    if ((sd = bme_open()) < 0) {
        std::cerr << "Cannot open socket connected to BME server\n";
        return false;
    }

    if (bme_stat_get(sd, &st) < 0) {
        std::cerr << "Cannot get BME statistics\n";
        bme_close(sd);
        return false;
    }

    bme_bat_state bat_state = (bme_bat_state)st[bme_stat_bat_state];
    bme_charging_state charging_state =
        (bme_charging_state)st[bme_stat_charging_state];
    if (bat_state > bme_bat_state_err)
        bat_state = bme_bat_state_err;
    bme_charger_state charger_state = (bme_charger_state)st[bme_stat_charger_state];

    bool _isCharging = charging_state == bme_charging_state_started
                       && bat_state != bme_bat_state_full;
    set(Prop::IsCharging, statefs_attr(_isCharging));

    bool _onBattery = charger_state != bme_charger_state_connected;
    set(Prop::OnBattery, statefs_attr(_onBattery));

    bool _lowBattery = bat_state == bme_bat_state_low;
    set(Prop::LowBattery, statefs_attr(_lowBattery));

    int cp = st[bme_stat_bat_pct_remain];
    std::cerr << "ChargePercentage: " << statefs_attr(cp) << "\n";
    set(Prop::ChargePercentage, statefs_attr(cp));

    if (st[bme_stat_bat_units_max] != 0) {
        set(Prop::ChargeBars, statefs_attr(st[bme_stat_bat_units_now]));
    } else {
        set(Prop::ChargeBars, statefs_attr(0));
    }

    char const *state = "unknown";
    if (_isCharging) {
        state = "charging";
    } else {
        switch (bat_state) {
        case bme_bat_state_empty:
            state = "empty";
            break;
        case bme_bat_state_low:
        case bme_bat_state_ok:
            state = "discharging";
            break;
        case bme_bat_state_full:
            state = "full";
            break;
        case bme_bat_state_err:
            // use default value 
            break;
        }
    }
    set(Prop::State, state);

    set(Prop::TimeUntilFull, statefs_attr(st[bme_stat_charging_time_left_min] * NANOSECS_PER_MIN));

    set(Prop::TimeUntilLow, statefs_attr(st[bme_stat_bat_time_left] * NANOSECS_PER_MIN));

    bme_close(sd);

    return true;
}

bool BatteryNs::initProviderSource()
{
    std::cerr << "initProviderSource\n";
    int rc = bme_inotify_watch_add(xchg);
    if (rc < 0) {
        return false;
    }
    return true;
}

void BatteryNs::cleanProviderSource()
{
    std::cerr << "cleanProviderSource\n";
    bme_inotify_watch_rm(xchg);
}

class Provider;
static Provider *provider = nullptr;

class Provider : public statefs::AProvider
{
public:
    Provider(statefs_server *server)
        : AProvider("bme", server)
    {
        ns = std::make_shared<BatteryNs>();
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
    std::shared_ptr<BatteryNs> ns;
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
    return statefs::bme::init_provider(server);
}
