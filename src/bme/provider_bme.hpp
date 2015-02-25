#ifndef _STATEFS_PRIVATE_BME_HPP_
#define _STATEFS_PRIVATE_BME_HPP_

#include <statefs/property.hpp>
#include <statefs/consumer.hpp>

#include "bmeipc.h"

#include <cor/mt.hpp>

#include <poll.h>
#include <fcntl.h>
#include <sys/inotify.h>
#include <sys/eventfd.h>

namespace statefs { namespace bme {

class BatteryNs : public statefs::Namespace
{
public:
    enum class Prop {
        ChargePercentage, ChargeBars, OnBattery, LowBattery
        , TimeUntilLow, TimeUntilFull, IsCharging, State
        , Level, ChargerType, ChargingState
        , EOE // end of enum
    };

    typedef std::tuple<char const*, char const*> info_item_type;
    static const size_t prop_count = static_cast<size_t>(Prop::EOE);
    typedef std::array<info_item_type, prop_count> info_type;

    static const info_type info;

    BatteryNs();
    virtual ~BatteryNs();

    virtual void release() { }

    void set(Prop, std::string const &);

private:
    void initialize_bme();
    void start_listening();

    void onBMEEvent();
    bool readBatteryValues();

    bool initProviderSource();
    void cleanProviderSource();

    int desc;
    struct pollfd ufds[2];
    int exit_handler;
    bool exiting;

    bme_xchg_t xchg;

    std::thread listener;

    std::array<statefs::setter_type, prop_count> setters_;
};

}}

#endif // _STATEFS_PRIVATE_BME_HPP_
