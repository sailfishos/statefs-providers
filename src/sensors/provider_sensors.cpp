/**
 * @file propvider_sensors.cpp
 * @brief Statefs provider of Sensor namespace
 * @copyright (C) 2015 Jolla Ltd.
 * @par License: LGPL 2.1 http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 */

#include <statefs/util.hpp>
#include <statefs/qt/ns.hpp>
#include <QOrientationSensor>

namespace statefs { namespace sensors {

using statefs::qt::PropertiesSource;
using statefs::qt::Namespace;
using statefs::qt::make_proper_source;

class SensorNs;
class Bridge : public QObject, public PropertiesSource
{
    Q_OBJECT;
public:
    Bridge(SensorNs *);
    virtual void init();
private:
    std::unique_ptr<QOrientationSensor> src_;
};

/**
 * @addtogroup statefs_properties
 *
 * @section sensor_ns Sensor namespace properties:
 *
 * - Orientation [unknown, top, bottom, left, right, face, back] -
 *   reports which part of the device points up
 */
class SensorNs : public Namespace
{
public:
    SensorNs(statefs_provider_mode);
private:
    friend class Bridge;
    statefs::qt::DefaultProperties defaults_;
};

Bridge::Bridge(SensorNs *ns) : PropertiesSource(ns) {}

static const QString orientation_names[] = {
    "unknown", "top", "bottom", "left"
    , "right", "face", "back"
};

void Bridge::init()
{
    auto onChanged = [this]() {
        auto v = src_->reading()->orientation();
        size_t i = ((v >= 0
                   && v < sizeof(orientation_names)/sizeof(orientation_names[0]))
                    ? v : 0);
        updateProperty("Orientation", orientation_names[i]);
    };
    src_ = cor::make_unique<QOrientationSensor>();
    connect(src_.get(), &QSensor::readingChanged , onChanged);
    src_->start();
}

SensorNs::SensorNs(statefs_provider_mode mode)
    : Namespace("Sensor", make_proper_source<Bridge>(mode, this))
    , defaults_({{"Orientation", "unknown"}})
{
    for (auto v : defaults_)
        addProperty(v.first, v.second);
    if (mode == statefs_provider_mode_run)
        src_->init();
}

class Provider;
static Provider *provider = nullptr;

class Provider : public statefs::AProvider
{
public:
    Provider(statefs_server *server)
        : AProvider("sensors", server)
    {
        auto ns = std::make_shared<SensorNs>
            (server ? server->mode : statefs_provider_mode_run);
        insert(std::static_pointer_cast<statefs::ANode>(ns));
    }
    virtual ~Provider() {}

    virtual void release() {
        if (this == provider) {
            delete provider;
            provider = nullptr;
        }
    }
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
    return statefs::sensors::init_provider(server);
}

#include <provider_sensors.moc>
