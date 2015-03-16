#ifndef _STATEFS_QT_NS_HPP_
#define _STATEFS_QT_NS_HPP_

#include <statefs/qt/util.hpp>
#include <statefs/property.hpp>

#include <map>
#include <memory>
#include <QString>
#include <QVariant>

namespace statefs { namespace qt {

class Namespace;

typedef std::vector<std::pair<char const*, char const*> > DefaultProperties;

class PropertiesSource
{
public:
    PropertiesSource(Namespace *ns) : target_(ns) {}
    virtual ~PropertiesSource() {}
    virtual void init() =0;
    void setProperties(QVariantMap const &);
    void setProperties(std::map<QString, QVariant> const &);
    void updateProperty(const QString &, const QVariant &);
protected:
    Namespace *target_;
};

class DummySource : public PropertiesSource
{
public:
    DummySource(Namespace *ns) : PropertiesSource(ns) {}
    virtual void init() {}
};

template <typename T, typename ... Args>
std::unique_ptr<PropertiesSource> make_source(Args&& ... args)
{
    return std::unique_ptr<PropertiesSource>(new T(std::forward<Args>(args)...));
}

static inline std::unique_ptr<PropertiesSource> dummy_source(Namespace *ns)
{
    return std::unique_ptr<PropertiesSource>(new DummySource(ns));
}

template <typename T, typename NamespaceT, typename ... Args>
std::unique_ptr<PropertiesSource> make_proper_source
(statefs_provider_mode mode, NamespaceT *ns, Args&& ... args)
{
    return (mode == statefs_provider_mode_run
            ? make_source<T>(ns, std::forward<Args>(args)...)
            : dummy_source(ns));
}

class Namespace : public statefs::Namespace
{
public:

    Namespace(char const *, std::unique_ptr<PropertiesSource> &&);

    virtual ~Namespace() {}
    virtual void release() { }

protected:
    void addProperty(char const *, char const *);
    void addProperty(char const *, char const *, char const *);
    void setProperties(DefaultProperties const &);
    void updateProperty(const QString &, const QVariant &);

    std::unique_ptr<PropertiesSource> src_;

private:

    void setProperties(QVariantMap const &);
    void setProperties(std::map<QString, QVariant> const &);

    friend class PropertiesSource;

    std::map<QString, setter_type> setters_for_props_;
};

}}

#endif // _STATEFS_QT_NS_HPP_
