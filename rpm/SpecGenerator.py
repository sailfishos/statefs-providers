#!/usr/bin/python

import SpecGenerator

import os, sys, re, itertools
from optparse import OptionParser


def action(fn):
    setattr(fn, 'action', True)
    return fn

class Templates:
    def __init__(self):
        self.re = re.compile(r'.*@@([a-z0-9A-Z_-]+)@@.*')

templates = Templates()

templates.package_qt5 = '''
%package -n {name}
Summary: Statefs provider{summary}
Group: System/Libraries
Requires(post): /sbin/ldconfig
Requires(postun): /sbin/ldconfig
Requires: %{{n_common}} = %{{version}}-%{{release}}
Requires: statefs-loader-qt5 >= 0.0.9
{extra}
{obsoletes}
{provides}
{conflicts}
%description -n {name}
%{{summary}}

'''

templates.package_default = '''
%package -n {name}
Summary: Statefs provider{summary}
Group: System/Libraries
Requires(post): /sbin/ldconfig
Requires(postun): /sbin/ldconfig
{extra}
{obsoletes}
{provides}
{conflicts}
%description -n {name}
%{{summary}}

'''

templates.package_inout = '''
%package -n {name}
Summary: Statefs inout provider{summary}
Group: System/Libraries
Requires: statefs >= %{{statefs_ver}}
{extra}
{obsoletes}
{provides}
{conflicts}
BuildArch: noarch
%description -n {name}
%{{summary}}
'''

templates.decl_udev = '''
%if %{{undefined suse_version}}
BuildRequires: boost-filesystem >= 1.51.0
%endif
BuildRequires: boost-devel >= 1.51.0
BuildRequires: pkgconfig(cor-udev) >= 0.1.14
BuildRequires: pkgconfig(statefs-util) >= %{{statefs_ver}}
{aux}
'''

templates.decl_udev_mer = '''
Obsoletes: statefs-provider-upower <= 0.2.66.1
Provides: statefs-provider-upower = 0.2.66.1
'''

templates.decl_bluez = '''
BuildRequires: pkgconfig(KF5BluezQt)
'''

templates.decl_bme = '''
Requires: bme-rm-680-bin >= 0.9.95
'''

templates.decl_upower = '''
Requires: upower >= 0.9.18
'''

templates.decl_connman = '''
Requires: connman >= 1.15
'''

templates.decl_ofono = '''
Requires: ofono >= 1.12
'''

templates.decl_mce = '''
BuildRequires: pkgconfig(mce)
Obsoletes: statefs-provider-inout-mce <= 0.2.43
Provides: statefs-provider-inout-mce = 0.2.44
Obsoletes: statefs-provider-keyboard-generic <= 0.2.73
Provides: statefs-provider-keyboard-generic = 0.2.74
'''

templates.decl_sensors = '''
BuildRequires: pkgconfig(Qt5Sensors)
'''

templates.decl_profile = '''
Requires: profiled >= 0.30
Obsoletes: statefs-provider-inout-profile <= 0.2.44.99
Provides: statefs-provider-inout-profile = 0.2.44.99
'''

templates.decl_keyboard_generic = '''
BuildRequires: pkgconfig(cor-udev) >= 0.1.14
'''

templates.make = '''
%cmake -DVERSION=%{{version}} %{{?_with_multiarch:-DENABLE_MULTIARCH=ON}} {options}
make %{{?jobs:-j%jobs}}
make doc
'''

def filter_out(prefix, data, filter_fn):
    t = type(data)
    if t == list:
        res = [filter_out(prefix, x, filter_fn) for x in data]
        return [x for x in res if x is not None]
    elif t == dict:
        res = [(filter_fn(k), v) for k, v in data.items()]
        res = [x for x in res if x[0] is not None]
        return dict(res)
    else:
        return data if filter_fn('_'.join((prefix, str(data)))) else None

def setup(d, target):
    filters = tuple()
    make_options = ""
    udev_conflicts = [ "upower", "inout-power" ]
    udev_aux = ""
    if target == "mer":
        filters = ['.*upower', '.*keyboard_generic']
        make_options = "-DENABLE_UPOWER=OFF"
        udev_aux = templates.decl_udev_mer
        udev_conflicts = [ "inout-power" ]
    else:
        filters = ['.*mce']

    def mk_filter_fn(expr):
        r = re.compile(expr)
        def dump(r,  v):
            res = r.match(v)
            return res
        return lambda v: r.match(v) is None

    templates.filters = [mk_filter_fn(x) for x in filters]
    templates.make = templates.make.format(options = make_options)
    templates.decl_udev = templates.decl_udev.format(aux = udev_aux)

    d.templates = {
        "qt5" : templates.package_qt5
        , "default": templates.package_default
        , "inout" : templates.package_inout }

    d.extra = {
        "qt5" : {
            "bluez" : templates.decl_bluez
            , "upower" : templates.decl_upower
            , "connman" : templates.decl_connman
            , "ofono" : templates.decl_ofono
            , "mce" : templates.decl_mce
            , "sensors" : templates.decl_sensors
            , "profile" : templates.decl_profile
        }, "default" : {
            "power_udev" : templates.decl_udev
            , "keyboard_generic" : templates.decl_keyboard_generic
            , "bme" : templates.decl_bme
        }
    }

    d.provides = {
        "qt5" : {
            "bluez" : "bluetooth"
            , "upower" : "power"
            , "connman" : ["internet", "network"]
            , "ofono" : "cellular"
            , "mce" : "system"
            , "sensors" : "sensors"
            , "profile" : "profile-info"
        }, "default" : {
            "power_udev" : "power"
            , "keyboard_generic" : "keyboard"
            , "bme" : "power"
        }, "inout" : {
            "bluetooth" : "bluetooth"
            , "power" : "power"
            , "network" : ["internet", "network"]
            , "cellular" : "cellular"
            , "mode_control" : "system"
            , "keyboard" : "keyboard"
            , "profile" : "profile-info"
            , "location" : "location"
        }
    }

    d.conflicts = {
        "qt5" : {
            "bluez" : "inout-bluetooth"
            , "upower" : [ "power-udev", "inout-power" ]
            , "connman" : "inout-network"
            , "ofono" : "inout-cellular"
            , "mce" : "inout-mode-control"
            , "profile" : "inout-profile"
        }, "default" : {
            "power_udev" : udev_conflicts
            , "keyboard_generic" : "inout-keyboard"
            , "bme" : [ "upower", "inout-power" ]
        }, "inout" : {
            "bluetooth" : "bluez"
            , "power" : [ "upower", "power-udev" ]
            , "network" : [ "connman" ]
            , "cellular" : "ofono"
            , "mode_control" : "mce"
            , "keyboard" : "keyboard_generic"
            , "profile" : "profile"
            , "location" : "geoclue"
        }
    }

    d.summaries = {
        "qt5" : {
            "bluez" : ", source - bluez"
            , "upower" : ", source - upower"
            , "connman" : ", source - connman"
            , "ofono" : ", source - ofono"
            , "mce" : ", source - mce"
            , "sensors" : ", source - sensors"
            , "profile" : ", source - profiled"
        }, "default" : {
            "power_udev" : ", source - sysfs/udev"
            , "keyboard_generic" : ", source - sysfs/udev"
            , "bme" : ", source - bme"
            , "back_cover" : ", source - back_cover"
        }, "inout" : {
            "bluetooth" : ": bluetooth properties"
            , "power" : ": power properties"
            , "network" : ": network properties"
            , "cellular" : ": cellular properties"
            , "mode_control" : ": system properties"
            , "keyboard" : ": keyboard properties"
            , "profile" : ": profile properties"
            , "location" : ": location properties"
        }
    }

    d.obsoletes = {
        "meego" : {
            "bluetooth" : "bluetooth"
            , "power" : "battery-upower"
            , "internet" : "internet"
            , "cellular" : ["cellular", "phone"]
            , "location" : [ "location-geoclue", "location-skyhook" ]
        }
        , "maemo" : {
            "system" : "mce"
        }, "ckit" : {
            "bluetooth" : ["bluez", "bluetooth"]
            , "power" : ["power", "upower", "power-bme"]
            , "internet" : "connman"
            , "cellular" : ["cellular", "ofono"]
            , "system" : "mce"
            , "keyboard" : "keyboard-generic"
            , "profile-info" : "profile"
            , "location" : ["location-gypsy", "location-skyhook", "location"]
        }
    }

    d.old_formats = {
        "meego" : "contextkit-meego-{}"
        , "maemo" : "contextkit-maemo-{}"
        , "ckit" : "contextkit-plugin-{}"
    }

    d.versions = {
        "meego" : [ "%{meego_ver}", "%{meego_ver1}" ]
        , "maemo" : [ "%{maemo_ver}", "%{maemo_ver1}" ]
        , "ckit" : [ "%{ckit_version}", "%{ckit_version1}" ]
    }

    def is_suitable(v):
        res = True
        for fn in templates.filters:
            res = fn(v)
            if not res:
                break
        return res

    only_suitable = lambda pk_type, x: filter_out(pk_type, x, is_suitable)
    d.qt5_system = only_suitable('qt5', ["bluez", "upower", "connman", "ofono", "mce", "sensors"])
    d.qt5_user = only_suitable('qt5', ["profile"])

    d.default_system = only_suitable('default', ["power_udev", "bme", "back_cover", "keyboard_generic"])

    d.old_names = { "keyboard_generic" : "keyboard-generic" }

    d.inout_system = only_suitable('inout', ["bluetooth", "power", "network", "cellular", "mode_control"
                                             , "keyboard", "location"])
    d.inout_user = only_suitable('inout', ["profile"])

    d.packages = {
        "qt5" : d.qt5_system + d.qt5_user
        , "default" : d.default_system
        , "inout" : d.inout_system + d.inout_user
    }

    d.inout_user = ["inout_" + a for a in d.inout_user]
    d.inout_system = ["inout_" + a for a in d.inout_system]

def mk_pkg_name(name):
    return name.replace('_', '-')

class Actions:

    def pk_type_name_data(self, src, pk_type, name, defval):
        res = src.get(pk_type, None)
        if res:
            res = res.get(name, None)

        return res or defval

    def pk_type_name_list(self, src, pk_type, name):
        res = self.pk_type_name_data(src, pk_type, name, [])
        if type(res) == str:
            res = [res]
        return res

    def get_extra(self, pk_type, name):
        res = self.pk_type_name_data(Actions.extra, pk_type, name, "")
        return res.strip().split('\n')

    def get_summary(self, pk_type, name):
        return Actions.summaries[pk_type][name]

    def generic_name(self, pk_type, name):
        src = Actions.provides[pk_type]
        res = src.get(name, name)
        return res if type(res) == str else res[0]

    def get_provides(self, pk_type, name):
        src = Actions.provides[pk_type]
        res = src.get(name, None)
        if res is None:
            return res

        if type(res) == str:
            res = [res]
        fmt = "Provides: statefs-provider-{} = %{{version}}-%{{release}}"
        return [fmt.format(mk_pkg_name(v)) for v in res]

    def get_conflicts(self, pk_type, name):
        src = Actions.conflicts[pk_type]
        res = src.get(name, None)
        if res is None:
            return []
        if type(res) == str:
            res = [res]
        fmt = "Conflicts: statefs-provider-{}"
        return [fmt.format(mk_pkg_name(v)) for v in res]

    def get_obsoletes(self, pk_type, old_type, name):
        name = self.generic_name(pk_type, name)
        fmt = Actions.old_formats[old_type]
        pkgs = self.pk_type_name_list(Actions.obsoletes, old_type, name)

        ver = Actions.versions[old_type]
        def obsolete(name):
            old_name = mk_pkg_name(fmt.format(name))
            return ("Obsoletes: {} <= {}".format(old_name, ver[0])
                    , "Provides: {} = {}".format(old_name, ver[1]))

        return list(itertools.chain(*[obsolete(x) for x in pkgs]))

    def package_name(self, pk_type, name):
        name = mk_pkg_name(name)
        if pk_type == "inout":
            return "statefs-provider-inout-{}".format(name)
        else:
            return "statefs-provider-{}".format(name)

    def get_package_description(self, pk_type, name):
        old_types = Actions.old_formats.keys()
        obsoletes = itertools.chain(*[self.get_obsoletes(pk_type, t, name)
                                      for t in old_types])
        res = Actions.templates[pk_type].format(
            name = self.package_name(pk_type, name),
            summary = self.get_summary(pk_type, name),
            obsoletes = '\n'.join(list(obsoletes)),
            conflicts = '\n'.join(self.get_conflicts(pk_type, name)),
            extra = '\n'.join(self.get_extra(pk_type, name)),
            provides = '\n'.join(self.get_provides(pk_type, name) or [])
        )

        res = [x for x in res.split('\n') if x]
        res.append("\n")
        return '\n'.join(res)

    def __init__(self, l):
        self.line = l

    def replace__(self, tpl, src, **kwargs):
        res = ""
        for name in src:
            old_name = Actions.old_names.get(name, name)
            res += tpl.format(name = name, old_name=old_name, **kwargs)
        return res.split("\n")

    @action
    def providers(self, name):
        src = getattr(Actions, name)
        with open(name + "-providers.spec.tpl") as f:
            return self.replace__(''.join(f.readlines()), src)

    def install__(self, name, kind):
        src = getattr(Actions, '_'.join((name, kind)), None)
        if src is None:
            return
        with open(name + "-install.spec.tpl") as f:
            return self.replace__(''.join(f.readlines()), src, kind = kind)

    @action
    def install(self, name):
        res = self.install__(name, "system") or []
        res.extend(self.install__(name, "user") or [])
        return res

    @action
    def declare(self, name):
        res = [self.get_package_description(name, p).split('\n')
               for p in Actions.packages[name]]
        return list(itertools.chain(*res))

    @action
    def make(self, name):
        return templates.make.split('\n')

def replaced(l):
    m = templates.re.match(l)
    if (m is None):
        return (l,)
    actions = Actions(l)
    (part, name) = m.group(1).split("-")
    action = getattr(actions, part)
    if not hasattr(action, 'action'):
        raise Exception("There is no action {}".format(part))
    return action(name)


def process(out_file, target):
    setup(Actions, target)

    #print Actions("").get_obsoletes("qt5", "meego", "bluez")
    #print Actions("").get_provides("qt5", "connman")
    #print Actions("").get_package_description("qt5", "connman")
    #print Actions("").get_package_description("inout", "power")
    with open("statefs-providers.spec.tpl") as f:
        lines = f.readlines()
        res = itertools.chain(*[replaced(l.strip()) for l in lines])
        #res = list(itertools.chain(*[replaced(l) for l in res]))
        with open(out_file, "w") as out:
            out.write('\n'.join(list(res)))
            out.write('\n')

if __name__ == '__main__':
    process(out_file = "statefs-providers.spec")
