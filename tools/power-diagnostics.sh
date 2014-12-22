#!/bin/bash

if $(who) != "root"; then
    echo "Please execute this script as root" 1>&2
    exit 1
fi

function sep
{
    echo
    echo "@${@}@"
}

sep rpm
rpm -qa

sep rpm-upower
rpm -ql upower

sep rpm-statefs
rpm -ql statefs-provider-upower

sep files 
ls -al /usr/libexec/upowerd
find /usr/lib/statefs/ -type f -exec ls -al {} \;

sep provider-info
find /var/lib/statefs -name '*power*' -print -exec cat {} \;

sep ps
ps aux

sep sysfs
find /sys/class/power_supply/ -follow -maxdepth 2 -name uevent -print -exec cat {} \;

sep upower
upower -d

function dump_file
{
    for f in $@; do
        ls -al $f
        cat $f
        echo
    done
}

export -f dump_file

sep statefs
find /run/state/namespaces/Battery -follow -type f | xargs bash -c 'dump_file "$@"'


