#!/bin/sh
# build a debian sourcepackage and upload it to the launchpad ppa
export GPGKEY=DA94BB53
export DEBEMAIL="richi@paraeasy.ch"
export DEBFULLNAME="Richard Ulrich"


rm -rf build
rm -rf build-*
rm *-stamp
rm -rf racs/qt/build
rm -rf racs/uc/build
rm -rf racs/uc/.dep
rm racs/uc/*.e*
rm racs/uc/*.hex
rm racs/uc/*.lst
rm racs/uc/*.map
rm racs/uc/*.o
rm racs/uc/*.cmake
rm -rf ros/arexx_robotarm/build
rm -rf ros/arexx_robotarm/bin
rm -rf ros/uc_arm/build
rm -rf debian/build*
rm -rf debian/tmp
rm -rf debian/arexx-robotarm-examples/
rm -rf debian/arexx-robotarm-racsqt/
rm -rf debian/arexx-robotarm-examples/
rm -rf debian/arexx-robot-arm/
rm debian/*.log
rm debian/*.debhelper
rm debian/*.substvars
rm debian/files
rm debian/substvars

rm *.zip
rm -rf RobotLoader_20100712/
rm -rf RobotArm_Examples*
rm -rf RACS-*
rm -rf RAC-MINI.hex
wget http://arexx.com/rp6/downloads/RobotLoader_20100712.zip


:${VERSIONNBR:=$(parsechangelog | grep Version | sed -e "s/Version: //g" -e "s/\\~.*//g")}

for DISTRIBUTION in precise oneiric natty maverick 
do
	sed -i  -e "s/maverick/${DISTRIBUTION}/g" -e "s/natty/${DISTRIBUTION}/g" -e "s/oneiric/${DISTRIBUTION}/g" -e "s/precise/${DISTRIBUTION}/g" debian/changelog
	dpkg-buildpackage -rfakeroot -S
	dput ppa:richi-paraeasy/ppa ../arexx-robot-arm_${VERSIONNBR}~${DISTRIBUTION}_source.changes
done
