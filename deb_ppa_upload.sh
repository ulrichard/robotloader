#!/bin/sh
# build a debian sourcepackage and upload it to the launchpad ppa
export GPGKEY=DA94BB53
export DEBEMAIL="richi@paraeasy.ch"
export DEBFULLNAME="Richard Ulrich"

rm *.zip
wget http://arexx.com/rp6/downloads/RobotLoader_20100712.zip
wget http://arexx.com/robot_arm/downloads/RobotArmExamples_MINI.zip
wget http://arexx.com/robot_arm/downloads/RACS_v1.0.zip

for DISTRIBUTION in precise oneiric natty maverick 
do
	VERSIONSTR=1.5-8~${DISTRIBUTION}
	sed -i  -e "s/maverick/${DISTRIBUTION}/g" -e "s/natty/${DISTRIBUTION}/g" -e "s/oneiric/${DISTRIBUTION}/g" -e "s/precise/${DISTRIBUTION}/g" debian/changelog
	dpkg-buildpackage -rfakeroot -S
	dput ppa:richi-paraeasy/ppa ../robotloader_${VERSIONSTR}_source.changes
done
