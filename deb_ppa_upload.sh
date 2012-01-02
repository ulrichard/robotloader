#!/bin/sh
# build a debian sourcepackage and upload it to the launchpad ppa
export GPGKEY=DA94BB53
export DEBEMAIL="richi@paraeasy.ch"
export DEBFULLNAME="Richard Ulrich"

rm *.zip
rm -rf RobotLoader_20100712/
rm -rf RobotArm_Examples*
rm -rf RACS-*
rm -rf RAC-MINI.hex
wget http://arexx.com/rp6/downloads/RobotLoader_20100712.zip
wget http://arexx.com/robot_arm/downloads/RobotArmExamples_MINI.zip
wget http://arexx.com/robot_arm/downloads/RACS_v1.0.zip


for DISTRIBUTION in precise oneiric natty maverick 
do
	VERSIONSTR=1.5-12~${DISTRIBUTION}
	sed -i  -e "s/maverick/${DISTRIBUTION}/g" -e "s/natty/${DISTRIBUTION}/g" -e "s/oneiric/${DISTRIBUTION}/g" -e "s/precise/${DISTRIBUTION}/g" debian/changelog
	dpkg-buildpackage -rfakeroot -S
	dput ppa:richi-paraeasy/ppa ../robotloader_${VERSIONSTR}_source.changes
done
