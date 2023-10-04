#!/bin/bash

# This build script assumes that the kauailabs allwpilib repo, and 
# any other needed repos, have already been acquired.

set -x

for BUILDTYPE in Debug Release
do
	for BUILD_CPP_ONLY in ON OFF
	do

		if [ "$BUILD_CPP_ONLY" == "ON" ]; then
			SHARED_LIBS=OFF
			BUILD_DIR=${BUILDTYPE}_Static
		else
			SHARED_LIBS=ON
			BUILD_DIR=${BUILDTYPE}_Shared
		fi

		cd ~
		sudo rm -r wpilib_build_${BUILD_DIR}
		mkdir wpilib_build_${BUILD_DIR}/
		cd wpilib_build_${BUILD_DIR}
		export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-armhf
		cmake -D WITHOUT_ALLWPILIB=OFF -D WITHOUT_JAVA=${BUILD_CPP_ONLY} -DCMAKE_BUILD_TYPE=${BUILDTYPE} -DBUILD_SHARED_LIBS=${SHARED_LIBS} -D OpenCV_DIR=/home/pi/opencv/build ~/allwpilib
		make -j5
		sudo make install
		cd hal
		gradle --rerun-tasks publishMavenPublicationToMavenLocal

		# Copy VMX-pi Channel Maps.  Note that this copying is now optional; defaults are used if the ChannelMap is not present.
		#sudo cp -r ~/allwpilib/hal/src/main/native/mau/Translator/Maps/ChannelMap.json /usr/local/frc/third-party/lib

		# Not sure why this is here - it may no longer be necessary
		sudo mkdir /tmp/frc_versions
	done
done

# Once all the builds are complete, publish the final results to MavenCentral
cd ~
cd ~/.m2/repository/com/kauailabs/vmx/first/hal/hal-cpp
latest_subdir=$(ls -td -- */ | head -n 1)
cd $latest_subdir
ls *
jar -cvf bundle.jar *.pom *.asc *.zip
ls *

publish_bundle_dir=$(pwd)
echo "******************************************************"
echo
echo "Upload the bundle file ($publish_bundle_dir/bundle.jar) as an Artifact Bundle on the Staging Upload page at the Sonatype OSS site (oss.sonatype.org)"
echo
echo "******************************************************"
