# Manual cross-compilation setup (original guide)

This is the original step-by-step guide, kept for reference and for when you
want to understand or debug what `docker/Dockerfile` automates. For normal use
follow the [Installation section of the README](../README.md#installation).

If you go the manual way, a few shortcuts from the docker setup also apply here:

- Instead of editing `tf2_geometry_msgsConfig.cmake` by hand and copying the
  sysroot's `/usr/lib/aarch64-linux-gnu` and `opencv4` into your host's `/usr`,
  run `docker/rewrite_ros_cmake_paths.sh $SYSROOT` once. It makes all ROS CMake
  configs point into the sysroot.
- The Python requirements no longer need to be installed on the host; the
  build-time check is off by default (`-DCHECK_PYTHON_DEPS=ON` turns it on).
- The final build command is `./cross_compile.sh --host`.

## 1. Install NVIDIA DRIVE OS Docker

Download and install NVIDIA DRIVE OS 6.0.6 and DriveWorks 5.10 from the [NVIDIA Developer portal](https://developer.nvidia.com/docs/drive/drive-os/6.0.6/public/drive-os-linux-installation/common/topics/installation/docker-ngc/download-install-docker-linux-devzone.html).
I am using the DRIVE OS 6.0.6 but the workflow should be compatible with 6.0.10. 
Perhaps 7 (to be tested by someone who has a Thor) I only have an Orin saddly.

Follow the NVIDIA installation guide to:
- Pull the DRIVE OS Docker container
```bash
mkdir nvidia_ws
export WORKSPACE=~/nvidia_ws
sudo docker run -it --privileged --name cross_compile --net=host -v /dev/bus/usb:/dev/bus/usb -v ${WORKSPACE}:/home/nvidia/ nvcr.io/drive/driveos-sdk/drive-agx-orin-linux-aarch64-sdk-build-x86:6.0.6.0-0004
```
Just be aware this is one line not 2!
- Extract DriveWorks SDK
Essentially inside the docker theres is two OS, the one that will be installed (not really but as a method to convey info it is), and the one that compiles and flahes it
The docker has literally everything you need.
The thing is, well, depending on how you want to work the data is in an annoying place.
So Logically this work now becomes a somewhat redundant work about re doing what the really cool guys at NVidia did, but in a way i understand and works... the issue inside the docker is to do some chroots and overlays, because the system is overlayed in an overlay doenst work really nicely, so what i do next is to remove the system from the system.
Inside the docker with a lot of snooping around you will find a folder with the targetfs. we want that.
```bash
docker cp cross_compile:/drive/drive-linux/filesystem/targetfs nvidia_ws/target_fs/
```
OK! now we have the crosscompilation environment at the nvidia_ws/target_fs folder!
Now we need to install inside this work inside this target_fs, Use `chroot` to install ROS Noetic inside the target filesystem. You will need `aptitude` for proper dependency resolution:
```bash
# Enter the target filesystem
export SYSROOT=~/nvidia_ws/target_fs
cd ${SYSROOT}

# Mount necessary filesystems
sudo mount --bind /dev ${SYSROOT}/dev
sudo mount --bind /proc ${SYSROOT}/proc
sudo mount --bind /sys ${SYSROOT}/sys

# Enter chroot environment
sudo chroot ${SYSROOT} /bin/bash
```
ok this is now kinda wild, because if you are inside the container you may feel like youre outside (?) 
but if you did correctly you should be super user in the / of the folder
NOTE: the chroot has no well DNS nameservers youll need to add a DNS in the /etc/resolv.conf (add 'nameserver 1.1.1.1' if in doubt, if cloudfare decided to die try google or any other ofc)

```bash
apt update
apt install -y aptitude software-properties-common
add-apt-repository universe
apt upgrade
```
Ok basic stuff, before we continue we need to install some libraries with aptitude.
the thing is you need to deny the solution a couple times

first thing
```bash
aptitude install libboost-all-dev    
```
you want the solution that does this
```bash
The following actions will resolve these dependencies:

     Downgrade the following packages:
1)     ibverbs-providers [58mlnx43-1.58101 (now) -> 28.0-1ubuntu1 (focal)]
2)     libibverbs1 [58mlnx43-1.58101 (now) -> 28.0-1ubuntu1 (focal)]
3)     librdmacm1 [58mlnx43-1.58101 (now) -> 28.0-1ubuntu1 (focal)]
4)     perftest [4.5-0.18.gfcddfe0.58101 (now) -> 4.4+0.5-1 (focal)]
```
OK that was the "hard" part of the moment!

now we need to install ros noetic as usual
```bash

# Add ROS Noetic repository
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Noetic using aptitude
apt update
apt install ros-noetic-desktop-full
```
you definetly dont need ALL these package but well its easy also notice how the apt is pulling from the arm repo^^
```bash
# Install build dependencies
aptitude install -y build-essential cmake python3-rosdep python3-rosinstall \
  python3-rosinstall-generator python3-wstool \
  libboost-all-dev libtinyxml-dev libtinyxml2-dev liblz4-dev libbz2-dev \
  libapr1 libaprutil1 libconsole-bridge-dev libpoco-dev libgpgme-dev \
  python3-defusedxml python3-rospkg python3-catkin-pkg python3-netifaces \
  liblog4cxx-dev libopencv-dev

# Exit chroot
exit

# Unmount filesystems
sudo umount ${SYSROOT}/dev
sudo umount ${SYSROOT}/proc
sudo umount ${SYSROOT}/sys
```
as a suggestion as you didmake long impactful changes make a copy from the target_fs folder and save it for a minute until everything is ready just for safety

## 4. Cross-Compile ROS Packages

#### Setup Cross-Compilation Toolchain

```bash
# Install ARM64 cross-compiler on host
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# Clone this repository
cd ~/nvidia_ws
git clone git@github.com:roadview-project/NVidiaDriveAGX-Orin_ROS_Driveworks_Node_with_YOLOv8.git DRIVE_AGX_ORIN_ONNX_ROS_integration
cd DRIVE_AGX_ORIN_ONNX_ROS_integration
```

#### Cross-Compile
OK we are now ready to try and break stuff!!! 
QUITE Literally most of the symlinks were broken when we copied we need to fix it so do
```bash
python3 fix_symlinks.py

```
you have to source THE RIGHT ROS.... think like a bootstrap problem. soooo 

```bash
source $SYSROOT/opt/ros/noetic/setup.bash
```
but ofc this itself would be just too easy... so you also need to change the line 96 from $SYSROOT/target_fs/opt/ros/noetic/share/tf2_geometry_msgs/cmake/tf2_geometry_msgsConfig.cmake
to be this:
```cmake
set(_include_dirs "include;${CMAKE_SYSROOT}/usr/share/orocos_kdl/cmake/../../../include;${CMAKE_SYSROOT}/usr/include/eigen3")
```
if this ROS1 wasnt EOL id complain but thats life ig...

ok so now you must be wondering why the hell did we do this like this
well because we need even more libraries but now from the docker, but from outside the QEMU!

basically, $SYSROOT is missing the tensorRT libs that can be found in /usr/include/aarch64-linux-gnu/ on the docker so we need them!
lets copy them!

```bash
docker cp cross_compile:/usr/include/aarch64-linux-gnu/ $SYSROOT/usr/include/
docker cp cross_compile:/usr/lib/aarch64-linux-gnu/ $SYSROOT/usr/lib/
docker cp cross_compile:/usr/local/driveworks/targets^Carch64-Linux $SYSROOT/usr/local/driveworks/targets/ #
docker cp cross_compile:/drive/drive-linux/include/. $SYSROOT/usr/include/
docker cp cross_compile:/drive/drive-linux/include/nvmedia_6x $SYSROOT/usr/include
docker cp cross_compile:/drive/drive-linux/include/nvmedia_6x  $SYSROOT/usr/include/

docker cp cross_compile:/drive/drive-linux/include/nvmedia_dla.h $SYSROOT/usr/include/
docker cp cross_compile:/drive/drive-linux/include/nvmedia_tensor.h $SYSROOT/usr/include/
docker cp cross_compile:/drive/drive-linux/include/nvmedia_tensormetadata.h $SYSROOT/usr/include/


#we also need to make some symlinks so
cd  ~/nvidia_ws/target_fs/usr/local/driveworks
ln -s targets/aarch64-Linux/include/ .
ln -s targets/aarch64-Linux/lib/ .
cd $SYSROOT/usr/include
ln -sf nvmedia_6x/nvmedia_core.h nvmedia_image.h

```
i probably would have a smarter way to do this but this works, the thing is bc we installed ros inside the qemu hes looking for the boost files in the wrong place, even though i changed that in the CMAKE (see in drive_yolo CMAKE) so this is the workaround bc im tired bossman
```bash
sudo cp -r $SYSROOT/usr/lib/aarch64-linux-gnu/ /usr/lib/
sudo ln -s $SYSROOT/usr/include/opencv4 /usr/include/opencv4 # OMG CROSSCOMPILATIOn AHHHH
```
because i was very smart and added some python package to the whole package you need now to run (~Smort++~)
```bash

pip3 install -r  ~/nvidia_ws/DRIVE_AGX_ORIN_ONNX_ROS_integration/src/drive_yolo/requirements.txt
```

FINALLY this should work now

```bash
./cross_compile.sh --host

```

