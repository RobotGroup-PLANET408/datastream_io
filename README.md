# datastream_io
## A toolkit for data conversion and data stream management

**Introduction:**
The datastream_io is a toolkit used to convert data format and manage data stream, which supports the multiple data format conversion *(ros standard format, text format, binary format, ublox/rtcm and other GNSS data format)* and multiple data communication mode *(file, ros, tcp, serial)*. We contribute this toolkit to the community and hope that it will help researchers avoid the problem of data format conversion.

**Features:**
- multiple data format in multi-sensors fusion: ros standard, text format, binary format, ublox/rtcm and other GNSS data format
- data conversion
- data communication: file, ros, tcp, serial

**Authors:**
The [PLANET] in WHU-SGG

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit: 18.04 or later versions
ROS: Melodic or later versions. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2 **OpenCV**
OpenCV: both version 3/4 are avariable

## 2. Build
Clone the project to '~/catkin_ws/src'
```
    cd ~/catkin_ws
    catkin_make
```

## 3. Example
### 3.1 extract VisionRTK data, convert to RobotGVINS format and write to rosbag
```
    cd ~/catkin_ws
    source devel/setup.bash
    rosrun datastreamio bagconvert_VisionRTK src/datastream_io/config/bagconvert_VisionRTK.yaml
```


## 4. Acknowledgements
We used [rtklib](https://github.com/tomojitakasu/RTKLIB/tree/rtklib_2.4.3) for GNSS format data conversion.

## 5. License
We are still working on improving the code reliability. For any technical issues, please contact Weihao Lei <whlei20000419@whu.edu.cn>.

## 6. History / Events
* **2025.03.01** - The *(Version 1.0.0)*:  This version can extract GNSS raw data from Vision-RTK2(Fixposition) and convert as RobotGVINS format to process. It can also extract GNSS raw data in RINEX 3.0x format and convert as RobotGVINS format. Thanks to the contribution of Lei weihao.
* **2025.07.04** - The *(Version 1.1.0)*:  This version can receive GNSS raw data from Vision-RTK2(Fixposition), convert as RobotGVINS format, and publish as ros messages to process. It can also receive GNSS raw data from RTCM3 TCP stream and publish as ros messages. Thanks to the contribution of Yan zhehua, Zhao yueqing, Lei weihao.
* **2025.07.20** - The *(Version 1.1.1)*:  This version integrated the codes, fixed some problems, and can read configuration files to process data conversion. Thanks to the contribution of Lei weihao.
* **2025.09.05** - The *(Version 1.1.2)*:  This version can convert convert GNSS raw data from RINEX 3.x format to GICI-LIB format. Thanks to the contribution of Lei weihao.
* **2025.09.28** - The *(Version 1.1.3)*:  This version used template to simplify funcionts and improve reliability. Thanks to the contribution of Lei weihao.
* **2025.11.18** - The *(Version 1.1.5)*:  This version can convert IMU raw data from IMR format to ROS standard format. Thanks to the contribution of Lei weihao.
* **2025.11.18** - The *(Version 1.1.6)*:  This version can extract GNDS raw data from ubx format file to RobotGVINS format. And it can extract and convert camera/imu data from Intel D457. Thanks to the contribution of Lei weihao.
