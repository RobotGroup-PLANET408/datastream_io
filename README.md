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
'''
    cd ~/catkin_ws
    catkin_make
'''

## 3. Example
### 3.1 data conversion
For an example, we convert the *RINEX 3.x format* GNSS raw data file to the *RobotGVINS GNSS observation data*, the steps are as follows:

    // step 1: extract GNSS observation data from rinex file
    // NOTE: The gnss observations are stored as customized format
    std::list<gnss_common::IPS_OBSDATA> ips_baseobs(0);
    dataio_common::Extract_GNSSObsData_RINEX3Format(baseobs_infilepath.c_str(), ips_baseobs);
    
    // step 2: convert GNSS observation data format to RobotGVINS format
    std::list<datastreamio::RobotGVINS_GNSSObs> robot_baseobs(0);
    dataio_common::Convert_GNSSObsData_IPS2RobotGVINS(ips_baseobs, robot_baseobs);
    
    // step 3: write GNSS obervation data to bag file
    // NOTE: The output gnss observation data should be RobotGVINS format
    dataio_common::Write_GNSSObsData_RobotGVINSFormat(bag_outfilepath.c_str(), dataio_common::RobotGVINS_gnssobs_topic_base, dataio_common::RobotGVINS_imu_topic, robot_baseobs, 2);


## 4. Acknowledgements
We used [rtklib](https://github.com/tomojitakasu/RTKLIB/tree/rtklib_2.4.3) for GNSS format data conversion.

## 5. License
We are still working on improving the code reliability. For any technical issues, please contact Weihao Lei <whlei20000419@whu.edu.cn>.