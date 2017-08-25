#ifndef VIEWER_ROS_H
#define VIEWER_ROS_H

#include <fstream>
#include <iostream>
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QPixmap>
#include <QVector>
#include <QVector3D>
#include <QDateTime>
#include <QReadLocker>
#include <QPainter>
#include <QLabel>
#include <algorithm>
#include <ros/ros.h>
#include <ros/time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <irp_sen_msgs/vrs.h>
#include <irp_sen_msgs/altimeter.h>
#include <irp_sen_msgs/encoder.h>
#include <irp_sen_msgs/fog.h>
#include <irp_sen_msgs/imu.h>
#include <irp_sen_msgs/fog_3axis.h>
#include <irp_sen_msgs/LaserScanArray.h>
#include <velodyne_msgs/VelodyneScan.h>

#include <dynamic_reconfigure/server.h>
#include <file_player/dynamic_file_playerConfig.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <condition_variable>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "color.h"
#include "rosbag/bag.h"
#include <ros/transport_hints.h>

using namespace std;

class ROSThread : public QThread
{
    Q_OBJECT

public:
    explicit ROSThread(QObject *parent = 0, QMutex *th_mutex = 0);
    ~ROSThread();
    void ros_initialize(ros::NodeHandle &n);
    void run();
    QMutex *mutex_;
    ros::NodeHandle nh_;

    bool play_flag_;
    bool pause_flag_;
    string data_folder_path_;
    void Ready();

signals:


private:

    map<int64_t, string>                    data_stamp_;
    map<int64_t, irp_sen_msgs::altimeter>   altimeter_data_;
    map<int64_t, irp_sen_msgs::encoder>     encoder_data_;
    map<int64_t, irp_sen_msgs::fog_3axis>   fog_data_;
    map<int64_t, sensor_msgs::NavSatFix>    gps_data_;
    map<int64_t, irp_sen_msgs::vrs>         vrs_data_;
    map<int64_t, irp_sen_msgs::imu>         imu_data_;


public slots:

};

#endif // VIEWER_LCM_H
