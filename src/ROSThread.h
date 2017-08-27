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
#include <sensor_msgs/LaserScan.h>
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

#include "file_player/color.h"
#include "rosbag/bag.h"
#include <ros/transport_hints.h>
#include "file_player/datathread.h"
#include <sys/types.h>

#include <algorithm>
#include <iterator>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


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
    double play_rate_;
    string data_folder_path_;
    void Ready();

signals:
    void StampShow(quint64 stamp);


private:

    ros::Publisher altimeter_pub_;
    ros::Publisher encoder_pub_;
    ros::Publisher fog_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher vrs_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher velodyne_left_pub_;
    ros::Publisher velodyne_right_pub_;
    ros::Publisher sick_back_pub_;
    ros::Publisher sick_middle_pub_;

    map<int64_t, string>                    data_stamp_;
    map<int64_t, irp_sen_msgs::altimeter>   altimeter_data_;
    map<int64_t, irp_sen_msgs::encoder>     encoder_data_;
    map<int64_t, irp_sen_msgs::fog_3axis>   fog_data_;
    map<int64_t, sensor_msgs::NavSatFix>    gps_data_;
    map<int64_t, irp_sen_msgs::vrs>         vrs_data_;
    map<int64_t, irp_sen_msgs::imu>         imu_data_;

    DataThread<int64_t> data_stamp_thread_;
    DataThread<int64_t> altimter_thread_;
    DataThread<int64_t> encoder_thread_;
    DataThread<int64_t> fog_thread_;
    DataThread<int64_t> gps_thread_;
    DataThread<int64_t> vrs_thread_;
    DataThread<int64_t> imu_thread_;
    DataThread<int64_t> omni_thread_;
    DataThread<int64_t> stereo_thread_;
    DataThread<int64_t> velodyne_left_thread_;
    DataThread<int64_t> velodyne_right_thread_;
    DataThread<int64_t> sick_back_thread_;
    DataThread<int64_t> sick_middle_thread_;

    void DataStampThread();
    void AltimeterThread();
    void EncoderThread();
    void FogThread();
    void GpsThread();
    void VrsThread();
    void ImuThread();
    void VelodyneLeftThread();
    void VelodyneRightThread();
    void SickBackThread();
    void SickMiddleThread();

    vector<string> velodyne_left_file_list_;
    vector<string> velodyne_right_file_list_;
    vector<string> sick_back_file_list_;
    vector<string> sick_middle_file_list_;


    int64_t initial_data_stamp_;


    ros::Timer timer_;
    void TimerCallback(const ros::TimerEvent&);
    int64_t processed_stamp_;
    int64_t pre_timer_stamp_;

    pair<string,sensor_msgs::PointCloud2> velodyne_left_next_;
    pair<string,sensor_msgs::PointCloud2> velodyne_right_next_;
    pair<string,irp_sen_msgs::LaserScanArray> sick_back_next_;
    pair<string,irp_sen_msgs::LaserScanArray> sick_middle_next_;

    int GetDirList(string dir, vector<string> &files);


public slots:

};

#endif // VIEWER_LCM_H
