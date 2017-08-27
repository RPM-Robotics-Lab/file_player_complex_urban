#include <QMutexLocker>

#include "ROSThread.h"

using namespace std;

ROSThread::ROSThread(QObject *parent, QMutex *th_mutex) :
    QThread(parent), mutex_(th_mutex)
{

  processed_stamp_ = 0;
  play_rate_ = 1.0;

}

ROSThread::~ROSThread()
{

  data_stamp_thread_.active_ = false;
  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();

  altimter_thread_.active_ = false;
  altimter_thread_.cv_.notify_all();
  if(altimter_thread_.thread_.joinable()) altimter_thread_.thread_.join();

  encoder_thread_.active_ = false;
  encoder_thread_.cv_.notify_all();
  if(encoder_thread_.thread_.joinable()) encoder_thread_.thread_.join();

  fog_thread_.active_ = false;
  fog_thread_.cv_.notify_all();
  if(fog_thread_.thread_.joinable()) fog_thread_.thread_.join();

  gps_thread_.active_ = false;
  gps_thread_.cv_.notify_all();
  if(gps_thread_.thread_.joinable()) gps_thread_.thread_.join();

  vrs_thread_.active_ = false;
  vrs_thread_.cv_.notify_all();
  if(vrs_thread_.thread_.joinable()) vrs_thread_.thread_.join();

  velodyne_left_thread_.active_ = false;
  velodyne_left_thread_.cv_.notify_all();
  if(velodyne_left_thread_.thread_.joinable()) velodyne_left_thread_.thread_.join();

  velodyne_right_thread_.active_ = false;
  velodyne_right_thread_.cv_.notify_all();
  if(velodyne_right_thread_.thread_.joinable()) velodyne_right_thread_.thread_.join();

  sick_back_thread_.active_ = false;
  sick_back_thread_.cv_.notify_all();
  if(sick_back_thread_.thread_.joinable()) sick_back_thread_.thread_.join();

  sick_middle_thread_.active_ = false;
  sick_middle_thread_.cv_.notify_all();
  if(sick_middle_thread_.thread_.joinable()) sick_middle_thread_.thread_.join();
}

void ROSThread::ros_initialize(ros::NodeHandle &n)
{
  nh_ = n;

  pre_timer_stamp_ = ros::Time::now().toNSec();
  timer_ = nh_.createTimer(ros::Duration(0.0001), boost::bind(&ROSThread::TimerCallback, this, _1));

  altimeter_pub_ = nh_.advertise<irp_sen_msgs::altimeter>("/altimeter_data", 1000);
  encoder_pub_ = nh_.advertise<irp_sen_msgs::encoder>("/encoder_count", 1000);
  fog_pub_ = nh_.advertise<irp_sen_msgs::fog_3axis>("/dsp1760_data", 1000);
  gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);
  vrs_pub_ = nh_.advertise<irp_sen_msgs::vrs>("/vrs_gps_data", 1000);
  imu_pub_ = nh_.advertise<irp_sen_msgs::imu>("/xsens_imu_data", 1000);
  velodyne_left_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ns2/velodyne_points", 1000);
  velodyne_right_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ns1/velodyne_points", 1000);
  sick_back_pub_ = nh_.advertise<irp_sen_msgs::LaserScanArray>("/lms511_back/scan", 1000);
  sick_middle_pub_ = nh_.advertise<irp_sen_msgs::LaserScanArray>("/lms511_middle/scan", 1000);
}

void ROSThread::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

void ROSThread::Ready()
{
  //check path is right or not

  ifstream f((data_folder_path_+"/sensor_data/data_stamp.csv").c_str());
  if(!f.good()){
     cout << "Please check file path. Input path is wrong" << endl;
     return;
  }

  //Read CSV file and make map
  FILE *fp;
  int64_t stamp;
  //data stamp data load

  fp = fopen((data_folder_path_+"/sensor_data/data_stamp.csv").c_str(),"r");
  char data_name[50];
  while(fscanf(fp,"%ld,%s\n",&stamp,data_name) == 2){
    data_stamp_[stamp] = data_name;
  }
  cout << "Stamp data are loaded" << endl;
  fclose(fp);

  initial_data_stamp_ = data_stamp_.begin()->first - 1;

  //Read altimeter data
  fp = fopen((data_folder_path_+"/sensor_data/altimeter.csv").c_str(),"r");
  double altimeter_value;
  irp_sen_msgs::altimeter altimeter_data;
  while(fscanf(fp,"%ld,%lf\n",&stamp,&altimeter_value) == 2){
    altimeter_data.header.stamp.fromNSec(stamp);
    altimeter_data.header.frame_id = "altimeter";
    altimeter_data.data = altimeter_value;
    altimeter_data_[stamp] = altimeter_data;
  }
  cout << "Altimeter data are loaded" << endl;
  fclose(fp);

  //Read encoder data
  fp = fopen((data_folder_path_+"/sensor_data/encoder.csv").c_str(),"r");
  int64_t left_count, right_count;
  irp_sen_msgs::encoder encoder_data;
  while(fscanf(fp,"%ld,%ld,%ld\n",&stamp,&left_count,&right_count) == 3){
    encoder_data.header.stamp.fromNSec(stamp);
    encoder_data.header.frame_id = "encoder";
    encoder_data.left_count = left_count;
    encoder_data.right_count = right_count;
    encoder_data_[stamp] = encoder_data;
  }
  cout << "Encoder data are loaded" << endl;
  fclose(fp);

  //Read fog data
  fp = fopen((data_folder_path_+"/sensor_data/fog.csv").c_str(),"r");
  float d_roll, d_pitch, d_yaw;
  irp_sen_msgs::fog_3axis fog_data;
  while(fscanf(fp,"%ld,%f,%f,%f\n",&stamp,&d_roll,&d_pitch,&d_yaw) == 4){
    fog_data.header.stamp.fromNSec(stamp);
    fog_data.header.frame_id = "dsp1760";
    fog_data.d_roll = d_roll;
    fog_data.d_pitch = d_pitch;
    fog_data.d_yaw = d_yaw;
    fog_data_[stamp] = fog_data;
  }
  cout << "Fog data are loaded" << endl;
  fclose(fp);

  //Read gps data
  fp = fopen((data_folder_path_+"/sensor_data/gps.csv").c_str(),"r");
  double latitude, longitude, altitude;
  double cov[9];
  sensor_msgs::NavSatFix gps_data;
  while(fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&stamp,&latitude,&longitude,&altitude,&cov[0],&cov[1],&cov[2],&cov[3],&cov[4],&cov[5],&cov[6],&cov[7],&cov[8]) == 13){
    gps_data.header.stamp.fromNSec(stamp);
    gps_data.header.frame_id = "gps";
    gps_data.latitude = latitude;
    gps_data.longitude = longitude;
    gps_data.altitude = altitude;
    for(int i = 0 ; i < 9 ; i ++) gps_data.position_covariance[i] = cov[i];
    gps_data_[stamp] = gps_data;
  }
  cout << "Gps data are loaded" << endl;
  fclose(fp);

  //Read gps data
  fp = fopen((data_folder_path_+"/sensor_data/vrs_gps.csv").c_str(),"r");
  double x_coordinate, y_coordinate, horizental_precision, lat_std, lon_std, altitude_std, heading_magnet, speed_knot, speed_km;
  int fix_state, number_of_sat, heading_valid;
  char GNVTG_mode;
  irp_sen_msgs::vrs vrs_data;;
  while(fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%d,%d,%lf,%lf,%lf,%lf,%d,%lf,%lf,%lf,%c\n",&stamp,&latitude,&longitude,&x_coordinate,
               &y_coordinate,&altitude,&fix_state,&number_of_sat,&horizental_precision,&lat_std,&lon_std,&altitude_std,
               &heading_valid,&heading_magnet,&speed_knot,&speed_km,&GNVTG_mode) == 17){
    vrs_data.header.stamp.fromNSec(stamp);
    vrs_data.header.frame_id = "vrs_gps";
    vrs_data.latitude = latitude;
    vrs_data.longitude = longitude;
    vrs_data.x_coordinate = x_coordinate;
    vrs_data.y_coordinate = y_coordinate;
    vrs_data.altitude = altitude;
    vrs_data.fix_state = fix_state;
    if(fix_state == 1) vrs_data.fix_state_str = "normal";
    if(fix_state == 4) vrs_data.fix_state_str = "fix";
    if(fix_state == 5) vrs_data.fix_state_str = "float";
    vrs_data.number_of_sat = number_of_sat;
    vrs_data.horizental_precision = horizental_precision;
    vrs_data.lat_std = lat_std;
    vrs_data.lon_std = lon_std;
    vrs_data.altitude_std = altitude_std;
    vrs_data.heading_valid = heading_valid;
    vrs_data.heading_magnet = heading_magnet;
    vrs_data.speed_knot = speed_knot;
    vrs_data.speed_km = speed_km;
    vrs_data.GNVTG_mode = GNVTG_mode;
    vrs_data_[stamp] = vrs_data;
  }
  cout << "Vrs gps data are loaded" << endl;
  fclose(fp);


  //Read IMU data
  fp = fopen((data_folder_path_+"/sensor_data/xsens_imu.csv").c_str(),"r");
  double q_x,q_y,q_z,q_w,x,y,z;
  irp_sen_msgs::imu imu_data;;
  while(fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&stamp,&q_x,&q_y,&q_z,&q_w,&x,&y,&z) == 8){
    imu_data.header.stamp.fromNSec(stamp);
    imu_data.header.frame_id = "xsens_imu";
    imu_data.quaternion_data.x = q_x;
    imu_data.quaternion_data.y = q_y;
    imu_data.quaternion_data.z = q_z;
    imu_data.quaternion_data.w = q_w;
    imu_data.eular_data.x = x;
    imu_data.eular_data.y = y;
    imu_data.eular_data.z = z;
    imu_data_[stamp] = imu_data;
  }
  cout << "IMU data are loaded" << endl;
  fclose(fp);

  GetDirList(data_folder_path_ + "/sensor_data/VLP_left",velodyne_left_file_list_);
  GetDirList(data_folder_path_ + "/sensor_data/VLP_right",velodyne_right_file_list_);
  GetDirList(data_folder_path_ + "/sensor_data/SICK_back",sick_back_file_list_);
  GetDirList(data_folder_path_ + "/sensor_data/SICK_middle",sick_middle_file_list_);

  data_stamp_thread_.thread_ = std::thread(&ROSThread::DataStampThread,this);
  altimter_thread_.thread_ = std::thread(&ROSThread::AltimeterThread,this);
  encoder_thread_.thread_ = std::thread(&ROSThread::EncoderThread,this);
  fog_thread_.thread_ = std::thread(&ROSThread::FogThread,this);
  gps_thread_.thread_ = std::thread(&ROSThread::GpsThread,this);
  vrs_thread_.thread_ = std::thread(&ROSThread::VrsThread,this);
  imu_thread_.thread_ = std::thread(&ROSThread::ImuThread,this);

  velodyne_left_thread_.thread_ = std::thread(&ROSThread::VelodyneLeftThread,this);
  velodyne_right_thread_.thread_ = std::thread(&ROSThread::VelodyneRightThread,this);
  sick_back_thread_.thread_ = std::thread(&ROSThread::SickBackThread,this);
  sick_middle_thread_.thread_ = std::thread(&ROSThread::SickMiddleThread,this);
}

void ROSThread::DataStampThread()
{
  for(auto iter = data_stamp_.begin() ; iter != data_stamp_.end() ; iter ++){
    auto stamp = iter->first;

    while((stamp > (initial_data_stamp_+processed_stamp_))&&(data_stamp_thread_.active_ == true)){
      if(processed_stamp_ == 0){
          iter = data_stamp_.begin();
          stamp = iter->first;
      }
      usleep(1);
    }
    if(data_stamp_thread_.active_ == false) return;
    if(iter->second.compare("altimeter") == 0){
      altimter_thread_.push(stamp);
      altimter_thread_.cv_.notify_all();
    }else if(iter->second.compare("encoder") == 0){
      encoder_thread_.push(stamp);
      encoder_thread_.cv_.notify_all();
    }else if(iter->second.compare("fog") == 0){
      fog_thread_.push(stamp);
      fog_thread_.cv_.notify_all();
    }else if(iter->second.compare("gps") == 0){
      gps_thread_.push(stamp);
      gps_thread_.cv_.notify_all();
    }else if(iter->second.compare("vrs") == 0){
      vrs_thread_.push(stamp);
      vrs_thread_.cv_.notify_all();
    }else if(iter->second.compare("imu") == 0){
      imu_thread_.push(stamp);
      imu_thread_.cv_.notify_all();
    }else if(iter->second.compare("velodyne_left") == 0){
        velodyne_left_thread_.push(stamp);
        velodyne_left_thread_.cv_.notify_all();
    }else if(iter->second.compare("velodyne_right") == 0){
        velodyne_right_thread_.push(stamp);
        velodyne_right_thread_.cv_.notify_all();
    }else if(iter->second.compare("sick_back") == 0){
        sick_back_thread_.push(stamp);
        sick_back_thread_.cv_.notify_all();
    }else if(iter->second.compare("sick_middle") == 0){
        sick_middle_thread_.push(stamp);
        sick_middle_thread_.cv_.notify_all();
    }
    emit StampShow(stamp);
  }
}

void ROSThread::AltimeterThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(altimter_thread_.mutex_);
    altimter_thread_.cv_.wait(ul);
    if(altimter_thread_.active_ == false) return;
    ul.unlock();

    while(!altimter_thread_.data_queue_.empty()){
      auto data = altimter_thread_.pop();
      //process
      altimeter_pub_.publish(altimeter_data_[data]);

    }
  }
}

void ROSThread::EncoderThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(encoder_thread_.mutex_);
    encoder_thread_.cv_.wait(ul);
    if(encoder_thread_.active_ == false) return;
    ul.unlock();

    while(!encoder_thread_.data_queue_.empty()){
      auto data = encoder_thread_.pop();
      //process
      encoder_pub_.publish(encoder_data_[data]);

    }
  }
}

void ROSThread::FogThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(fog_thread_.mutex_);
    fog_thread_.cv_.wait(ul);
    if(fog_thread_.active_ == false) return;
    ul.unlock();

    while(!fog_thread_.data_queue_.empty()){
      auto data = fog_thread_.pop();
      //process
      fog_pub_.publish(fog_data_[data]);

    }
  }
}

void ROSThread::GpsThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(gps_thread_.mutex_);
    gps_thread_.cv_.wait(ul);
    if(gps_thread_.active_ == false) return;
    ul.unlock();

    while(!gps_thread_.data_queue_.empty()){
      auto data = gps_thread_.pop();
      //process
      gps_pub_.publish(gps_data_[data]);

    }
  }
}

void ROSThread::VrsThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(vrs_thread_.mutex_);
    vrs_thread_.cv_.wait(ul);
    if(vrs_thread_.active_ == false) return;
    ul.unlock();

    while(!vrs_thread_.data_queue_.empty()){
      auto data = vrs_thread_.pop();
      //process
      vrs_pub_.publish(vrs_data_[data]);

    }
  }
}

void ROSThread::ImuThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(imu_thread_.mutex_);
    imu_thread_.cv_.wait(ul);
    if(imu_thread_.active_ == false) return;
    ul.unlock();

    while(!imu_thread_.data_queue_.empty()){
      auto data = imu_thread_.pop();
      //process
      imu_pub_.publish(imu_data_[data]);

    }
  }
}

void ROSThread::TimerCallback(const ros::TimerEvent&)
{
    if(play_flag_ == true && pause_flag_ == false){
      processed_stamp_ += static_cast<int64_t>(static_cast<double>(ros::Time::now().toNSec() - pre_timer_stamp_) * play_rate_);
    }
    pre_timer_stamp_ = ros::Time::now().toNSec();

    if(play_flag_ == false){
      processed_stamp_ = 0; //reset
    }
}
void ROSThread::VelodyneLeftThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(velodyne_left_thread_.mutex_);
    velodyne_left_thread_.cv_.wait(ul);
    if(velodyne_left_thread_.active_ == false) return;
    ul.unlock();

    while(!velodyne_left_thread_.data_queue_.empty()){
      auto data = velodyne_left_thread_.pop();

      //publish data
      if(to_string(data) + ".bin" == velodyne_left_next_.first){
        //publish
        velodyne_left_next_.second.header.stamp.fromNSec(data);
        velodyne_left_next_.second.header.frame_id = "left_velodyne";
        velodyne_left_pub_.publish(velodyne_left_next_.second);

      }else{
        cout << "Re-load left velodyne from path" << endl;
        //load current data
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.clear();
        sensor_msgs::PointCloud2 publish_cloud;
        string current_file_name = data_folder_path_ + "/sensor_data/VLP_left" +"/"+ to_string(data) + ".bin";

        ifstream file;
        file.open(current_file_name, ios::in|ios::binary);
        while(!file.eof()){
            pcl::PointXYZI point;
            file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
            cloud.points.push_back (point);
        }
        file.close();

        pcl::toROSMsg(cloud, publish_cloud);
        publish_cloud.header.stamp.fromNSec(data);
        publish_cloud.header.frame_id = "left_velodyne";
        velodyne_left_pub_.publish(publish_cloud);
      }

      //load next data
      pcl::PointCloud<pcl::PointXYZI> cloud;
      cloud.clear();
      sensor_msgs::PointCloud2 publish_cloud;
      int current_file_index = find(velodyne_left_file_list_.begin(),velodyne_left_file_list_.end(),to_string(data)+".bin") - velodyne_left_file_list_.begin();
      string next_file_name = data_folder_path_ + "/sensor_data/VLP_left" +"/"+ velodyne_left_file_list_[current_file_index+1];

      ifstream file;
      file.open(next_file_name, ios::in|ios::binary);
      while(!file.eof()){
          pcl::PointXYZI point;
          file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
          cloud.points.push_back (point);
      }
      file.close();
      pcl::toROSMsg(cloud, publish_cloud);
      velodyne_left_next_ = make_pair(velodyne_left_file_list_[current_file_index+1], publish_cloud);
    }
  }
}
void ROSThread::VelodyneRightThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(velodyne_right_thread_.mutex_);
    velodyne_right_thread_.cv_.wait(ul);
    if(velodyne_right_thread_.active_ == false) return;
    ul.unlock();

    while(!velodyne_right_thread_.data_queue_.empty()){
      auto data = velodyne_right_thread_.pop();
      //process

      //publish data
      if(to_string(data) + ".bin" == velodyne_right_next_.first){
        //publish
        velodyne_right_next_.second.header.stamp.fromNSec(data);
        velodyne_right_next_.second.header.frame_id = "right_velodyne";
        velodyne_right_pub_.publish(velodyne_right_next_.second);

      }else{
        cout << "Re-load right velodyne from path" << endl;
        //load current data
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.clear();
        sensor_msgs::PointCloud2 publish_cloud;
        string current_file_name = data_folder_path_ + "/sensor_data/VLP_right" +"/"+ to_string(data) + ".bin";

        ifstream file;
        file.open(current_file_name, ios::in|ios::binary);
        while(!file.eof()){
            pcl::PointXYZI point;
            file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
            cloud.points.push_back (point);
        }
        file.close();

        pcl::toROSMsg(cloud, publish_cloud);
        publish_cloud.header.stamp.fromNSec(data);
        publish_cloud.header.frame_id = "right_velodyne";
        velodyne_right_pub_.publish(publish_cloud);
      }

      //load next data
      pcl::PointCloud<pcl::PointXYZI> cloud;
      cloud.clear();
      sensor_msgs::PointCloud2 publish_cloud;
      int current_file_index = find(velodyne_right_file_list_.begin(),velodyne_right_file_list_.end(),to_string(data)+".bin") - velodyne_right_file_list_.begin();
      string next_file_name = data_folder_path_ + "/sensor_data/VLP_right" +"/"+ velodyne_right_file_list_[current_file_index+1];

      ifstream file;
      file.open(next_file_name, ios::in|ios::binary);
      while(!file.eof()){
          pcl::PointXYZI point;
          file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
          cloud.points.push_back (point);
      }
      file.close();
      pcl::toROSMsg(cloud, publish_cloud);
      velodyne_right_next_ = make_pair(velodyne_right_file_list_[current_file_index+1], publish_cloud);
    }
  }
}
void ROSThread::SickBackThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(sick_back_thread_.mutex_);
    sick_back_thread_.cv_.wait(ul);
    if(sick_back_thread_.active_ == false) return;
    ul.unlock();

    while(!sick_back_thread_.data_queue_.empty()){
      auto data = sick_back_thread_.pop();
      //process

      //publish data
      if(to_string(data) + ".bin" == sick_back_next_.first){
        //publish
        sick_back_next_.second.header.stamp.fromNSec(data);
        sick_back_next_.second.header.frame_id = "back_sick";
        sick_back_pub_.publish(sick_back_next_.second);

      }else{
        cout << "Re-load back sick from path" << endl;
        //load current data
        irp_sen_msgs::LaserScanArray publish_data;
        sensor_msgs::LaserScan scan_data;
        string current_file_name = data_folder_path_ + "/sensor_data/SICK_back" +"/"+ to_string(data)+".bin";

        ifstream file;
        file.open(current_file_name, ios::in|ios::binary);
        while(!file.eof()){
            float range;
            float intensity;
            file.read(reinterpret_cast<char *>(&range), sizeof(float));
            file.read(reinterpret_cast<char *>(&intensity), sizeof(float));
            scan_data.ranges.push_back(range);
            scan_data.intensities.push_back((intensity));
        }
        file.close();
        scan_data.header.stamp.fromNSec(data);
        scan_data.header.frame_id = "back_sick";
        scan_data.angle_min = -1.65806281567;
        scan_data.angle_max = -1.65806281567;
        scan_data.angle_increment = 0.0116355288774;
        scan_data.time_increment = 0.0;
        scan_data.range_min = 0.0;
        scan_data.range_max = 81.0;
        publish_data.LaserScans.push_back(scan_data);
        publish_data.size = publish_data.LaserScans.size();

        publish_data.header.stamp.fromNSec(data);
        publish_data.header.frame_id = "back_sick";
        sick_back_pub_.publish(publish_data);
      }

      //load next data
      irp_sen_msgs::LaserScanArray publish_data;
      sensor_msgs::LaserScan scan_data;
      int current_file_index = find(sick_back_file_list_.begin(),sick_back_file_list_.end(),to_string(data)+".bin") - sick_back_file_list_.begin();
      string next_file_name = data_folder_path_ + "/sensor_data/SICK_back" +"/"+ sick_back_file_list_[current_file_index+1];

      ifstream file;
      file.open(next_file_name, ios::in|ios::binary);
      while(!file.eof()){
          float range;
          float intensity;
          file.read(reinterpret_cast <char *>(&range), sizeof(float));
          file.read(reinterpret_cast <char *>(&intensity), sizeof(float));
          scan_data.ranges.push_back(range);
          scan_data.intensities.push_back(intensity);
      }
      file.close();
      char* pEnd;
      scan_data.header.stamp.fromNSec(strtoll(sick_back_file_list_[current_file_index+1].substr(0,19).c_str(),&pEnd,10));
      scan_data.header.frame_id = "back_sick";
      scan_data.angle_min = -1.65806281567;
      scan_data.angle_max = -1.65806281567;
      scan_data.angle_increment = 0.0116355288774;
      scan_data.time_increment = 0.0;
      scan_data.range_min = 0.0;
      scan_data.range_max = 81.0;
      publish_data.LaserScans.push_back(scan_data);
      publish_data.size = publish_data.LaserScans.size();
      sick_back_next_ = make_pair(sick_back_file_list_[current_file_index+1], publish_data);
    }
  }
}
void ROSThread::SickMiddleThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(sick_middle_thread_.mutex_);
    sick_middle_thread_.cv_.wait(ul);
    if(sick_middle_thread_.active_ == false) return;
    ul.unlock();

    while(!sick_middle_thread_.data_queue_.empty()){
      auto data = sick_middle_thread_.pop();
      //process
      //publish data
      if(to_string(data) + ".bin" == sick_middle_next_.first){
        //publish
        sick_middle_next_.second.header.stamp.fromNSec(data);
        sick_middle_next_.second.header.frame_id = "middle_sick";
        sick_middle_pub_.publish(sick_middle_next_.second);

      }else{
        cout << "Re-load middle sick from path" << endl;
        //load current data
        irp_sen_msgs::LaserScanArray publish_data;
        sensor_msgs::LaserScan scan_data;
        string current_file_name = data_folder_path_ + "/sensor_data/SICK_middle" +"/"+ to_string(data)+".bin";

        ifstream file;
        file.open(current_file_name, ios::in|ios::binary);
        while(!file.eof()){
            float range;
            float intensity;
            file.read(reinterpret_cast <char *>(&range), sizeof(range));
            file.read(reinterpret_cast <char *>(&intensity), sizeof(intensity));
            scan_data.ranges.push_back(range);
            scan_data.intensities.push_back((intensity));
        }
        file.close();
        scan_data.header.stamp.fromNSec(data);
        scan_data.header.frame_id = "middle_sick";
        scan_data.angle_min = -1.65806281567;
        scan_data.angle_max = -1.65806281567;
        scan_data.angle_increment = 0.0116355288774;
        scan_data.time_increment = 0.0;
        scan_data.range_min = 0.0;
        scan_data.range_max = 81.0;
        publish_data.LaserScans.push_back(scan_data);
        publish_data.size = publish_data.LaserScans.size();

        publish_data.header.stamp.fromNSec(data);
        publish_data.header.frame_id = "middle_sick";
        sick_middle_pub_.publish(publish_data);
      }

      //load next data
      irp_sen_msgs::LaserScanArray publish_data;
      sensor_msgs::LaserScan scan_data;
      int current_file_index = find(sick_middle_file_list_.begin(),sick_middle_file_list_.end(),to_string(data)+".bin") - sick_middle_file_list_.begin();
      string next_file_name = data_folder_path_ + "/sensor_data/SICK_middle" +"/"+ sick_middle_file_list_[current_file_index+1];

      ifstream file;
      file.open(next_file_name, ios::in|ios::binary);
      while(!file.eof()){
          float range;
          float intensity;
          file.read(reinterpret_cast <char *>(&range), sizeof(range));
          file.read(reinterpret_cast <char *>(&intensity), sizeof(intensity));
          scan_data.ranges.push_back(range);
          scan_data.intensities.push_back((intensity));
      }
      file.close();
      char* pEnd;
      scan_data.header.stamp.fromNSec(strtoll(sick_middle_file_list_[current_file_index+1].substr(0,19).c_str(),&pEnd,10));
      scan_data.header.frame_id = "middle_sick";
      scan_data.angle_min = -1.65806281567;
      scan_data.angle_max = -1.65806281567;
      scan_data.angle_increment = 0.0116355288774;
      scan_data.time_increment = 0.0;
      scan_data.range_min = 0.0;
      scan_data.range_max = 81.0;
      publish_data.LaserScans.push_back(scan_data);
      publish_data.size = publish_data.LaserScans.size();
      sick_middle_next_ = make_pair(sick_middle_file_list_[current_file_index+1], publish_data);
    }
  }
}

int ROSThread::GetDirList(string dir, vector<string> &files)
{

  vector<string> tmp_files;
  struct dirent **namelist;
  int n;
  n = scandir(dir.c_str(),&namelist, 0 , alphasort);
  if (n < 0)
      perror("scandir");
  else {
      while (n--) {
      if(string(namelist[n]->d_name) != "." && string(namelist[n]->d_name) != ".."){
        tmp_files.push_back(string(namelist[n]->d_name));
      }
      free(namelist[n]);
      }
      free(namelist);
  }

  for(auto iter = tmp_files.rbegin() ; iter!= tmp_files.rend() ; iter++){
    files.push_back(*iter);
  }
    return 0;
}

