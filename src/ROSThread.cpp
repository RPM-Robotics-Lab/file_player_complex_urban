#include <QMutexLocker>

#include "ROSThread.h"

using namespace std;

ROSThread::ROSThread(QObject *parent, QMutex *th_mutex) :
    QThread(parent), mutex_(th_mutex)
{

}
ROSThread::~ROSThread()
{


}

void ROSThread::ros_initialize(ros::NodeHandle &n)
{
  nh_ = n;

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

}
