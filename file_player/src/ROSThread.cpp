#include <QMutexLocker>

#include "ROSThread.h"

using namespace std;

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    uint32_t t;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (uint32_t, t, t)
)

ROSThread::ROSThread(QObject *parent, QMutex *th_mutex) :
    QThread(parent), mutex_(th_mutex)
{
  processed_stamp_ = 0;
  play_rate_ = 1.0;
  loop_flag_ = false;
  stop_skip_flag_ = true;

  search_bound_ = 10;
  // search_bound_ = 1000000;
  reset_process_stamp_flag_ = false;
  auto_start_flag_ = true;
  stamp_show_count_ = 0;
  imu_data_version_ = 0;
  prev_clock_stamp_ = 0;
}

ROSThread::~ROSThread()
{
  data_stamp_thread_.active_ = false;
  gps_thread_.active_ = false;
  //inspva_thread_.active_ = false;
  //inspvax_thread_.active_ = false;
  imu_thread_.active_ = false;
  velodyne_left_thread_.active_ = false;
  velodyne_right_thread_.active_ = false;
  livox_avia_thread_.active_ = false;
  livox_tele_thread_.active_ = false;
  ouster_thread_.active_ = false;

  usleep(100000);

  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();

  gps_thread_.cv_.notify_all();
  if(gps_thread_.thread_.joinable()) gps_thread_.thread_.join();
  
  /*inspva_thread_.cv_.notify_all();
  if(inspva_thread_.thread_.joinable()) inspva_thread_.thread_.join();

  inspvax_thread_.cv_.notify_all();
  if(inspvax_thread_.thread_.joinable()) inspvax_thread_.thread_.join();*/

  imu_thread_.cv_.notify_all();
  if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();

  velodyne_left_thread_.cv_.notify_all();
  if(velodyne_left_thread_.thread_.joinable()) velodyne_left_thread_.thread_.join();

  velodyne_right_thread_.cv_.notify_all();
  if(velodyne_right_thread_.thread_.joinable()) velodyne_right_thread_.thread_.join();

  livox_avia_thread_.cv_.notify_all();
  if(livox_avia_thread_.thread_.joinable()) livox_avia_thread_.thread_.join();

  livox_tele_thread_.cv_.notify_all();
  if(livox_tele_thread_.thread_.joinable()) livox_tele_thread_.thread_.join();

  ouster_thread_.cv_.notify_all();
  if(ouster_thread_.thread_.joinable()) ouster_thread_.thread_.join();
}

void ROSThread::ros_initialize(ros::NodeHandle &n)
{
  nh_ = n;

  pre_timer_stamp_ = ros::Time::now().toNSec();
  timer_ = nh_.createTimer(ros::Duration(0.0001), boost::bind(&ROSThread::TimerCallback, this, _1));

  start_sub_  = nh_.subscribe<std_msgs::Bool>("/file_player_start", 1, boost::bind(&ROSThread::FilePlayerStart, this, _1));
  stop_sub_    = nh_.subscribe<std_msgs::Bool>("/file_player_stop", 1, boost::bind(&ROSThread::FilePlayerStop, this, _1));

  //gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);
  //inspva_pub_ = nh_.advertise<novatel_gps_msgs::Inspva>("/inspva", 1000);
  //inspvax_pub_ = nh_.advertise<novatel_oem7_msgs::INSPVAX>("/inspvax", 1000);

  // imu_origin_pub_ = nh_.advertise<irp_sen_msgs::imu>("/imu/data", 1000);
  //imu_origin_pub_ = nh_.advertise<irp_sen_msgs::imu>("/xsens_imu_data", 1000);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/xsens_imu_data", 1000);
  //magnet_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/imu/mag", 1000);

  //velodyne_left_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ns3/velodyne_points", 1000);
  //velodyne_right_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ns2/velodyne_points", 1000);

  livox_avia_pub_ = nh_.advertise<livox_ros_driver::CustomMsg>("/livox/avia/points", 1000);
  livox_tele_pub_ = nh_.advertise<livox_ros_driver::CustomMsg>("/livox/tele/points", 1000);
  ouster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/os_cloud_node/points", 10000);


  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
}

void ROSThread::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

void ROSThread::Ready()
{
  data_stamp_thread_.active_ = false;
  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();
  /*gps_thread_.active_ = false;
  gps_thread_.cv_.notify_all();
  if(gps_thread_.thread_.joinable()) gps_thread_.thread_.join();
  inspva_thread_.active_ = false;
  inspva_thread_.cv_.notify_all();
  if(inspva_thread_.thread_.joinable()) inspva_thread_.thread_.join();
  inspvax_thread_.active_ = false;
  inspvax_thread_.cv_.notify_all();
  if(inspvax_thread_.thread_.joinable()) inspvax_thread_.thread_.join(); */
  imu_thread_.active_ = false;
  imu_thread_.cv_.notify_all(); 
  if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();
  /*velodyne_left_thread_.active_ = false;
  velodyne_left_thread_.cv_.notify_all();
  if(velodyne_left_thread_.thread_.joinable()) velodyne_left_thread_.thread_.join();
  velodyne_right_thread_.active_ = false;
  velodyne_right_thread_.cv_.notify_all();
  if(velodyne_right_thread_.thread_.joinable()) velodyne_right_thread_.thread_.join();*/
  livox_avia_thread_.active_ = false;
  livox_avia_thread_.cv_.notify_all();
  if(livox_avia_thread_.thread_.joinable()) livox_avia_thread_.thread_.join();
  livox_tele_thread_.active_ = false;
  livox_tele_thread_.cv_.notify_all();
  if(livox_tele_thread_.thread_.joinable()) livox_tele_thread_.thread_.join();
  ouster_thread_.active_ = false;
  ouster_thread_.cv_.notify_all();
  if(ouster_thread_.thread_.joinable()) ouster_thread_.thread_.join();

  //check path is right or not
  ifstream f((data_folder_path_+"/sensor_data/data_stamp.csv").c_str());
  if(!f.good()){
     cout << "Please check file path. Input path is wrong" << endl;
     return;
  }
  f.close();

  //Read CSV file and make map
  FILE *fp;
  int64_t stamp;
  //data stamp data load

  fp = fopen((data_folder_path_+"/sensor_data/data_stamp.csv").c_str(),"r");
  char data_name[50];
  data_stamp_.clear();
  while(fscanf(fp,"%ld,%s\n",&stamp,data_name) == 2){
//    data_stamp_[stamp] = data_name;
    data_stamp_.insert( multimap<int64_t, string>::value_type(stamp, data_name));
  }
  cout << "Stamp data are loaded" << endl;
  fclose(fp);

  initial_data_stamp_ = data_stamp_.begin()->first - 1;
  last_data_stamp_ = prev(data_stamp_.end(),1)->first - 1;

  //Read inspva data
  /*fp = fopen((data_folder_path_+"/sensor_data/inspva.csv").c_str(),"r");
  double latitude, longitude, altitude, altitude_orthometric;
  double height, north_velocity, east_velocity, up_velocity, roll, pitch, azimuth;
  // string status;
  char status[17];
  novatel_gps_msgs::Inspva inspva_data;
  inspva_data_.clear();
  while(fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%s",&stamp,&latitude,&longitude,&height,&north_velocity,&east_velocity,&up_velocity,&roll,&pitch,&azimuth,status) == 11){
  //17%19[^\n] %29[^\n]
    inspva_data.header.stamp.fromNSec(stamp);
    inspva_data.header.frame_id = "inspva";
    inspva_data.latitude = latitude;
    inspva_data.longitude = longitude;
    inspva_data.height = height;
    inspva_data.north_velocity = north_velocity;
    inspva_data.east_velocity = east_velocity;
    inspva_data.up_velocity = up_velocity;
    inspva_data.roll = roll;
    inspva_data.pitch = pitch;
    inspva_data.azimuth = azimuth;
    inspva_data.status = status;
    inspva_data_[stamp] = inspva_data;
  }
  cout << "Inspva data are loaded" << endl;
  fclose(fp);*/

  //Read inspvax data
  /*fp = fopen((data_folder_path_+"/sensor_data/inspvax.csv").c_str(),"r");
  double latitude, longitude, altitude, altitude_orthometric;
  double height, north_velocity, east_velocity, up_velocity, roll, pitch, azimuth;
  // string status;
  char status[17];
  novatel_oem7_msgs::INSPVAX inspvax_data;
  inspvax_data_.clear();
  while(fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%s",&stamp,&latitude,&longitude,&height,&north_velocity,&east_velocity,&up_velocity,&roll,&pitch,&azimuth,status) == 11){
  //17%19[^\n] %29[^\n]
    inspvax_data.header.stamp.fromNSec(stamp);
    inspvax_data.header.frame_id = "inspvax";
    inspvax_data.latitude = latitude;
    inspvax_data.longitude = longitude;
    inspvax_data.height = height;
    inspvax_data.north_velocity = north_velocity;
    inspvax_data.east_velocity = east_velocity;
    inspvax_data.up_velocity = up_velocity;
    inspvax_data.roll = roll;
    inspvax_data.pitch = pitch;
    inspvax_data.azimuth = azimuth;
    inspvax_data_[stamp] = inspvax_data;
  }
  cout << "Inspva data are loaded" << endl;
  fclose(fp);*/

  //Read IMU data
  fp = fopen((data_folder_path_+"/sensor_data/xsens_imu.csv").c_str(),"r");
  double q_x,q_y,q_z,q_w,x,y,z,g_x,g_y,g_z,a_x,a_y,a_z,m_x,m_y,m_z;
  irp_sen_msgs::imu imu_data_origin;
  sensor_msgs::Imu imu_data;
  sensor_msgs::MagneticField mag_data;
  imu_data_.clear();
  mag_data_.clear();

  while(1){
    int length = fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&stamp,&q_x,&q_y,&q_z,&q_w,&x,&y,&z,&g_x,&g_y,&g_z,&a_x,&a_y,&a_z,&m_x,&m_y,&m_z);
    if(length != 8 && length != 11 && length != 17) break;
    if(length == 8){
      imu_data.header.stamp.fromNSec(stamp);
      imu_data.header.frame_id = "imu_link";
      imu_data.orientation.x = q_x;
      imu_data.orientation.y = q_y;
      imu_data.orientation.z = q_z;
      imu_data.orientation.w = q_w;

      imu_data_[stamp] = imu_data;
      imu_data_version_ = 1;

      imu_data_origin.header.stamp.fromNSec(stamp);
      imu_data_origin.header.frame_id = "imu";
      imu_data_origin.quaternion_data.x = q_x;
      imu_data_origin.quaternion_data.y = q_y;
      imu_data_origin.quaternion_data.z = q_z;
      imu_data_origin.quaternion_data.w = q_w;
      imu_data_origin.eular_data.x = x;
      imu_data_origin.eular_data.y = y;
      imu_data_origin.eular_data.z = z;
      imu_data_origin_[stamp] = imu_data_origin;

    }
    else if(length == 11){
      imu_data.header.stamp.fromNSec(stamp);
      imu_data.header.frame_id = "imu_link";
      imu_data.orientation.x = q_x;
      imu_data.orientation.y = q_y;
      imu_data.orientation.z = q_z;
      imu_data.orientation.w = q_w;
      imu_data.angular_velocity.x = x;
      imu_data.angular_velocity.y = y;
      imu_data.angular_velocity.z = z;
      imu_data.linear_acceleration.x = g_x;
      imu_data.linear_acceleration.y = g_y;
      imu_data.linear_acceleration.z = g_z;
      imu_data_[stamp] = imu_data;
      imu_data_version_ = 1;

      imu_data_origin.header.stamp.fromNSec(stamp);
      imu_data_origin.header.frame_id = "imu";
      imu_data_origin.quaternion_data.x = q_x;
      imu_data_origin.quaternion_data.y = q_y;
      imu_data_origin.quaternion_data.z = q_z;
      imu_data_origin.quaternion_data.w = q_w;
      imu_data_origin.eular_data.x = x;
      imu_data_origin.eular_data.y = y;
      imu_data_origin.eular_data.z = z;
      imu_data_origin_[stamp] = imu_data_origin;
    }
    else if(length == 17){
      imu_data.header.stamp.fromNSec(stamp);
      imu_data.header.frame_id = "husky4/base_link";
      imu_data.orientation.x = q_x;
      imu_data.orientation.y = q_y;
      imu_data.orientation.z = q_z;
      imu_data.orientation.w = q_w;
      imu_data.angular_velocity.x = x;
      imu_data.angular_velocity.y = y;
      imu_data.angular_velocity.z = z;
      imu_data.linear_acceleration.x = g_x;
      imu_data.linear_acceleration.y = g_y;
      imu_data.linear_acceleration.z = g_z;

      imu_data.orientation_covariance[0] = 3;
      imu_data.orientation_covariance[4] = 3;
      imu_data.orientation_covariance[8] = 3;
      imu_data.angular_velocity_covariance[0] = 3;
      imu_data.angular_velocity_covariance[4] = 3;
      imu_data.angular_velocity_covariance[8] = 3;
      imu_data.linear_acceleration_covariance[0] = 3;
      imu_data.linear_acceleration_covariance[4] = 3;
      imu_data.linear_acceleration_covariance[8] = 3;


      imu_data_[stamp] = imu_data;
      mag_data.header.stamp.fromNSec(stamp);
      mag_data.header.frame_id = "husky4/base_link";
      mag_data.magnetic_field.x = m_x;
      mag_data.magnetic_field.y = m_y;
      mag_data.magnetic_field.z = m_z;
      mag_data_[stamp] = mag_data;
      imu_data_version_ = 2;


      imu_data_origin.header.stamp.fromNSec(stamp);
      imu_data_origin.header.frame_id = "husky4/base_link";
      imu_data_origin.quaternion_data.x = q_x;
      imu_data_origin.quaternion_data.y = q_y;
      imu_data_origin.quaternion_data.z = q_z;
      imu_data_origin.quaternion_data.w = q_w;
      imu_data_origin.eular_data.x = x;
      imu_data_origin.eular_data.y = y;
      imu_data_origin.eular_data.z = z;
      imu_data_origin.gyro_data.x = g_x;
      imu_data_origin.gyro_data.y = g_y;
      imu_data_origin.gyro_data.z = g_z;
      imu_data_origin.acceleration_data.x = a_x;
      imu_data_origin.acceleration_data.y = a_y;
      imu_data_origin.acceleration_data.z = a_z;
      imu_data_origin.magneticfield_data.x = m_x;
      imu_data_origin.magneticfield_data.y = m_y;
      imu_data_origin.magneticfield_data.z = m_z;
      imu_data_origin_[stamp] = imu_data_origin;

    }
  }
  std::cout << imu_data_.size() << std::endl;
  cout << "IMU data are loaded" << endl;
  fclose(fp);
  velodyne_left_file_list_.clear();
  velodyne_right_file_list_.clear();
  livox_avia_file_list_.clear();
  livox_tele_file_list_.clear();
  ouster_file_list_.clear();


  GetDirList(data_folder_path_ + "/sensor_data/ouster",ouster_file_list_);
  GetDirList(data_folder_path_ + "/sensor_data/Livox_avia",livox_avia_file_list_);
  GetDirList(data_folder_path_ + "/sensor_data/Livox_tele",livox_tele_file_list_);

  data_stamp_thread_.active_ = true;
  imu_thread_.active_ = true;
  livox_avia_thread_.active_ = true;
  livox_tele_thread_.active_ = true;
  ouster_thread_.active_ = true;
  data_stamp_thread_.thread_ = std::thread(&ROSThread::DataStampThread,this);
 
  imu_thread_.thread_ = std::thread(&ROSThread::ImuThread,this);
 
  livox_avia_thread_.thread_ = std::thread(&ROSThread::LivoxAviaThread,this);
  livox_tele_thread_.thread_ = std::thread(&ROSThread::LivoxTeleThread,this);
  ouster_thread_.thread_ = std::thread(&ROSThread::OusterThread,this);

}

void ROSThread::DataStampThread()
{
  auto stop_region_iter = stop_period_.begin();

  for(auto iter = data_stamp_.begin() ; iter != data_stamp_.end() ; iter ++){
    auto stamp = iter->first;

    while((stamp > (initial_data_stamp_+processed_stamp_))&&(data_stamp_thread_.active_ == true)){
      if(processed_stamp_ == 0){
          iter = data_stamp_.begin();
          stop_region_iter = stop_period_.begin();
          stamp = iter->first;
      }
      usleep(1);
      if(reset_process_stamp_flag_ == true) break;
      //wait for data publish
    }

    if(reset_process_stamp_flag_ == true){
      auto target_stamp = processed_stamp_ + initial_data_stamp_;
      //set iter
      iter = data_stamp_.lower_bound(target_stamp);
      iter = prev(iter,1);
      //set stop region order
      auto new_stamp = iter->first;
      stop_region_iter = stop_period_.upper_bound(new_stamp);

      reset_process_stamp_flag_ = false;
      continue;
    }


    //check whether stop region or not
    if(stamp == stop_region_iter->first){
      if(stop_skip_flag_ == true){
        cout << "Skip stop section!!" << endl;
        iter = data_stamp_.find(stop_region_iter->second);  //find stop region end
        iter = prev(iter,1);
        processed_stamp_ = stop_region_iter->second - initial_data_stamp_;
      }
      stop_region_iter++;
      if(stop_skip_flag_ == true){
        continue;
      }
    }

    if(data_stamp_thread_.active_ == false) return;
    if(iter->second.compare("imu") == 0){
      imu_thread_.push(stamp);
      imu_thread_.cv_.notify_all();
    }
    else if(iter->second.compare("gps") == 0){
      gps_thread_.push(stamp);
      gps_thread_.cv_.notify_all();
    }
    else if(iter->second.compare("livox_avia") == 0){
        livox_avia_thread_.push(stamp);
        livox_avia_thread_.cv_.notify_all();
    }else if(iter->second.compare("livox_tele") == 0){
        livox_tele_thread_.push(stamp);
        livox_tele_thread_.cv_.notify_all();
    }else if(iter->second.compare("ouster") == 0){
        ouster_thread_.push(stamp);
        ouster_thread_.cv_.notify_all();
    }
    stamp_show_count_++;
    if(stamp_show_count_ > 100){
      stamp_show_count_ = 0;
      emit StampShow(stamp);
    }

    if(prev_clock_stamp_ == 0 || (stamp - prev_clock_stamp_) > 10000000){
        rosgraph_msgs::Clock clock;
        clock.clock.fromNSec(stamp);
        clock_pub_.publish(clock);
        prev_clock_stamp_ = stamp;
    }

    if(loop_flag_ == true && iter == prev(data_stamp_.end(),1)){
        iter = data_stamp_.begin();
        stop_region_iter = stop_period_.begin();
        processed_stamp_ = 0;
    }
    if(loop_flag_ == false && iter == prev(data_stamp_.end(),1)){
        play_flag_ = false;
        while(!play_flag_){
            iter = data_stamp_.begin();
            stop_region_iter = stop_period_.begin();
            processed_stamp_ = 0;
            usleep(10000);
        }
    }
  }
  cout << "Data publish complete" << endl;

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
      if(gps_data_.find(data) != gps_data_.end()){
        gps_pub_.publish(gps_data_[data]);
      }

    }
    if(gps_thread_.active_ == false) return;
  }
}
/*void ROSThread::InspvaThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(inspva_thread_.mutex_);
    inspva_thread_.cv_.wait(ul);
    if(inspva_thread_.active_ == false) return;
    ul.unlock();

    while(!inspva_thread_.data_queue_.empty()){
      auto data = inspva_thread_.pop();
      //process
      if(inspva_data_.find(data) != inspva_data_.end()){
        inspva_pub_.publish(inspva_data_[data]);
      }

    }
    if(inspva_thread_.active_ == false) return;
  }
}

void ROSThread::InspvaxThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(inspvax_thread_.mutex_);
    inspvax_thread_.cv_.wait(ul);
    if(inspvax_thread_.active_ == false) return;
    ul.unlock();

    while(!inspvax_thread_.data_queue_.empty()){
      auto data = inspvax_thread_.pop();
      //process
      if(inspvax_data_.find(data) != inspvax_data_.end()){
        inspvax_pub_.publish(inspvax_data_[data]);
      }

    }
    if(inspvax_thread_.active_ == false) return;
  }
}*/
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
      if(imu_data_.find(data) != imu_data_.end()){
        imu_pub_.publish(imu_data_[data]);
        imu_origin_pub_.publish(imu_data_origin_[data]);
        if(imu_data_version_ == 2){
          magnet_pub_.publish(mag_data_[data]);
        }
      }

    }
    if(imu_thread_.active_ == false) return;
  }
}

void ROSThread::TimerCallback(const ros::TimerEvent&)
{
    int64_t current_stamp = ros::Time::now().toNSec();
    if(play_flag_ == true && pause_flag_ == false){
      processed_stamp_ += static_cast<int64_t>(static_cast<double>(current_stamp - pre_timer_stamp_) * play_rate_);
    }
    pre_timer_stamp_ = current_stamp;

    if(play_flag_ == false){
      processed_stamp_ = 0; //reset
      prev_clock_stamp_ = 0;
    }
}

void ROSThread::VelodyneLeftThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
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
//        cout << "Re-load left velodyne from path" << endl;
        //load current data
        pcl::PointCloud<PointXYZIRT> cloud;
        cloud.clear();
        sensor_msgs::PointCloud2 publish_cloud;
        string current_file_name = data_folder_path_ + "/sensor_data/VLP_left" +"/"+ to_string(data) + ".bin";
        if(1){
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);
            while(!file.eof()){
                PointXYZIRT point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
                file.read(reinterpret_cast<char *>(&point.time), sizeof(float));
                cloud.points.push_back (point);
            }
            file.close();

            pcl::toROSMsg(cloud, publish_cloud);
            publish_cloud.header.stamp.fromNSec(data);
            publish_cloud.header.frame_id = "left_velodyne";
            velodyne_left_pub_.publish(publish_cloud);

        }
        previous_file_index = 0;
      }

      //load next data
      pcl::PointCloud<PointXYZIRT> cloud;
      cloud.clear();
      sensor_msgs::PointCloud2 publish_cloud;
      current_file_index = find(next(velodyne_left_file_list_.begin(),max(0,previous_file_index-search_bound_)),velodyne_left_file_list_.end(),to_string(data)+".bin") - velodyne_left_file_list_.begin();
      if(find(next(velodyne_left_file_list_.begin(),max(0,previous_file_index-search_bound_)),velodyne_left_file_list_.end(),velodyne_left_file_list_[current_file_index+1]) != velodyne_left_file_list_.end()){
          string next_file_name = data_folder_path_ + "/sensor_data/VLP_left" +"/"+ velodyne_left_file_list_[current_file_index+1];

          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);
          while(!file.eof()){
              PointXYZIRT point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
              file.read(reinterpret_cast<char *>(&point.time), sizeof(float));
              cloud.points.push_back (point);
          }
          file.close();
          pcl::toROSMsg(cloud, publish_cloud);
          velodyne_left_next_ = make_pair(velodyne_left_file_list_[current_file_index+1], publish_cloud);
      }
      previous_file_index = current_file_index;
    }
    if(velodyne_left_thread_.active_ == false) return;
  }
}
void ROSThread::VelodyneRightThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
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
//        cout << "Re-load right velodyne from path" << endl;
        //load current data
        pcl::PointCloud<PointXYZIRT> cloud;
        cloud.clear();
        sensor_msgs::PointCloud2 publish_cloud;
        string current_file_name = data_folder_path_ + "/sensor_data/VLP_right" +"/"+ to_string(data) + ".bin";
        if(1){
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);
            while(!file.eof()){
                PointXYZIRT point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
                file.read(reinterpret_cast<char *>(&point.time), sizeof(float));
                cloud.points.push_back (point);
            }
            file.close();

            pcl::toROSMsg(cloud, publish_cloud);
            publish_cloud.header.stamp.fromNSec(data);
            publish_cloud.header.frame_id = "right_velodyne";
            velodyne_right_pub_.publish(publish_cloud);

        }
        previous_file_index = 0;
      }

      //load next data
      pcl::PointCloud<PointXYZIRT> cloud;
      cloud.clear();
      sensor_msgs::PointCloud2 publish_cloud;
      current_file_index = find(next(velodyne_right_file_list_.begin(),max(0,previous_file_index-search_bound_)),velodyne_right_file_list_.end(),to_string(data)+".bin") - velodyne_right_file_list_.begin();
      if(find(next(velodyne_right_file_list_.begin(),max(0,previous_file_index-search_bound_)),velodyne_right_file_list_.end(),velodyne_right_file_list_[current_file_index+1]) != velodyne_right_file_list_.end()){
          string next_file_name = data_folder_path_ + "/sensor_data/VLP_right" +"/"+ velodyne_right_file_list_[current_file_index+1];

          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);
          while(!file.eof()){
              PointXYZIRT point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
              file.read(reinterpret_cast<char *>(&point.time), sizeof(float));
              cloud.points.push_back (point);
          }
          file.close();
          pcl::toROSMsg(cloud, publish_cloud);
          velodyne_right_next_ = make_pair(velodyne_right_file_list_[current_file_index+1], publish_cloud);
      }

      previous_file_index = current_file_index;
    }
    if(velodyne_right_thread_.active_ == false) return;
  }
}


void ROSThread::LivoxAviaThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
  while(1){
    std::unique_lock<std::mutex> ul(livox_avia_thread_.mutex_);
    livox_avia_thread_.cv_.wait(ul);
    
    if(livox_avia_thread_.active_ == false) return;
    ul.unlock();

    while(!livox_avia_thread_.data_queue_.empty()){
      auto data = livox_avia_thread_.pop();
      //process

      //publish data
      if(to_string(data) + ".bin" == livox_avia_next_.first){
        //publish
        livox_avia_next_.second.header.stamp.fromNSec(data);
        livox_avia_next_.second.header.frame_id = "livox_avia";
        livox_avia_pub_.publish(livox_avia_next_.second);
      }else{
//        cout << "Re-load right velodyne from path" << endl;
        //load current data
        livox_ros_driver::CustomMsg livox_msg;
        string current_file_name = data_folder_path_ + "/sensor_data/Livox_avia" +"/"+ to_string(data) + ".bin";
        int i = 0;
        if(1){
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);
            while(!file.eof()){
                livox_ros_driver::CustomPoint point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint8_t));
                file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
                file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
                file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint16_t));
                livox_msg.points.push_back(point);
                i++;
            }
            file.close();
            livox_msg.point_num = i;
            livox_msg.header.stamp.fromNSec(data);
            livox_msg.header.frame_id = "livox_avia";
            livox_avia_pub_.publish(livox_msg);

        }
        previous_file_index = 0;
      }

      //load next data
      livox_ros_driver::CustomMsg livox_msg;
      current_file_index = find(next(livox_avia_file_list_.begin(),max(0,previous_file_index-search_bound_)),livox_avia_file_list_.end(),to_string(data)+".bin") - livox_avia_file_list_.begin();
      if(find(next(livox_avia_file_list_.begin(),max(0,previous_file_index-search_bound_)),livox_avia_file_list_.end(),livox_avia_file_list_[current_file_index+1]) != livox_avia_file_list_.end()){
          string next_file_name = data_folder_path_ + "/sensor_data/Livox_avia" +"/"+ livox_avia_file_list_[current_file_index+1];
          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);
          int i = 0;
          while(!file.eof()){
              livox_ros_driver::CustomPoint point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint8_t));
              file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
              file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
              file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint32_t));
              livox_msg.points.push_back(point);
              i++;
          }
          file.close();
          livox_msg.point_num = i;
          livox_msg.header.stamp.fromNSec(data);
          livox_msg.header.frame_id = "livox_avia";
          livox_avia_next_ = make_pair(livox_avia_file_list_[current_file_index+1], livox_msg);
      }

      previous_file_index = current_file_index;
    }
    if(livox_avia_thread_.active_ == false) return;
  }
}

void ROSThread::LivoxTeleThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
  while(1){
    std::unique_lock<std::mutex> ul(livox_tele_thread_.mutex_);
    livox_tele_thread_.cv_.wait(ul);
    
    if(livox_tele_thread_.active_ == false) return;
    ul.unlock();

    while(!livox_tele_thread_.data_queue_.empty()){
      auto data = livox_tele_thread_.pop();
      //process
      
      //publish data
      if(to_string(data) + ".bin" == livox_tele_next_.first){
        //publish
        livox_tele_next_.second.header.stamp.fromNSec(data);
        livox_tele_next_.second.header.frame_id = "livox_tele";
        livox_tele_pub_.publish(livox_tele_next_.second);
      }else{
        //load current data
        livox_ros_driver::CustomMsg livox_msg;
        string current_file_name = data_folder_path_ + "/sensor_data/Livox_tele" +"/"+ to_string(data) + ".bin";
        int i = 0;
        if(1){
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);
            while(!file.eof()){
                livox_ros_driver::CustomPoint point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint8_t));
                file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
                file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
                file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint32_t));
                livox_msg.points.push_back(point);
                i++;
            }
            file.close();
            livox_msg.point_num = i;
            livox_msg.header.stamp.fromNSec(data);
            livox_msg.header.frame_id = "livox_tele";
            livox_tele_pub_.publish(livox_msg);

        }
        previous_file_index = 0;
      }

      //load next data
      livox_ros_driver::CustomMsg livox_msg;
      current_file_index = find(next(livox_tele_file_list_.begin(),max(0,previous_file_index-search_bound_)),livox_tele_file_list_.end(),to_string(data)+".bin") - livox_tele_file_list_.begin();
      if(find(next(livox_tele_file_list_.begin(),max(0,previous_file_index-search_bound_)),livox_tele_file_list_.end(),livox_tele_file_list_[current_file_index+1]) != livox_tele_file_list_.end()){
          string next_file_name = data_folder_path_ + "/sensor_data/Livox_tele" +"/"+ livox_tele_file_list_[current_file_index+1];
          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);
          int i = 0;
          while(!file.eof()){
              livox_ros_driver::CustomPoint point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint8_t));
              file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
              file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
              file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint32_t));
              livox_msg.points.push_back(point);
              i++;
          }
          file.close();
          livox_msg.point_num = i;
          livox_msg.header.stamp.fromNSec(data);
          livox_msg.header.frame_id = "livox_tele";
          livox_tele_next_ = make_pair(livox_tele_file_list_[current_file_index+1], livox_msg);
      }

      previous_file_index = current_file_index;
    }
    if(livox_tele_thread_.active_ == false) return;
  }
}

//ouster
void ROSThread::OusterThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
  while(1){
    std::unique_lock<std::mutex> ul(ouster_thread_.mutex_);
    ouster_thread_.cv_.wait(ul);
    if(ouster_thread_.active_ == false) return;
    ul.unlock();

    while(!ouster_thread_.data_queue_.empty()){
      auto data = ouster_thread_.pop();

      if(to_string(data) + ".bin" == ouster_next_.first){
        //publish
        ouster_next_.second.header.stamp.fromNSec(data);
        ouster_next_.second.header.frame_id = "ouster";
        ouster_pub_.publish(ouster_next_.second);

      }else{
        cout << "Re-load right ouster from path" << endl;
        //load current data
        pcl::PointCloud<OusterPointXYZIRT> cloud;
        cloud.clear();
        sensor_msgs::PointCloud2 publish_cloud;
        string current_file_name = data_folder_path_ + "/sensor_data/ouster" +"/"+ to_string(data) + ".bin";

        if(1){
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);
            while(!file.eof()){
                OusterPointXYZIRT point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
                file.read(reinterpret_cast<char *>(&point.t), sizeof(uint32_t));
                cloud.points.push_back (point);
            }
	   
            file.close();

            pcl::toROSMsg(cloud, publish_cloud);
            publish_cloud.header.stamp.fromNSec(data);
            publish_cloud.header.frame_id = "ouster";
            ouster_pub_.publish(publish_cloud);

        }
        previous_file_index = 0;
      }
      //load next data
      pcl::PointCloud<OusterPointXYZIRT> cloud;
      cloud.clear();
      sensor_msgs::PointCloud2 publish_cloud;
      current_file_index = find(next(ouster_file_list_.begin(),max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),to_string(data)+".bin") - ouster_file_list_.begin();
      if(find(next(ouster_file_list_.begin(),max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),ouster_file_list_[current_file_index+1]) != ouster_file_list_.end()){
          string next_file_name = data_folder_path_ + "/sensor_data/ouster" +"/"+ ouster_file_list_[current_file_index+1];
          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);
          while(!file.eof()){
              OusterPointXYZIRT point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
              file.read(reinterpret_cast<char *>(&point.t), sizeof(uint32_t));
              cloud.points.push_back (point);
          }
          file.close();
          pcl::toROSMsg(cloud, publish_cloud);
          ouster_next_ = make_pair(ouster_file_list_[current_file_index+1], publish_cloud);
      }

      previous_file_index = current_file_index;
    }
    if(ouster_thread_.active_ == false) return;
  }
} //end ouster


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

void ROSThread::FilePlayerStart(const std_msgs::BoolConstPtr& msg)
{
  if(auto_start_flag_ == true){
    cout << "File player auto start" << endl;
    usleep(1000000);
    play_flag_ = false;
    emit StartSignal();
  }
}

void ROSThread::FilePlayerStop(const std_msgs::BoolConstPtr& msg)
{
  cout << "File player auto stop" << endl;
  play_flag_ = true;
  emit StartSignal();
}
void ROSThread::ResetProcessStamp(int position)
{
  if(position > 0 && position < 10000){
    processed_stamp_ = static_cast<int64_t>(static_cast<float>(last_data_stamp_ - initial_data_stamp_)*static_cast<float>(position)/static_cast<float>(10000));
    reset_process_stamp_flag_ = true;
  }

}
