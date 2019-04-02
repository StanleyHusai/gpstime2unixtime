#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <vector>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <hdl_graph_slam/FloorCoeffs.h>

#include <std_srvs/Empty.h>
#include <hdl_graph_slam/SaveMap.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/ros_time_hash.hpp>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/keyframe_updater.hpp>
#include <hdl_graph_slam/loop_detector.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <hdl_graph_slam/map_cloud_generator.hpp>
#include <hdl_graph_slam/nmea_sentence_parser.hpp>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>

#include <amsi/gnss_tools.hpp>


namespace hdl_graph_slam {

class HdlGraphSlamNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;

  HdlGraphSlamNodelet() {
    preX = 0;
    preY = 0;
    initialLLH.resize(3,1);
    initialLLH(0) =  114.19289862;
    initialLLH(1) = 22.3181226456 ;
    initialLLH(2) = 0;

  }
  virtual ~HdlGraphSlamNodelet() {

  }

  virtual void onInit() {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    // init parameters
    map_frame_id = private_nh.param<std::string>("map_frame_id", "map");
    odom_frame_id = private_nh.param<std::string>("odom_frame_id", "odom");
    map_cloud_resolution = private_nh.param<double>("map_cloud_resolution", 0.05);
    trans_odom2map.setIdentity();

    max_keyframes_per_update = private_nh.param<int>("max_keyframes_per_update", 10);

    //
    graph_slam.reset(new GraphSLAM());
    keyframe_updater.reset(new KeyframeUpdater(private_nh));
    loop_detector.reset(new LoopDetector(private_nh));
    map_cloud_generator.reset(new MapCloudGenerator());
    inf_calclator.reset(new InformationMatrixCalculator(private_nh));
    nmea_parser.reset(new NmeaSentenceParser());

    gps_edge_stddev = private_nh.param<double>("gps_edge_stddev", 10000.0);
    floor_edge_stddev = private_nh.param<double>("floor_edge_stddev", 10.0);
    initialGNSSLon = private_nh.param<double>("initialGNSSLon", 114.19095395699307);
    initialGNSSLat = private_nh.param<double>("initialGNSSLat", 22.3216976234397251);
    last_gps_edge_stamp = ros::Time(0);

    // subscribers
    odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/odom", 32));
    cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/filtered_points", 32));
    sync.reset(new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>(*odom_sub, *cloud_sub, 32));
    sync->registerCallback(boost::bind(&HdlGraphSlamNodelet::cloud_callback, this, _1, _2));
    floor_sub = nh.subscribe("/floor_detection/floor_coeffs", 32, &HdlGraphSlamNodelet::floor_coeffs_callback, this);

    if(private_nh.param<bool>("enable_gps", true)) {
      gps_sub = nh.subscribe("/gps/geopoint", 32, &HdlGraphSlamNodelet::gps_callback, this);
      nmea_sub = nh.subscribe("/nmea_sentence", 32, &HdlGraphSlamNodelet::nmea_callback, this);
    }

    // publishers
    markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/markers", 16);
    odom2map_pub = nh.advertise<geometry_msgs::TransformStamped>("/hdl_graph_slam/odom2pub", 16);
    map_points_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/map_points", 1);
    read_until_pub = nh.advertise<std_msgs::Header>("/hdl_graph_slam/read_until", 32);
    optimizedOdom = nh.advertise<nav_msgs::Odometry>("/optimizedOdom", 32); // the odom of the final 
    map2odomTrans = nh.advertise<nav_msgs::Odometry>("/compsensateOdom", 32); // the odom of from /map to odom 

    dump_service_server = nh.advertiseService("/hdl_graph_slam/dump", &HdlGraphSlamNodelet::dump_service, this);
    save_map_service_server = nh.advertiseService("/hdl_graph_slam/save_map", &HdlGraphSlamNodelet::save_map_service, this);

    double graph_update_interval = private_nh.param<double>("graph_update_interval", 3.0);
    double map_cloud_update_interval = private_nh.param<double>("map_cloud_update_interval", 10.0);
    optimization_timer = mt_nh.createTimer(ros::Duration(graph_update_interval), &HdlGraphSlamNodelet::optimization_timer_callback, this);
    map_publish_timer = mt_nh.createWallTimer(ros::WallDuration(map_cloud_update_interval), &HdlGraphSlamNodelet::map_points_publish_timer_callback, this);
  }

private:
  /**
   * @brief received point clouds are pushed to #keyframe_queue
   * @param odom_msg
   * @param cloud_msg
   */
  void cloud_callback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    const ros::Time& stamp = odom_msg->header.stamp;
    Eigen::Isometry3d odom = odom2isometry(odom_msg);
    LiDAROdometry = *odom_msg;
    LiDAROdometryTime.push_back((double)LiDAROdometry.header.stamp.sec + double(LiDAROdometry.header.stamp.nsec)*1e-9);
    LiDAROdometryx.push_back(LiDAROdometry.pose.pose.position.x);
    LiDAROdometryy.push_back(LiDAROdometry.pose.pose.position.y);
    LiDAROdometryz.push_back(LiDAROdometry.pose.pose.position.z);

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if(!keyframe_updater->update(odom)) {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      if(keyframe_queue.empty()) {
        std_msgs::Header read_until;
        read_until.stamp = stamp + ros::Duration(30, 0);
        read_until.frame_id = "/velodyne_points";
        read_until_pub.publish(read_until);
        read_until.frame_id = "/filtered_points";
        read_until_pub.publish(read_until);
      }

      return;
    }

    double accum_d = keyframe_updater->get_accum_distance();
    KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud));

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue.push_back(keyframe);
  }

  /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph
   * @return if true, at least one keyframe is added to the pose graph
   */
  bool flush_keyframe_queue() {
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

    if(keyframe_queue.empty()) {
      return false;
    }

    trans_odom2map_mutex.lock();
    Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
    trans_odom2map_mutex.unlock();

    int num_processed = 0;
    for(int i=0; i<std::min<int>(keyframe_queue.size(), max_keyframes_per_update); i++) {
      num_processed = i;

      const auto& keyframe = keyframe_queue[i];
      new_keyframes.push_back(keyframe);

      Eigen::Isometry3d odom = odom2map * keyframe->odom;
      keyframe->node = graph_slam->add_se3_node(odom);
      keyframe_hash[keyframe->stamp] = keyframe;

      if(i==0 && keyframes.empty()) {
        continue;
      }

      // add edge between keyframes
      const auto& prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i - 1];

      Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
      Eigen::MatrixXd information = inf_calclator->calc_information_matrix(prev_keyframe->cloud, keyframe->cloud, relative_pose);
      graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
    }

    std_msgs::Header read_until;
    read_until.stamp = keyframe_queue[num_processed]->stamp + ros::Duration(30, 0);
    read_until.frame_id = "/velodyne_points";
    read_until_pub.publish(read_until);
    read_until.frame_id = "/filtered_points";
    read_until_pub.publish(read_until);

    keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1);

    return true;
  }

  void nmea_callback(const nmea_msgs::SentenceConstPtr& nmea_msg) {
    std::vector<std::string> str_vec_ptr;
    std::string token;
    std::stringstream ss(nmea_msg->sentence);
    bool find_SOL_COMPUTED =0;
    while (getline(ss, token, ' '))
    {
      if(token == "SOL_COMPUTED") // solutions are computed 
      {
        // std::cout<<"message obtained"<<std::endl;
        find_SOL_COMPUTED = true;
      }
      if( find_SOL_COMPUTED ) // find flag SOL_COMPUTED
      {
        str_vec_ptr.push_back(token);
      }
    }
        if(find_SOL_COMPUTED)
    {
      sensor_msgs::NavSatFix navfix_ ;
      navfix_.header = nmea_msg->header;
      std::cout << std::setprecision(17);
      double lat = strtod((str_vec_ptr[2]).c_str(), NULL);
      double lon = strtod((str_vec_ptr[3]).c_str(), NULL);
      double alt = strtod((str_vec_ptr[4]).c_str(), NULL);
      std::cout << std::setprecision(17);

      navfix_.latitude = lat;
      navfix_.longitude = lon;
      navfix_.altitude = alt;
      if(ini_navf.latitude == NULL)
      {
        ini_navf = navfix_;
        std::cout<<"ini_navf.header  -> "<<ini_navf.header<<std::endl;
        originllh.resize(3, 1);
        originllh(0) = navfix_.longitude;
        originllh(1) = navfix_.latitude;
        originllh(2) = navfix_.altitude;
        std::cout<<"reference longitude: "<<navfix_.longitude<<std::endl;
        std::cout<<"reference latitude: "<<navfix_.latitude<<std::endl;
      }
      Eigen::MatrixXd curLLh; // 
      curLLh.resize(3, 1);
      curLLh(0) = navfix_.longitude;
      curLLh(1) = navfix_.latitude;
      curLLh(2) = navfix_.altitude;

      Eigen::MatrixXd ecef; // 
      ecef.resize(3, 1);
      ecef = gnss_tools_.llh2ecef(curLLh);
      Eigen::MatrixXd eigenENU;; // 
      eigenENU.resize(3, 1);
      eigenENU = gnss_tools_.ecef2enu(originllh,ecef);

      // trans and rotation
      double prex_ = eigenENU(0);
      double prey_ = eigenENU(1);
      double theta = (68.5 )*( 3.141592 / 180.0 ); //
      eigenENU(0) = prex_ * cos(theta) - prey_ * sin(theta) ;
      eigenENU(1) = prex_ * sin(theta) + prey_ * cos(theta) ; 

      geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
      gps_msg->header = nmea_msg->header;
      gps_msg->position.latitude = 1 * eigenENU(1);
      gps_msg->position.longitude = -1 * eigenENU(0);
      gps_msg->position.altitude = eigenENU(2);

      gps_callback(gps_msg);
      std::cout<<"push back message to gps_queue..."<<std::endl;
    }
  }

  /**
   * @brief received gps data is added to #gps_queue
   * @param gps_msg
   */
  void gps_callback(const geographic_msgs::GeoPointStampedConstPtr& gps_msg) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);
    gps_queue.push_back(gps_msg);
  }

  std::ofstream ofs;

  /**
   * @brief
   * @return
   */
  bool flush_gps_queue() {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);

    if(keyframes.empty()) {
      return false;
    }

    const auto& latest_keyframe_stamp = keyframes.back()->stamp;

    bool updated = false;
    auto seek = keyframes.begin();
    for(const auto& gps_msg : gps_queue) {
      seek = std::lower_bound(seek, keyframes.end(), gps_msg->header.stamp, [&](const KeyFrame::Ptr& key, const ros::Time& stamp) { return key->stamp < stamp; });
      if(seek == keyframes.end()) {
        break;
      }

      double  residual = ((*seek)->stamp - gps_msg->header.stamp).toSec(); // GPS-Keyframe and Gps frame time bias difference  
      std::cout << "residual---------------------:" << residual<< std::endl;
      if(std::abs(residual) > 1.25 || (*seek)->utm_coord) { // 0.25 
        continue;
      }

      if(gps_msg->header.stamp - last_gps_edge_stamp < ros::Duration(1.0)   ) { // constant update with GPS initially 30
        continue;
      }

      Eigen::MatrixXd enu_; // the enu for output
      enu_.resize(3, 1);
      double E_ = double(gps_msg->position.latitude); // this is the E in ENU coordinate system
      double N_ = double(gps_msg->position.longitude); // this is the N in ENU coordinate system
      double Covariance = double(gps_msg->position.altitude); // this is the positioning covariance of GNSS positioning
      enu_(0) = E_;
      enu_(1) = N_;
      enu_(2) = double(gps_msg->position.altitude);
      Eigen::Vector3d xyz(enu_(0), enu_(1), enu_(2)); // save the 
      double prex_ = xyz(0);
      double prey_ = xyz(1);

      pt2GPS_ = xyz;
      (*seek)->utm_coord = xyz;

      Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity() / (gps_edge_stddev ); // fixed : 13
      if(Covariance<3200000000) //  for open loop datasheet: set threshold 30            for closed loop datasheet: set threshold 40
      {
        graph_slam->add_se3_prior_xyz_edge((*seek)->node, xyz.head<3>(), information_matrix); // 2D position added into graph_slam
        std::cout << "xyz(0):" << xyz(0)<< std::endl;
        std::cout << "xyz(1):" << xyz(1)<< std::endl;
        std::cout << "xyz(2):" << xyz(2)<< std::endl;

      }
      
      last_gps_edge_stamp = gps_msg->header.stamp;

      updated = true;
    }

    auto remove_loc = std::upper_bound(gps_queue.begin(), gps_queue.end(), latest_keyframe_stamp,
      [=](const ros::Time& stamp, const geographic_msgs::GeoPointStampedConstPtr& geopoint) {
        return stamp < geopoint->header.stamp;
      }
    );
    gps_queue.erase(gps_queue.begin(), remove_loc);

    return updated;
  }


/*
author: WEN Weisong (17902061r@connect.polyu.hk)
function: llh to ecef
input: llh (Matrix3d)
output: ecef (Matrix3d)
*/
Eigen::MatrixXd llh2ecef(Eigen::MatrixXd data) // transform the llh to ecef
{
  Eigen::MatrixXd ecef; // the ecef for output
  ecef.resize(3, 1);
  double a = 6378137.0;
  double b = 6356752.314;
  double n, Rx, Ry, Rz;
  double lon = (double)data(0) * 3.1415926 / 180.0; // lon to radis
  double lat = (double)data(1) * 3.1415926 / 180.0; // lat to radis
  double alt = (double)data(2); // altitude
  n = a * a / sqrt(a * a * cos(lat) * cos(lat) + b * b * sin(lat) * sin(lat));
  Rx = (n + alt) * cos(lat) * cos(lon);
  Ry = (n + alt) * cos(lat) * sin(lon);
  Rz = (b * b / (a * a) * n + alt) * sin(lat);
  ecef(0) = Rx; // return value in ecef
  ecef(1) = Ry; // return value in ecef
  ecef(2) = Rz; // return value in ecef
  return ecef;

  /**************for test purpose*************************
  Eigen::MatrixXd llh;
  llh.resize(3, 1);
  Eigen::MatrixXd ecef;
  ecef.resize(3, 1);
  llh(0) = 114.1772621294604;
  llh(1) = 22.29842880200087;
  llh(2) = 58;
  ecef = llh2ecef(llh);
  cout << "ecef ->: " << ecef << "\n";
  */
}

/*
author: WEN Weisong (17902061r@connect.polyu.hk)
function: ecef to enu
input: original llh, and current ecef (Matrix3d)
output: enu (Matrix3d)
*/
Eigen::MatrixXd ecef2enu(Eigen::MatrixXd originllh, Eigen::MatrixXd ecef) // transform the ecef to enu 
{
  double pi = 3.1415926; // pi 
  double DEG2RAD = pi / 180.0;
  double RAD2DEG = 180.0 / pi;

  Eigen::MatrixXd enu; // the enu for output
  enu.resize(3, 1); // resize to 3X1
  Eigen::MatrixXd oxyz; // the original position 
  oxyz.resize(3, 1); // resize to 3X1

  double x, y, z; // save the x y z in ecef
  x = ecef(0);
  y = ecef(1);
  z = ecef(2);

  double ox, oy, oz; // save original reference position in ecef
  oxyz = llh2ecef(originllh);
  ox = oxyz(0); // obtain x in ecef 
  oy = oxyz(1); // obtain y in ecef
  oz = oxyz(2); // obtain z in ecef

  double dx, dy, dz;
  dx = x - ox;
  dy = y - oy;
  dz = z - oz;

  double lonDeg, latDeg, _; // save the origin lon alt in llh
  lonDeg = originllh(0);
  latDeg = originllh(1);
  double lon = lonDeg * DEG2RAD;
  double lat = latDeg * DEG2RAD;

  //save ENU
  enu(0) = -sin(lon) * dx + cos(lon) * dy;
  enu(1) = -sin(lat) * cos(lon) * dx - sin(lat) * sin(lon) * dy + cos(lat) * dz;
  enu(2) = cos(lat) * cos(lon) * dx + cos(lat) * sin(lon) * dy + sin(lat) * dz;
  return enu;

  /**************for test purpose*****suqare distance is about 37.4 meters********************
  Eigen::MatrixXd llh;  //original
  llh.resize(3, 1);
  llh(0) = 114.1775072541416;
  llh(1) = 22.29817969722738;
  llh(2) = 58;
  Eigen::MatrixXd ecef;
  ecef.resize(3, 1);
  ecef(0) = -2418080.9387265667;
  ecef(1) = 5386190.3905763263;
  ecef(2) = 2405041.9305451373;
  Eigen::MatrixXd enu;
  enu.resize(3, 1);
  enu = ecef2enu(llh, ecef);
  cout << "enu ->: " << enu << "\n";
  */
}

  /**
   * @brief received floor coefficients are added to #floor_coeffs_queue
   * @param floor_coeffs_msg
   */
  void floor_coeffs_callback(const hdl_graph_slam::FloorCoeffsConstPtr& floor_coeffs_msg) {
    if(floor_coeffs_msg->coeffs.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);
    floor_coeffs_queue.push_back(floor_coeffs_msg);
  }

  /**
   * @brief this methods associates floor coefficients messages with registered keyframes, and then adds the associated coeffs to the pose graph
   * @return if true, at least one floor plane edge is added to the pose graph
   */
  bool flush_floor_queue() {
    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);

    if(keyframes.empty()) {
      return false;
    }

    const auto& latest_keyframe_stamp = keyframes.back()->stamp;

    bool updated = false;
    for(const auto& floor_coeffs : floor_coeffs_queue) {
      if(floor_coeffs->header.stamp > latest_keyframe_stamp) {
        break;
      }

      auto found = keyframe_hash.find(floor_coeffs->header.stamp);
      if(found == keyframe_hash.end()) {
        continue;
      }

      const auto& keyframe = found->second;

      Eigen::Vector4d coeffs(floor_coeffs->coeffs[0], floor_coeffs->coeffs[1], floor_coeffs->coeffs[2], floor_coeffs->coeffs[3]);
      Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / floor_edge_stddev);
      graph_slam->add_se3_plane_edge(keyframe->node, graph_slam->floor_plane_node, coeffs, information);

      keyframe->floor_coeffs = coeffs;

      updated = true;
    }

    auto remove_loc = std::upper_bound(floor_coeffs_queue.begin(), floor_coeffs_queue.end(), latest_keyframe_stamp,
      [=](const ros::Time& stamp, const hdl_graph_slam::FloorCoeffsConstPtr& coeffs) {
        return stamp < coeffs->header.stamp;
      }
    );
    floor_coeffs_queue.erase(floor_coeffs_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief generate a map point cloud and publish it
   * @param event
   */
  void map_points_publish_timer_callback(const ros::WallTimerEvent& event) {
    // if(!map_points_pub.getNumSubscribers()) {
    //   return;
    // }     

    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, 0.05);
    if(!cloud) {
      return;
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud, *cloud_msg);

    map_points_pub.publish(cloud_msg);
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
   * @param event
   */
  void optimization_timer_callback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    // add keyframes and floor coeffs in the queues to the pose graph
    if(!flush_keyframe_queue() & !flush_floor_queue() & !flush_gps_queue()) {
      std_msgs::Header read_until;
      read_until.stamp = event.current_real + ros::Duration(30, 0);
      read_until.frame_id = "/velodyne_points";
      read_until_pub.publish(read_until);
      read_until.frame_id = "/filtered_points";
      read_until_pub.publish(read_until);

      return;
    }

    // loop detection
    std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes, *graph_slam);
    for(const auto& loop : loops) {
      Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
      Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose);
      graph_slam->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);
    }

    std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes));
    new_keyframes.clear();

    // optimize the pose graph
    graph_slam->optimize();

    // publish tf
    const auto& keyframe = keyframes.back();
    Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse();
    trans_odom2map_mutex.lock();
    trans_odom2map = trans.matrix().cast<float>();
    trans_odom2map_mutex.unlock();

    // publish pose of the final optimization
    nav_msgs::Odometry odom_;
    nav_msgs::Odometry comOdom;
    // odom_.header.stamp = event.current_real; //stamp
    odom_.header.stamp = keyframe->stamp; //stamp
    odom_.header.frame_id = "velodyne";

    Eigen::Vector3d pos = keyframe->node->estimate().translation(); // this is 


    double graph_time = ((double)odom_.header.stamp.sec + double(odom_.header.stamp.nsec)*1e-9); // time stamp of the keyframe
    double bias =100000;
    int index_ =0;
    for(int index= 0; index<LiDAROdometryTime.size(); index++)
    {
      if(fabs (graph_time - LiDAROdometryTime[index]) <bias)
      {
        index_ = index;
        bias = fabs (graph_time - LiDAROdometryTime[index]);
      }
      
    }
    odom_.pose.pose.position.x = pos.x() - LiDAROdometryx[index_];
    odom_.pose.pose.position.y = pos.y() - LiDAROdometryy[index_];
    odom_.pose.pose.position.z = pos.z() - LiDAROdometryz[index_];
    std::cout << "time bias:" << fabs (graph_time - LiDAROdometryTime[index_])  << std::endl; 

    odom_.child_frame_id = "map";
    odom_.twist.twist.linear.x = 0.0;
    odom_.twist.twist.linear.y = 0.0;
    odom_.twist.twist.angular.z = 0.0;

    optimizedOdom.publish(odom_);


    if(map_points_pub.getNumSubscribers()) {
      std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
      std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(),
        [=](const KeyFrame::Ptr& k) {
          return std::make_shared<KeyFrameSnapshot>(k);
      });

      std::lock_guard<std::mutex> lock(keyframes_snapshot_mutex);
      keyframes_snapshot.swap(snapshot);
    }

//     if(odom2map_pub.getNumSubscribers()) {
    if(1) {
      geometry_msgs::TransformStamped ts = matrix2transform(keyframe->stamp, trans.matrix().cast<float>(), map_frame_id, odom_frame_id);
      odom2map_pub.publish(ts);
      comOdom.pose.pose.position.x = ts.transform.translation.x;
      comOdom.pose.pose.position.y = ts.transform.translation.y;
      comOdom.pose.pose.position.z = ts.transform.translation.z;

      comOdom.child_frame_id = "map";
      comOdom.twist.twist.linear.x = 0.0;
      comOdom.twist.twist.linear.y = 0.0;
      comOdom.twist.twist.angular.z = 0.0;
      map2odomTrans.publish(comOdom);
    }

    if(markers_pub.getNumSubscribers()) {
      auto markers = create_marker_array(event.current_real);
      markers_pub.publish(markers);
    }
  }

  /**
   * @brief create visualization marker
   * @param stamp
   * @return
   */
  visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp) const {
    visualization_msgs::MarkerArray markers;
    markers.markers.resize(4);

    // node markers
    visualization_msgs::Marker& traj_marker = markers.markers[0];
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = stamp;
    traj_marker.ns = "nodes";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 2; // initially 0.5

    traj_marker.points.resize(keyframes.size());
    traj_marker.colors.resize(keyframes.size());
    for(int i=0; i<keyframes.size(); i++) {  // Node
      Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
      traj_marker.points[i].x = pos.x();
      traj_marker.points[i].y = pos.y();
      traj_marker.points[i].z = pos.z();

      double p = static_cast<double>(i) / keyframes.size();
//       traj_marker.colors[i].r = 1.0 - p;
//       traj_marker.colors[i].g = p;
      traj_marker.colors[i].r = 1.0;
      traj_marker.colors[i].g = 0.0;
      traj_marker.colors[i].b = 0.0;
      traj_marker.colors[i].a = 1.0;
    }

    // edge markers
    visualization_msgs::Marker& edge_marker = markers.markers[1];
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "edges";
    edge_marker.id = 1;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;

    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.9; // initially 0.05

    edge_marker.points.resize(graph_slam->graph->edges().size() * 2);
    edge_marker.colors.resize(graph_slam->graph->edges().size() * 2);

    auto edge_itr = graph_slam->graph->edges().begin();
    for(int i=0; edge_itr != graph_slam->graph->edges().end(); edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *edge_itr;
      g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
      if(edge_se3) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = v2->estimate().translation();

        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z();
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z();

        double p1 = static_cast<double>(v1->id()) / graph_slam->graph->vertices().size();
        double p2 = static_cast<double>(v2->id()) / graph_slam->graph->vertices().size();
        edge_marker.colors[i*2].r = 0.0 ;
        edge_marker.colors[i*2].g = 0.0;
	edge_marker.colors[i*2].b = 1.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].r = 0;
        edge_marker.colors[i*2 + 1].g = 0;
	edge_marker.colors[i*2 + 1].b = 1.0;
        edge_marker.colors[i*2 + 1].a = 1.0;

        if(std::abs(v1->id() - v2->id()) > 2) {
          edge_marker.points[i*2].z += 0.5;
          edge_marker.points[i*2 + 1].z += 0.5;
        }

        continue;
      }

      g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge); // Plane 
      if(edge_plane) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_plane->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2(pt1.x(), pt1.y(), 0.0);

        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z();
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z();

        edge_marker.colors[i*2].b = 1.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].b = 1.0;
        edge_marker.colors[i*2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE3PriorXY* edge_priori_xy = dynamic_cast<g2o::EdgeSE3PriorXY*>(edge); // GPS
      if(edge_priori_xy) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xy->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = Eigen::Vector3d::Zero();
        pt2.head<2>() = edge_priori_xy->measurement();

        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z();
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z();
        // std::cout << "pt2(0)->:" << pt2(0)<< std::endl;
        // std::cout << "pt2(1)->:" << pt2(1)<< std::endl;

        edge_marker.colors[i*2].r = 1.0;
	      edge_marker.colors[i*2].g = 0.0;
        edge_marker.colors[i*2].b = 0.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].r = 1.0;
	      edge_marker.colors[i*2 + 1].g = 0.0;
	      edge_marker.colors[i*2 + 1].b = 0.0;
        edge_marker.colors[i*2 + 1].a = 1.0;

        continue;
      }
    }

    // sphere for loop closure 
    // visualization_msgs::Marker& sphere_marker = markers.markers[3];
    // sphere_marker.header.frame_id = "map";
    // sphere_marker.header.stamp = stamp;
    // sphere_marker.ns = "loop_close_radius";
    // sphere_marker.id = 0;
    // sphere_marker.type = visualization_msgs::Marker::SPHERE;

    // if(!keyframes.empty()) {
    //   Eigen::Vector3d pos = keyframes.back()->node->estimate().translation();
    //   sphere_marker.pose.position.x = pos.x();
    //   sphere_marker.pose.position.y = pos.y();
    //   sphere_marker.pose.position.z = pos.z();
    // }
    // sphere_marker.pose.orientation.w = 1.0;
    // sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = loop_detector->get_distance_thresh() * 2.0;

    // sphere_marker.color.r = 1.0;
    // sphere_marker.color.a = 0.3;

    return markers;
  }

  /**
   * @brief dump all data to the current directory
   * @param req
   * @param res
   * @return
   */
  bool dump_service(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    std::array<char, 64> buffer;
    buffer.fill(0);
    time_t rawtime;
    time(&rawtime);
    const auto timeinfo = localtime(&rawtime);
    strftime(buffer.data(), sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
    std::string directory(buffer.data());

    if(!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
    }

    std::cout << "all data dumped to:" << std::flush;
    system("pwd");

    graph_slam->save(directory + "/graph.g2o");
    for(int i=0; i<keyframes.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;

      keyframes[i]->dump(sst.str());
    }

    return true;
  }

  /**
   * @brief save map data as pcd
   * @param req
   * @param res
   * @return
   */
  bool save_map_service(hdl_graph_slam::SaveMapRequest& req, hdl_graph_slam::SaveMapResponse& res) {
    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, req.resolution);
    if(!cloud) {
      res.success = false;
      return true;
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    int ret = pcl::io::savePCDFileBinary(req.destination, *cloud);
    res.success = ret == 0;

    return true;
  }
private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;
  ros::Timer optimization_timer;
  ros::WallTimer map_publish_timer;

  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
  std::unique_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>> sync;

  ros::Subscriber gps_sub;
  ros::Subscriber nmea_sub;
  ros::Subscriber floor_sub;

  ros::Publisher markers_pub;

  std::string map_frame_id;
  std::string odom_frame_id;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map;
  ros::Publisher odom2map_pub;

  ros::Publisher read_until_pub;
  ros::Publisher map_points_pub;
  ros::Publisher optimizedOdom;
  ros::Publisher map2odomTrans;

  ros::ServiceServer dump_service_server;
  ros::ServiceServer save_map_service_server;

  // keyframe queue
  std::mutex keyframe_queue_mutex;
  std::deque<KeyFrame::Ptr> keyframe_queue;

  // gps queue
  double gps_edge_stddev;
  double preX;
  double preY;
  Eigen::Vector3d pt2GPS_ ;
  boost::optional<Eigen::Vector3d> zero_utm;
  boost::optional<Eigen::Vector3d> zero_utm_;
  ros::Time last_gps_edge_stamp;
  std::mutex gps_queue_mutex;
  std::deque<geographic_msgs::GeoPointStampedConstPtr> gps_queue;

  double initialGNSSLon; // initial gps longitude 
  double initialGNSSLat; // initial gps latitude
  Eigen::MatrixXd initialLLH; // the ecef for output
  nav_msgs::Odometry LiDAROdometry; // LiDAR odometry information
  std::vector <double> LiDAROdometryTime;
  std::vector <double> LiDAROdometryx;
  std::vector <double> LiDAROdometryy;
  std::vector <double> LiDAROdometryz;

  // gps sentence
  sensor_msgs::NavSatFix ini_navf ; // initial sensor msg
  // gnss_tools
  GNSS_Tools gnss_tools_;
  Eigen::MatrixXd originllh; // origin llh



  // floor_coeffs queue
  double floor_edge_stddev;
  std::mutex floor_coeffs_queue_mutex;
  std::deque<hdl_graph_slam::FloorCoeffsConstPtr> floor_coeffs_queue;

  // for map cloud generation
  double map_cloud_resolution;
  std::mutex keyframes_snapshot_mutex;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
  std::unique_ptr<MapCloudGenerator> map_cloud_generator;

  // graph slam
  // all the below members must be accessed after locking main_thread_mutex
  std::mutex main_thread_mutex;

  int max_keyframes_per_update;
  std::deque<KeyFrame::Ptr> new_keyframes;

  std::vector<KeyFrame::Ptr> keyframes;
  std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;

  std::unique_ptr<GraphSLAM> graph_slam;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;
  std::unique_ptr<NmeaSentenceParser> nmea_parser;

  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
};

}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::HdlGraphSlamNodelet, nodelet::Nodelet)
