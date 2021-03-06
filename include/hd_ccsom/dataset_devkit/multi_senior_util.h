#pragma once

#include <fstream>

#include <std_srvs/Empty.h>
#include <mutex>
#include <thread>


#include <time.h> 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "tf2_msgs/TFMessage.h"
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>

#include "seg_point_type.h"
#include "load_matrix.hpp"
// #include "semantickitti_util.h"

#include <numeric>


class Semanticmulti {
  public:
    Semanticmulti(ros::NodeHandle& nh,
             double resolution, double block_depth,
             double sf2, double ell,
             int num_class, double free_thresh,
             double occupied_thresh, float var_thresh, 
	           double ds_resolution,
             double free_resolution, double max_range,
             float prior, std::string name,
             std::string frientname, std::string cloudTP, 
             std::string cloudTP_gazebo, std::string cloudTP_sk, 
             std::string sequences, int color_num,
             int methods)
      : nh_(nh)
      , resolution_(resolution)
      , num_class_(num_class)
      , ds_resolution_(ds_resolution)
      , free_resolution_(free_resolution)
      , max_range_(max_range) 
      , name_(name)
      , frientname_(frientname)
      , sequences_(sequences)
      , color_num_(color_num)
      , methods_(methods){
        local_map_ = new hd_ccsom::SemanticBKIOctoMap(resolution, block_depth, num_class, sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);
        global_map_ = new hd_ccsom::SemanticBKIOctoMap(resolution, block_depth, num_class, sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);
        m_local_pub_ = new hd_ccsom::MarkerArrayPub(nh_, name_+"/local_map", resolution);
        m_global_pub_ = new hd_ccsom::MarkerArrayPub(nh_, name_+"/global_map", resolution);

        m_localMapPub = nh_.advertise<sensor_msgs::PointCloud2>(name_+"/map_point", 2, true);  // ???????????????????????????????????????
        m_friendMapSub = nh_.subscribe(frientname_+"/map_point", 10, &Semanticmulti::friendMapCallback, this);   //?????????????????????????????????

         // ?????????SemanticKITTI???rosbag
        m_pointCloudSub_kitti = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,cloudTP_sk, 1000);    //??????rosbag??????????????????
        m_tfPointCloudSub_kitti = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub_kitti, m_tfListener, "/map", 1000);  //??????tf???????????????????????????  world???frameid
        m_tfPointCloudSub_kitti->registerCallback(boost::bind(&Semanticmulti::insertCloudCallback, this, _1));   //????????????
        
        // ???????????????(????????????)???????????????tf?????????????????????
        // m_pointCloudSub_kitti_no_tf = nh_.subscribe(cloudTP_sk, 100, &Semanticmulti::insert_no_tf, this);   //?????????????????????????????????

        //???????????????????????????
        m_pointCloudSub_senior = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/seg_cloud", 1000);    //??????rosbag??????????????????
        m_tfPointCloudSub_senior = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub_senior, m_tfListener, "/map", 1000);  //??????tf???????????????????????????  world???frameid
        m_tfPointCloudSub_senior->registerCallback(boost::bind(&Semanticmulti::insertCloudCallback2, this, _1));   //????????????
        m_pointCloudSub_senior_2 = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/seg_cloud_2", 1000);    //??????rosbag??????????????????
        m_tfPointCloudSub_senior_2 = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub_senior, m_tfListener, "/map", 1000);  //??????tf???????????????????????????  world???frameid
        m_tfPointCloudSub_senior_2->registerCallback(boost::bind(&Semanticmulti::insertCloudCallback2, this, _1));   //????????????

        //?????????gazebo???myself????????????
        m_pointCloudSub_gazebo = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloudTP_gazebo, 1000);    //??????rosbag??????????????????
        m_tfPointCloudSub_gazebo = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub_gazebo, m_tfListener, "/map", 1000);  //??????tf???????????????????????????  world???frameid
        m_tfPointCloudSub_gazebo->registerCallback(boost::bind(&Semanticmulti::insertCloudCallback3, this, _1));   //????????????

        m_manualTrigerSrv = nh_.advertiseService("manual_triger", &Semanticmulti::manualTriger, this);   //????????????????????????
        m_label_write = nh_.advertiseService("label_write", &Semanticmulti::labelWrite, this);   //????????????????????????
        m_label_write_global = nh_.advertiseService("label_write_global", &Semanticmulti::labelWriteglobal, this);   //????????????????????????

        m_globalMapViewerTimer = nh_.createTimer(ros::Duration(10.0), &Semanticmulti::pubGlobalMap, this);   //????????????5s??????????????????
        m_localMapViewerTimer = nh_.createTimer(ros::Duration(3.0), &Semanticmulti::pubLocalMap, this);   //????????????2s??????

        count_pc = 0;
      	init_trans_to_ground_ << 1, 0, 0, 0,
                                 0, 0, 1, 0,
                                 0,-1, 0, 1,
                                 0, 0, 0, 1;
        // SemanticKitti?????????????????????????????????markerarray_pub.h???
        label2label =  { 0,  1,  10, 11, 13, 15, 16, 18, 20, 30,  31,  32,  40,  44,  48,  49,  50,
                             51, 52, 60, 70, 71, 72, 80, 81, 99, 252, 256, 253, 254, 255, 257, 258, 259 };
        label3label =  { 0,  0,  1, 2, 5, 3, 5, 4,  5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0, 9, 15, 16, 17, 18,
                                      19, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        // ?????????????????????rgb????????????????????????markerarray_pub.h???
        labelcolor =  { 4605510, 15999976, 8405120, 12491161, 10066329, 14423100, 6710940, 
                            7048739, 14474240, 20580, 10025880, 7801632, 142, 230, 16711680, 16427550,
                            15460, 70, 4620980};
        std::cout << frientname_ << " to" << name_ ;
        load_matrix(nh_, "friendToMe", friendToMe);
        start_time_ = ros::Time::now().toNSec();
      }

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// -----------------------------------------------------------------------????????????SemanticKitti??????---------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------//

      void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud)  //??????????????????tf???????????????tf???????????????????????????
      {
          hd_ccsom::point3f origin;
          pcl::PointCloud<PointSemanticKITTI> pc;
          pcl::PointCloud<PointSemanticKITTI> pc_global;
          pcl::fromROSMsg(*cloud, pc);
          pcl::PointCloud<pcl::PointXYZL>::Ptr  pc_after (new pcl::PointCloud<pcl::PointXYZL>);   
          tf::StampedTransform sensorToWorldTf;   //?????????????????????????????????
          try
          {
              // ??????????????????????????????????????? ??????????????????????????????????????????????????????tf??????????????????????????????
              m_tfListener.lookupTransform("/map", cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);   //?????????cloud->header.frame_id???left_camera????????????/world
          }
          catch (tf::TransformException &ex)
          {
              ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
              return;
          }

          Eigen::Matrix4f sensorToWorld;
          pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);   //??????????????????
          pcl::transformPointCloud(pc, pc_global, sensorToWorld);   //?????????????????????????????????
          lidar_poses_.push_back(sensorToWorld);
          std::cout<< "---------------------------This is the " << name_ <<"  " <<count_pc  << "st------------------------------"<< std::endl;
          count_pc = count_pc + 1;
          for (int i = 0; i <   pc_global.points.size(); i++)
          {
            pcl::PointXYZL p;
            std::vector<int>::iterator ite = find(label2label.begin(), label2label.end(), pc_global.points[i].label);
            p.x = pc_global.points[i].x;
            p.y = pc_global.points[i].y;
            p.z = pc_global.points[i].z;
            // int num_19 = std::distance(std::begin(label2label), ite);
            // p.label = label3label[num_19] +1;
            p.label = std::distance(std::begin(label2label), ite) +1;
            pc_after->points.push_back(p);
          }
          origin.x() =sensorToWorld(0, 3);  // ??????????????????????????????
          origin.y() = sensorToWorld(1, 3);
          origin.z() = sensorToWorld(2, 3);
          local_map_->insert_pointcloud_test(*pc_after, origin, ds_resolution_, free_resolution_, max_range_); 
          global_map_->insert_pointcloud(*pc_after, origin, ds_resolution_, free_resolution_, max_range_);   // ??????????????????????????????????????????
      }

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// -----------------------------------------------------------------------????????????SemanticKitti insert_no_tf????????????----------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------//
      // void insert_no_tf(const sensor_msgs::PointCloud2::ConstPtr &cloud)  //??????????????????tf???????????????tf???????????????????????????
      // {
      //     hd_ccsom::point3f origin;
      //     pcl::PointCloud<PointSemanticKITTI> pc;
      //     pcl::PointCloud<PointSemanticKITTI> pc_global;
      //     pcl::fromROSMsg(*cloud, pc);
      //     pcl::PointCloud<pcl::PointXYZL>::Ptr  pc_after (new pcl::PointCloud<pcl::PointXYZL>);
      //     Eigen::Matrix4f calibration;
      //     int count_pc_real;
      //     if (sequences_ == "01")
      //     {
      //          calibration <<  4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
      //           -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
      //            9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
      //  	                 0                ,  0                ,  0                ,  1.000000000000000;
      //           if  (name_ == "/robot2")
      //           {
      //             count_pc_real = count_pc+550;      
      //           }
      //           else
      //           {
      //             count_pc_real = count_pc;      
      //           }
      //     }
      //     if (sequences_ == "04")
      //     {
      //          calibration <<  -0.001857739385241, -0.999965951350955, -0.008039975204516, -0.004784029760483,
      //                   -0.006481465826011,  0.008051860151134, -0.999946608177406, -0.073374294642306,
      //                    0.999977309828677, -0.001805528627661, -0.006496203536139, -0.333996806443304,
      //  	                 0                ,  0                ,  0                ,  1.000000000000000;
      //           if  (name_ == "/robot2")
      //           {
      //             count_pc_real = count_pc+135;      
      //           }
      //           else
      //           {
      //             count_pc_real = count_pc;      
      //           }
      //     }
      //     if (sequences_ == "05")
      //     {
      //          calibration <<  -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
      //           -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02, 
      //           9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
      //  	                 0                ,  0                ,  0                ,  1.000000000000000;
      //           if  (name_ == "/robot2")
      //           {
      //             count_pc_real = count_pc+1378;      
      //           }
      //           else
      //           {
      //             count_pc_real = count_pc;      
      //           }
      //     }
      //     if (sequences_ == "07")
      //     {
      //          calibration <<
      //                    -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
      //                     -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02, 
      //                     9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
      //  	                 0                ,  0                ,  0                ,  1.000000000000000;
      //           if  (name_ == "/robot2")
      //           {
      //             count_pc_real = count_pc+549;      
      //           }
      //           else
      //           {
      //             count_pc_real = count_pc;      
      //           }
      //     }
      //     Eigen::Matrix4f transform = lidar_poses_[count_pc_real];
      //     Eigen::Matrix4f new_transform = init_trans_to_ground_ * transform * calibration;
      //     pcl::transformPointCloud(pc, pc_global, new_transform);   //?????????????????????????????????
      //     std::cout<< "---------------------------This is the " << name_ <<"  " <<count_pc_real  << "st------------------------------"<< std::endl;
      //     count_pc = count_pc + 1;
      //     for (int i = 0; i <   pc_global.points.size(); i++)
      //     {
      //       pcl::PointXYZL p;
      //       std::vector<int>::iterator ite = find(label2label.begin(), label2label.end(), pc_global.points[i].label);
      //       p.x = pc_global.points[i].x;
      //       p.y = pc_global.points[i].y;
      //       p.z = pc_global.points[i].z;
      //       // int num_19 = std::distance(std::begin(label2label), ite);
      //       // p.label = label3label[num_19] +1;
      //       p.label = std::distance(std::begin(label2label), ite) +1;
      //       pc_after->points.push_back(p);
      //     }
      //     origin.x() = transform(0, 3);  // ??????????????????????????????
      //     origin.y() =  transform(1, 3);
      //     origin.z() =  transform(2, 3);
      //     if (methods_ == 2 || methods_ == 4)
      //       local_map_->insert_pointcloud_csm(*pc_after, origin, ds_resolution_, free_resolution_, max_range_); 
      //     else
      //       local_map_->insert_pointcloud_test(*pc_after, origin, ds_resolution_, free_resolution_, max_range_); 
      //     // global_map_->insert_pointcloud(*pc_after, origin, ds_resolution_, free_resolution_, max_range_);   // ??????????????????????????????????????????
      // }

// //--------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// // -----------------------------------------------------------------------????????????????????????----------------------------------------------------------------------//
// //--------------------------------------------------------------------------------------------------------------------------------------------------------------------//
      void insertCloudCallback2(const sensor_msgs::PointCloud2::ConstPtr &cloud)  //??????????????????tf???????????????tf???????????????????????????
      {
          hd_ccsom::point3f origin;
          pcl::PointCloud<PointXYZRGBSemanticsBayesian> pc;
          pcl::PointCloud<PointXYZRGBSemanticsBayesian> pc_global;
          pcl::PointCloud<pcl::PointXYZL>::Ptr  pc_con_pt (new pcl::PointCloud<pcl::PointXYZL>);   
          pcl::fromROSMsg(*cloud, pc);
          // std::lock_guard<std::mutex> local_map_guard(m_localMapMutex);  //???????????????????????????????????????
          // std::lock_guard<std::mutex> global_map_guard(m_globalMapMutex);  //???????????????????????????????????????
          count_pc = count_pc + 1;
          tf::StampedTransform sensorToWorldTf;   //?????????????????????????????????
          try
          {
              // ??????????????????????????????????????? ??????????????????????????????????????????????????????tf??????????????????????????????
              m_tfListener.lookupTransform("/map", cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);   //?????????cloud->header.frame_id???left_camera????????????/world
          }
          catch (tf::TransformException &ex)
          {
              ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
              return;
          }
          Eigen::Matrix4f sensorToWorld;
          pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);   //??????????????????
          pcl::transformPointCloud(pc, pc_global, sensorToWorld);   //?????????????????????????????????
          std::cout<< "---------------------------This is the " << name_ <<"  "<<  count_pc  << "st------------------------------"<< std::endl;
          clock_t startTime,endTime;
          startTime = ros::Time::now().toNSec();
          float label_raw;
          std::uint32_t label, rgb;
          for (int i = 0; i <   pc_global.points.size(); i++)
          {
            label_raw = pc_global.points[i].semantic_color1;  //??????????????????????????????????????????
            std::memcpy(&rgb, &label_raw, sizeof(uint32_t));  //?????????????????????????????????RGB??????
            std::vector<uint32_t>::iterator ite = find(labelcolor.begin(), labelcolor.end(), rgb);
            label = std::distance(std::begin(labelcolor), ite)+1;
            pcl::PointXYZL p;
            p.x = pc_global.points[i].x;
            p.y = pc_global.points[i].y;
            p.z = pc_global.points[i].z;
            p.label = label;
            pc_con_pt->points.push_back(p);
          }
          origin.x() =sensorToWorld(0, 3);  // ??????????????????????????????
          origin.y() = sensorToWorld(1, 3);
          origin.z() = sensorToWorld(2, 3);
          local_map_->insert_pointcloud_test(*pc_con_pt, origin, ds_resolution_, free_resolution_, max_range_); 
          global_map_->insert_pointcloud_test(*pc_con_pt, origin, ds_resolution_, free_resolution_, max_range_); 
          // std::cout<< name_ <<"  "<< "waitting for next scan...................."<<std::endl;
          endTime = ros::Time::now().toNSec();
          // if (name_ == "/robot1")
            std::cout << "The run time is:" << (double)(endTime - startTime) / 10e6 << "ms" << std::endl;
      }

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// -----------------------------------------------------------------------????????????Gazebo Myself??????----------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------//

      void insertCloudCallback3(const sensor_msgs::PointCloud2::ConstPtr &cloud)  //??????????????????tf???????????????tf???????????????????????????
      {
          hd_ccsom::point3f origin;
          pcl::PointCloud<pcl::PointXYZL> pc;
          pcl::PointCloud<pcl::PointXYZL> pc_global;
          pcl::fromROSMsg(*cloud, pc);

          count_pc = count_pc + 1;
          tf::StampedTransform sensorToWorldTf;   //?????????????????????????????????
          try
          {
              // ??????????????????????????????????????? ??????????????????????????????????????????????????????tf??????????????????????????????
              // gazebo??????velodyne???map
              // ???????????????rslidar???map
              m_tfListener.lookupTransform("/map", cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);   //?????????cloud->header.frame_id???left_camera????????????/world
          }
          catch (tf::TransformException &ex)
          {
              ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
              return;
          }
          Eigen::Matrix4f sensorToWorld;
          pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);   //??????????????????
          pcl::transformPointCloud(pc, pc_global, sensorToWorld);   //?????????????????????????????????
          std::cout<< "---------------------------This is the " << name_ <<"  " <<count_pc  << "st------------------------------"<< std::endl;
          origin.x() =sensorToWorld(0, 3);  // ??????????????????????????????
          origin.y() = sensorToWorld(1, 3);
          origin.z() = sensorToWorld(2, 3);
          clock_t startTime,endTime;
          startTime = ros::Time::now().toNSec();
          local_map_->insert_pointcloud_test(pc_global, origin, ds_resolution_, free_resolution_, max_range_); 
          global_map_->insert_pointcloud_test(pc_global, origin, ds_resolution_, free_resolution_, max_range_); 
          endTime = ros::Time::now().toNSec();
          // std::cout << "The run time is:" << (double)(endTime - startTime) / 10e6 << "ms" << std::endl;
      }


    void pubLocalMap(const ros::TimerEvent &event)   //??????????????????
    {
      std::thread pubLocalMapObj(std::bind(&Semanticmulti::pubLocalMapRviz, this));  //????????????????????????????????????????????????
      if (pubLocalMapObj.joinable())  //???????????????????????????????????????
      {
          pubLocalMapObj.detach();
      }
    }
    void pubLocalMapRviz( void )
    {
      int need_out = 0;
      std::vector<float>  var_vec;
      m_local_pub_->clear_map(resolution_);
      std::lock_guard<std::mutex> local_map_guard(m_localMapMutex);  //???????????????????????????????????????
      int i = 0;
      // ???bkioctomap???????????????????????????????????????????????????begin_leaf()???bkioctomap?????????block??????????????????end_leaf()???????????????block??????????????????
      for (auto it = local_map_->begin_leaf(); it != local_map_->end_leaf(); ++it) {
        // if ( it.get_node().get_state() == hd_ccsom::State::OCCUPIED|| it.get_node().get_state() == hd_ccsom::State::FREE) {  // ???????????????????????????  it.get_node().get_state() == hd_ccsom::State::OCCUPIED || it.get_node().get_state() == hd_ccsom::State::FREE
        if ( it.get_node().get_state() == hd_ccsom::State::OCCUPIED) { 
          hd_ccsom::point3f p = it.get_loc();
          m_local_pub_->insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), it.get_node().get_semantics(), color_num_);  //?????????4???kitti???3??? semantickitti???2
          i++;
          if (need_out == 1)
          {
            std::vector<float> vars(num_class_);
            it.get_node().get_vars(vars);
            var_vec.push_back(vars[it.get_node().get_semantics()]);
          }
        }
      }
      if (need_out == 1)
      {
        double mean = accumulate(var_vec.begin(), var_vec.end(), 0.0) ;
        mean = mean/var_vec.size();
        std::cout << name_ <<" var is  "  <<std::fixed <<std::setprecision(6) <<  mean << "   "<< var_vec.size() <<std::endl;
        clock_t now_time = ros::Time::now().toNSec();
        // std::cout << now_time << "   "<< start_time_ <<std::endl;
        std::cout << name_ <<" use "<< (double)(now_time - start_time_) / 10e8  <<" seconds ,  has node num:" << i<<std::endl;
      }
      m_local_pub_->publish();
    }


    void pubGlobalMap(const ros::TimerEvent &event)   //????????????????????????
    {
      std::thread pubGlobalMapObj(std::bind(&Semanticmulti::pubGlobalMapRviz, this));  //????????????????????????????????????????????????
      if (pubGlobalMapObj.joinable())  //???????????????????????????????????????
      {
          pubGlobalMapObj.detach();
      }
    }
    void pubGlobalMapRviz( void )
    {
      m_global_pub_->clear_map(resolution_);
      std::lock_guard<std::mutex> global_map_guard(m_globalMapMutex);  //???????????????????????????????????????
      int i = 0;
      for (auto it = global_map_->begin_leaf(); it != global_map_->end_leaf(); ++it) {
        // if ( it.get_node().get_state() == hd_ccsom::State::OCCUPIED|| it.get_node().get_state() == hd_ccsom::State::FREE) {  // ???????????????????????????  it.get_node().get_state() == hd_ccsom::State::OCCUPIED || it.get_node().get_state() == hd_ccsom::State::FREE
        if ( it.get_node().get_state() == hd_ccsom::State::OCCUPIED) { 
          hd_ccsom::point3f p = it.get_loc();
          m_global_pub_->insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), it.get_node().get_semantics(), color_num_);
          i++;
        }
      }
      m_global_pub_->publish();
      // std::cout << name_ <<" has node num:" << i<<std::endl;
    }


    void pubLocalMapPoint(void)
    {
      pcl::PointCloud<MapPoint>::Ptr  map_point_pt (new pcl::PointCloud<MapPoint>);   
      MapPoint p;
      sensor_msgs::PointCloud2 map_point_msg;
      // ????????????????????????????????????????????????
      std::lock_guard<std::mutex> local_map_guard(m_localMapMutex);  //???????????????????????????????????????
      for (auto it = local_map_->begin_leaf(); it != local_map_->end_leaf(); ++it) {
        if ( it.get_node().get_state() == hd_ccsom::State::OCCUPIED|| it.get_node().get_state() == hd_ccsom::State::FREE) {  
          hd_ccsom::point3f p_raw = it.get_loc();
          p.x = p_raw.x();
          p.y = p_raw.y();
          p.z = p_raw.z();
          std::vector<float> scs(num_class_);
          it.get_node().get_ms(scs);

          // ????????????
          p.semantic_label1 = std::distance(scs.begin(), std::max_element(scs.begin(), scs.end()));  //???????????????????????????
          p.prob1 = *std::max_element(scs.begin(), scs.end());
          scs[p.semantic_label1] = 0.0;
          // ???????????????
          p.semantic_label2 = std::distance(scs.begin(), std::max_element(scs.begin(), scs.end()));  //???????????????????????????
          p.prob2 = *std::max_element(scs.begin(), scs.end());
          scs[p.semantic_label2] = 0.0;
          // ???????????????
          p.semantic_label3= std::distance(scs.begin(), std::max_element(scs.begin(), scs.end()));  //???????????????????????????
          p.prob3 = *std::max_element(scs.begin(), scs.end());
          scs[p.semantic_label3] = 0.0;

          std::vector<float> prob(num_class_);
          it.get_node().get_probs(prob);
          p.true_prob = prob[p.semantic_label1];

          std::vector<float> vars(num_class_);
          it.get_node().get_vars(vars);
          p.true_var = vars[p.semantic_label1];

          map_point_pt->points.push_back(p);
        }
      }
      map_point_pt->width = 1;
      map_point_pt->height = map_point_pt->points.size();
      pcl::toROSMsg( *map_point_pt, map_point_msg);  //????????????????????????????????????
      map_point_msg.header.frame_id = "map"; //???id?????????velodyne?????????
      m_localMapPub.publish(map_point_msg); //?????????????????????????????????
      std::cout << name_ << " has finshed share local map, the topic is " << name_+"/map_point" <<std::endl;
      std::cout << "size is " << map_point_pt->points.size() << std::endl;
    }

    // ???????????????????????????
    bool manualTriger(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {
      std::thread PubLocalMapObj(std::bind(&Semanticmulti::pubLocalMapPoint, this));
      if (PubLocalMapObj.joinable())
      {
        PubLocalMapObj.detach();
      }
      return true;
    }

    // ???????????????????????????
    void friendMapCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)  
    {
      std::cout << name_ <<" has recesived friend map"<<std::endl;
      pcl::PointCloud<MapPoint>::Ptr map_point_pt (new pcl::PointCloud<MapPoint>);  
      pcl::fromROSMsg(*cloud_msg, *map_point_pt);   //?????????????????????
      std::lock_guard<std::mutex> global_map_guard(m_globalMapMutex);  //???????????????????????????????????????
      // std::lock_guard<std::mutex> local_map_guard(m_localMapMutex);  //???????????????????????????????????????
      insertFriendMapToGlobal(*map_point_pt);   //???????????????????????????
      std::cout << name_ <<" has finished map fusion for its Gloabl Map" <<std::endl;
    }

    void insertFriendMapToGlobal(pcl::PointCloud<MapPoint> &map_cells)  //??????????????????????????????????????????
    {
      pcl::PointCloud<MapPoint>::Ptr transformed_cells(new pcl::PointCloud<MapPoint>);
      pcl::transformPointCloud(map_cells, *transformed_cells, friendToMe);  //??????????????????????????????
      if (methods_ == 5 || methods_ == 4)
        global_map_->insert_map_point_single(transformed_cells);    //???
      else if (methods_ == 6)
        global_map_->insert_map_point_multi(transformed_cells);    //?????????
       std::thread pubGlobalMapObj(std::bind(&Semanticmulti::pubGlobalMapRviz, this));  //????????????????????????????????????????????????
      if (pubGlobalMapObj.joinable())  //???????????????????????????????????????
      {
          pubGlobalMapObj.detach();
      }
    }

      pcl::PointCloud<pcl::PointXYZL>::Ptr kitti2pcl(std::string fn, std::string fn_label) 
    {
      FILE* fp_label = std::fopen(fn_label.c_str(), "r");
      if (!fp_label) {
        std::perror("File opening failed");
      }
      std::fseek(fp_label, 0L, SEEK_END); // ??????????????????????????????
      std::rewind(fp_label); // ????????????????????????
      FILE* fp = std::fopen(fn.c_str(), "r");
      if (!fp) {
        std::perror("File opening failed");
      }
      std::fseek(fp, 0L, SEEK_END);
      size_t sz = std::ftell(fp); // ????????????
      std::rewind(fp);
      int n_hits = sz / (sizeof(float) * 4);
      pcl::PointCloud<pcl::PointXYZL>::Ptr pc(new pcl::PointCloud<pcl::PointXYZL>);
      for (int i = 0; i < n_hits; i++) {
        pcl::PointXYZL point;
        float intensity;
        std::uint32_t label_int32;
        if (fread(&point.x, sizeof(float), 1, fp) == 0) break;
        if (fread(&point.y, sizeof(float), 1, fp) == 0) break;
        if (fread(&point.z, sizeof(float), 1, fp) == 0) break;
        if (fread(&intensity, sizeof(float), 1, fp) == 0) break;
        if (fread(&label_int32, sizeof(std::uint32_t), 1, fp_label) == 0) break;

        int label_int = label_int32 & 0xFFFF;
        // label?????????????????????1-34???????????????35????????????0???????????????
        std::vector<int>::iterator ite = find(label2label.begin(), label2label.end(), label_int);
        point.label = std::distance(std::begin(label2label), ite) + 1;
        // int num_19 = std::distance(std::begin(label2label), ite);
        // point.label = label3label[num_19] +1;
        // point.label = label_int;
        pc->push_back(point);
      }
      std::fclose(fp);
      std::fclose(fp_label);
      return pc;
    }

    // ???????????????????????????????????????????????????
    bool labelWrite(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {
      std::cout << name_ << " start writing local map label" << std::endl;
      int begin_id ;
      int end_id ;
      std::string methods_name;
      if (sequences_ == "01")
      {
        if (name_ == "/robot1")
        {
          begin_id = 540;   
          end_id = 549;  
        }
        if (name_ == "/robot2")
        {
          begin_id = 550;
          end_id = 559;
        }
      }
      else if (sequences_ == "04")
      {
        if (name_ == "/robot1")
        {
          begin_id = 125;   
          end_id = 134;  
        }
        if (name_ == "/robot2")
        {
          begin_id = 135;
          end_id = 144;
        }
      }
      else if (sequences_ == "05")
      {
        if (name_ == "/robot1")
        { 
          begin_id = 1372;   
          end_id = 1381;  
        }
        if (name_ == "/robot2")
        {
          begin_id = 1382;
          end_id = 1391;
        }
      }
      else if (sequences_ == "07")
      {
        if (name_ == "/robot1")
        {
          begin_id = 539;
          end_id = 548;
        }
        if (name_ == "/robot2")
        {
          begin_id = 549;
          end_id = 558;
        }
      }
      else if (sequences_ == "09")
      {
        if (name_ == "/robot1")
        {
          begin_id = 786;
          end_id = 795;
        }
        if (name_ == "/robot2")
        {
          begin_id = 796;
          end_id = 805;
        }
      }
      if (methods_ == 2 || methods_ == 4)
        methods_name = "S-CSM-Local";
      else if (methods_ == 3 || methods_ == 5 || methods_ == 6)
        methods_name = "HD-CSM-Local";
      for (int i=0, id = begin_id;  id<= end_id; id++, i++)
      {
        char scan_id_c[256];
        sprintf(scan_id_c, "%06d", id);
        std::string scan_name = "/media/dyn/DYN/Research/dataset/SemanticKITTI/dataset/sequences/"+ sequences_ +"/velodyne/" + std::string(scan_id_c) + ".bin";
        std::string gt_name = "/media/dyn/DYN/Research/dataset/SemanticKITTI/dataset/sequences/"+ sequences_ +"/predictions/" + std::string(scan_id_c) + ".label";
        std::string mylabel_name = "/home/dyn/catkin_ws/src/HD-CCSOM/semantickitti_bag_result/"+methods_name+"/sequences/"+ sequences_ +"/predictions/" + std::string(scan_id_c) + ".label";
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud = kitti2pcl(scan_name, gt_name);
  
        Eigen::Matrix4f calibration;
        if (sequences_ == "01")
        {
              calibration <<  4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
              -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
                9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
                        0                ,  0                ,  0                ,  1.000000000000000;
        }
        else if (sequences_ == "04")
        {
            calibration <<  -0.001857739385241, -0.999965951350955, -0.008039975204516, -0.004784029760483,
                      -0.006481465826011,  0.008051860151134, -0.999946608177406, -0.073374294642306,
                      0.999977309828677, -0.001805528627661, -0.006496203536139, -0.333996806443304,
                      0                ,  0                ,  0                ,  1.000000000000000;
        }
        else if (sequences_ == "05")
        {
            calibration <<  -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
             -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
              9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
                      0                ,  0                ,  0                ,  1.000000000000000;
        }
        else if (sequences_ == "07")
        {
               calibration <<
                         -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
                          -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02, 
                          9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
       	                 0                ,  0                ,  0                ,  1.000000000000000;
        }
        else if (sequences_ == "09")
        {
               calibration <<
                         -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
                          -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
                           9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
       	                 0                ,  0                ,  0                ,  1.000000000000000;
        }
        Eigen::Matrix4f transform = lidar_poses_[id];
        Eigen::Matrix4f new_transform = init_trans_to_ground_ * transform * calibration;
        pcl::transformPointCloud (*cloud, *cloud, new_transform);
        FILE* fp_label = std::fopen(mylabel_name.c_str(), "w");
        // ????????????txt????????????
        // std::ofstream result_file("/home/dyn/catkin_ws/src/HD-CCSOM/semantickitti_bag_result/txt/"+std::string(scan_id_c) + ".txt");
        for (int i = 0; i < cloud->points.size(); ++i) 
        {
          // ????????????????????????
          hd_ccsom::SemanticOcTreeNode node = local_map_->search(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
          int pred_label ;
          int pred_label_raw;
          pred_label = node.get_semantics();
          if (pred_label == 0)
            pred_label = cloud->points[i].label;
          pred_label_raw = label2label[pred_label -1] ;
          std::uint32_t my_label = (std::uint32_t)pred_label_raw;
          // ????????????????????????????????????
          // result_file << cloud->points[i].label - 1  << " " << pred_label - 1 << "\n";
          if (fwrite(&my_label, sizeof(std::uint32_t), 1, fp_label) == 0) break;
        }
        // result_file.close();
        std::fclose(fp_label);
      }
      std::cout << name_ << " finish wirting local map label" << std::endl;
    }

     // ???????????????????????????????????????????????????
    bool labelWriteglobal(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {
      std::cout << name_ << " start writing global map label" << std::endl;
      int begin_id ;
      int end_id ;
      std::string methods_name;
      if (sequences_ == "01")
      {
          begin_id = 540;
          end_id = 559;   
      }
      else if (sequences_ == "04")
      {
          begin_id = 125;
          end_id = 144;   
      }
      else if (sequences_ == "05")
      {
          begin_id = 1372;
          end_id = 1391;   
      }
      else if (sequences_ == "07")
      {
          begin_id = 539;
          end_id = 558;
      }
      else if (sequences_ == "09")
      {
          begin_id = 786;
          end_id = 805;
      }
      if (methods_ == 4 )
        methods_name = "S-CSM-Single";
      else if (methods_ == 5 )
        methods_name = "HD-CSM-Single";
      else if (methods_ == 6 )
        methods_name = "HD-CSM";

      for (int i=0, id = begin_id;  id<= end_id; id++, i++)
      {
        char scan_id_c[256];
        sprintf(scan_id_c, "%06d", id);
        std::string scan_name = "/media/dyn/DYN/Research/dataset/SemanticKITTI/dataset/sequences/"+ sequences_ +"/velodyne/" + std::string(scan_id_c) + ".bin";
        std::string gt_name = "/media/dyn/DYN/Research/dataset/SemanticKITTI/dataset/sequences/"+ sequences_ +"/predictions/" + std::string(scan_id_c) + ".label";
        std::string mylabel_name = "/home/dyn/catkin_ws/src/HD-CCSOM/semantickitti_bag_result/"+methods_name+"/sequences/"+ sequences_ +"/predictions/" + std::string(scan_id_c) + ".label";
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud = kitti2pcl(scan_name, gt_name);
  
        Eigen::Matrix4f calibration;
        if (sequences_ == "01")
        {
              calibration <<  4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
              -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
                9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
                        0                ,  0                ,  0                ,  1.000000000000000;
        }
        else if (sequences_ == "04")
        {
            calibration <<  -0.001857739385241, -0.999965951350955, -0.008039975204516, -0.004784029760483,
                      -0.006481465826011,  0.008051860151134, -0.999946608177406, -0.073374294642306,
                      0.999977309828677, -0.001805528627661, -0.006496203536139, -0.333996806443304,
                      0                ,  0                ,  0                ,  1.000000000000000;
        }
        else if (sequences_ == "05")
        {
            calibration <<  -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
             -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
              9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
                      0                ,  0                ,  0                ,  1.000000000000000;
        }
        else if (sequences_ == "07")
        {
               calibration <<
                         -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
                          -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02, 
                          9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
       	                 0                ,  0                ,  0                ,  1.000000000000000;
        }
        else if (sequences_ == "09")
        {
               calibration <<
                         -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
                          -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
                           9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
       	                 0                ,  0                ,  0                ,  1.000000000000000;
        }
        Eigen::Matrix4f transform = lidar_poses_[id];
        Eigen::Matrix4f new_transform = init_trans_to_ground_ * transform * calibration;
        pcl::transformPointCloud (*cloud, *cloud, new_transform);
        // pcl::transformPointCloud (*cloud, *cloud, transform);

        FILE* fp_label = std::fopen(mylabel_name.c_str(), "w");
        // ????????????txt????????????
        // std::ofstream result_file("/home/dyn/catkin_ws/src/HD-CCSOM/semantickitti_bag_result/txt/"+std::string(scan_id_c) + ".txt");
        for (int i = 0; i < cloud->points.size(); ++i) 
        {
          // ????????????????????????
          hd_ccsom::SemanticOcTreeNode node = local_map_->search(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
          int pred_label ;
          int pred_label_raw;
          pred_label = node.get_semantics();
          if (pred_label == 0)
            pred_label = cloud->points[i].label;
          pred_label_raw = label2label[pred_label -1] ;
          // if (pred_label_raw < 0)
          //   pred_label_raw = 0;
          std::uint32_t my_label = (std::uint32_t)pred_label_raw;
          // ????????????????????????????????????
          // result_file << cloud->points[i].label - 1  << " " << pred_label - 1 << "\n";
          if (fwrite(&my_label, sizeof(std::uint32_t), 1, fp_label) == 0) break;
        }
        // result_file.close();
        std::fclose(fp_label);
      }
      std::cout << name_ << " finish wirting global map label" << std::endl;
    }

    // ?????????????????????????????????????????????
    bool read_lidar_poses(const std::string lidar_pose_name) {
      if (std::ifstream(lidar_pose_name)) {
        std::ifstream fPoses;
        fPoses.open(lidar_pose_name.c_str());
        while (!fPoses.eof()) {
          std::string s;
          std::getline(fPoses, s);
          if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            Eigen::Matrix4f t_matrix = Eigen::Matrix4f::Identity();
            for (int i = 0; i < 3; ++i)
              for (int j = 0; j < 4; ++j)
                ss >> t_matrix(i, j);
            lidar_poses_.push_back(t_matrix);
          }
        }
        ROS_INFO ("FINISH READ POSES");
        fPoses.close();
        return true;
        } else {
         ROS_ERROR_STREAM("Cannot open evaluation list file " << lidar_pose_name);
         return false;
      }
    } 
    
    void startsemantic(std::string input_data_dir, std::string input_label_dir)
    {
      hd_ccsom::point3f origin;
      int begin_id; // ??????????????????
      int end_id; // ??????????????????
      if (sequences_ == "01")
      {
        if (name_ == "/robot1")
        {
          begin_id = 540;   
          end_id = 549;  
        }
        if (name_ == "/robot2")
        {
          begin_id = 550;
          end_id = 559;
        }
      }
      else if (sequences_ == "04")
      {
        if (name_ == "/robot1")
        {
          begin_id = 125;   
          end_id = 134;  
        }
        if (name_ == "/robot2")
        {
          begin_id = 135;
          end_id = 144;
        }
      }
      else if (sequences_ == "05")
      {
        if (name_ == "/robot1")
        { 
          begin_id = 1372;   
          end_id = 1381;  
        }
        if (name_ == "/robot2")
        {
          begin_id = 1382;
          end_id = 1391;
        }
      }
      else if (sequences_ == "07")
      {
        if (name_ == "/robot1")
        {
          begin_id = 539;
          end_id = 548;
        }
        if (name_ == "/robot2")
        {
          begin_id = 549;
          end_id = 558;
        }
      }
      else if (sequences_ == "09")
      {
        if (name_ == "/robot1")
        {
          begin_id = 786;
          end_id = 795;
        }
        if (name_ == "/robot2")
        {
          begin_id = 796;
          end_id = 805;
        }
      }
      for (int scan_id  = begin_id; scan_id <= end_id; ++scan_id) 
      {
        char scan_id_c[256];
        sprintf(scan_id_c, "%06d", scan_id);
        std::string scan_name = input_data_dir + std::string(scan_id_c) + ".bin";
        std::string label_name = input_label_dir + std::string(scan_id_c) + ".label";
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud = kitti2pcl(scan_name, label_name);
        Eigen::Matrix4f transform = lidar_poses_[scan_id];
        Eigen::Matrix4f calibration;
        if (sequences_ == "01")
        {
              calibration <<  4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
              -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
                9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
                        0                ,  0                ,  0                ,  1.000000000000000;
        }
        else if (sequences_ == "04")
        {
            calibration <<  -0.001857739385241, -0.999965951350955, -0.008039975204516, -0.004784029760483,
                      -0.006481465826011,  0.008051860151134, -0.999946608177406, -0.073374294642306,
                      0.999977309828677, -0.001805528627661, -0.006496203536139, -0.333996806443304,
                      0                ,  0                ,  0                ,  1.000000000000000;
        }
        else if (sequences_ == "05")
        {
            calibration <<  -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
             -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
              9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
                      0                ,  0                ,  0                ,  1.000000000000000;
        }
        else if (sequences_ == "07")
        {
               calibration <<
                         -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
                          -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02, 
                          9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
       	                 0                ,  0                ,  0                ,  1.000000000000000;
        }
        else if (sequences_ == "09")
        {
               calibration <<
                         -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
                          -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
                           9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
       	                 0                ,  0                ,  0                ,  1.000000000000000;
        }
        Eigen::Matrix4f new_transform = init_trans_to_ground_ * transform * calibration;
        // std::cout << new_transform << std::endl;
        pcl::transformPointCloud (*cloud, *cloud, new_transform);
        origin.x() = transform(0, 3);
        origin.y() = transform(1, 3);
        origin.z() = transform(2, 3);
        if (methods_ == 2 || methods_ == 4)
          local_map_->insert_pointcloud_csm(*cloud, origin, ds_resolution_, free_resolution_, max_range_); 
        else
          local_map_->insert_pointcloud_test(*cloud, origin, ds_resolution_, free_resolution_, max_range_); 
        std::cout << name_ << "   Inserted point cloud at " << scan_name << std::endl;
      }
    }


  private:
    ros::NodeHandle nh_;
    double resolution_;
    double ds_resolution_;
    double free_resolution_;
    double max_range_;
    int num_class_;
    std::string name_;
    std::string frientname_;
    std::string sequences_;
    int color_num_;
    int methods_;
    hd_ccsom::SemanticBKIOctoMap* local_map_;
    hd_ccsom::SemanticBKIOctoMap* global_map_;
    hd_ccsom::MarkerArrayPub* m_local_pub_;
    hd_ccsom::MarkerArrayPub* m_global_pub_;

    ros::Publisher m_localMapPub;  //????????????????????????
    ros::Subscriber m_friendMapSub;   //??????????????????
    ros::Subscriber m_pointCloudSub_kitti_no_tf;

    // std::vector<float> label_vec;   // ?????????????????????float??????
    std::vector<int> label2label;  // semantickitti???????????????
    std::vector<int> label3label;  // ??????21????????????
    std::vector<uint32_t> labelcolor;  // ?????????????????????uint32_t??????
    int count_pc;
    // ros::Publisher m_globalcloudPub;  //????????????????????????
    message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub_kitti;  //????????????
    message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub_senior;  //????????????
    message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub_senior_2;  //????????????
    message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub_gazebo;  //????????????
    tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub_kitti;  //??????/tf?????????????????????????????????????????????tf?????????
    tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub_senior;  //??????/tf?????????????????????????????????????????????tf?????????
    tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub_senior_2;  //??????/tf?????????????????????????????????????????????tf?????????
    tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub_gazebo;  //??????/tf?????????????????????????????????????????????tf?????????
    tf::TransformListener m_tfListener;  // ???????????????

    ros::ServiceServer m_manualTrigerSrv;   //??????????????????
    ros::ServiceServer m_label_write;   //??????????????????????????????????????????????????????label
    ros::ServiceServer m_label_write_global;   //???????????????????????????????????????????????????label

    std::mutex m_globalMapMutex;    //?????????????????????????????????????????????????????????????????????
    std::mutex m_localMapMutex;  //?????????????????????

    Eigen::Matrix4f friendToMe;   //???????????????????????????
    Eigen::Matrix4f init_trans_to_ground_;

    ros::Timer m_globalMapViewerTimer;   //?????????????????????
    ros::Timer m_localMapViewerTimer;   //?????????????????????
    clock_t start_time_;

    std::vector<Eigen::Matrix4f> lidar_poses_; // ??????????????????????????????????????????????????????

};
