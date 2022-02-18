#include <string>
#include <iostream>
#include <ros/ros.h>

#include "bkioctomap.h"
#include "markerarray_pub.h"
#include "multi_senior_util.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_senior_node");
    ros::NodeHandle nh("~");

    int block_depth = 4;
    double sf2 = 1.0;
    double ell = 1.0;
    float prior = 1.0f;
    float var_thresh = 1.0f;
    double free_thresh = 0.3;
    double occupied_thresh = 0.7;
    double resolution = 0.1;
    int num_class = 2;
    double free_resolution = 0.5;
    double ds_resolution = 0.1;
    int scan_num = 0;
    double max_range = -1;
    int methods = 6;
    

    bool query = false;
    bool visualize = false;


    std::string name, frientname, cloudTP, cloudTP_gazebo, cloudTP_sk, sequences;
    int color_num = 3;

    nh.param<std::string>("name", name, std::string("/robot1"));
    nh.param<std::string>("frientname",  frientname, std::string("/robot2"));
    nh.param<std::string>("cloudTP",  cloudTP, std::string("/seg_cloud"));
    nh.param<std::string>("cloudTP_gazebo",  cloudTP_gazebo, std::string("/gazebo/seg_cloud"));
    nh.param<std::string>("cloudTP_sk",  cloudTP_sk, std::string("/velodyne_points"));
    nh.param<std::string>("sequences",  sequences, std::string("07"));
    nh.param<int>("methods",  methods, methods);
    nh.param<int>("color_num", color_num, color_num);
    nh.param<int>("block_depth", block_depth, block_depth);
    nh.param<double>("sf2", sf2, sf2);
    nh.param<double>("ell", ell, ell);
    nh.param<float>("prior", prior, prior);
    nh.param<float>("var_thresh", var_thresh, var_thresh);
    nh.param<double>("free_thresh", free_thresh, free_thresh);
    nh.param<double>("occupied_thresh", occupied_thresh, occupied_thresh);
    nh.param<double>("resolution", resolution, resolution);
    nh.param<int>("num_class", num_class, num_class);
    nh.param<double>("free_resolution", free_resolution, free_resolution);
    nh.param<double>("ds_resolution", ds_resolution, ds_resolution);
    nh.param<int>("scan_num", scan_num, scan_num);
    nh.param<double>("max_range", max_range, max_range);
    nh.param<bool>("query", query, query);
    nh.param<bool>("visualize", visualize, visualize);


    ROS_INFO_STREAM(name<<" Parameters:" << std::endl <<
      "block_depth: " << block_depth << std::endl <<
      "sf2: " << sf2 << std::endl <<
      "ell: " << ell << std::endl <<
      "prior:" << prior << std::endl <<
      "var_thresh: " << var_thresh << std::endl <<
      "free_thresh: " << free_thresh << std::endl <<
      "occupied_thresh: " << occupied_thresh << std::endl <<
      "resolution: " << resolution << std::endl <<
      "num_class: " << num_class << std::endl << 
      "free_resolution: " << free_resolution << std::endl <<
      "ds_resolution: " << ds_resolution << std::endl <<
      "scan_num: " << scan_num << std::endl <<
      "max_range: " << max_range << std::endl <<
      "query: " << query << std::endl <<
      "visualize:" << visualize
      );

    
    // ///////// Build Map /////////////////////
    std::string dir = "/media/dyn/DYN/Research/dataset/SemanticKITTI/dataset/sequences/";
    Semanticmulti semantic_multi(nh, resolution, block_depth, sf2, ell, num_class, free_thresh, occupied_thresh, var_thresh, 
                                                                      ds_resolution, free_resolution, max_range, prior, name,  frientname, cloudTP, cloudTP_gazebo, cloudTP_sk, sequences, color_num, methods);
    
    // 只有semnatickitti定量分析需要
    if ( color_num  == 3)
    {
      semantic_multi.read_lidar_poses(dir + sequences+"/poses.txt");
      semantic_multi.startsemantic(dir + sequences+"/velodyne/",  dir + sequences+"/predictions/" );
    }
    ros::spin();
    return 0;
}
