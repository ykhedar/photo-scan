#ifndef PHOTO_SCAN_H
#define PHOTO_SCAN_H

#include <unistd.h>
#include <boost/filesystem.hpp>
#include <stdlib.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include "pcl_ros/point_cloud.h"

#include <opencv2/highgui/highgui.hpp>

#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <custom_msgs/StitchedImageCompressed.h>
#include <custom_msgs/PhotoScan.h>

#include <sensor_msgs/PointCloud2.h>
#include "gdal.h"
#include <ogr_spatialref.h>

using PCLPtCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>;

struct GTiffData
{
  double center_north;
  double center_east;
  double alt;
  uint8_t zone;
  char band;
  double heading;
  double scale;
};

class PhotoScan
{
  private:

    bool m_initialised = true;
    bool m_pub_once = false;

    GTiffData m_GTiff_data;

    std::string m_img_save_loc;
    std::string m_node_src_loc;
    std::string m_PS_auto_script_loc;
    std::string m_PS_start_script_loc;
    std::string m_PS_start_cmd;
    std::string m_ir_img_save_loc;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh_;
    ros::ServiceServer  m_start_service;
    ros::Subscriber     m_PS_info_subscriber;
    ros::Subscriber     m_ir_PS_info_subscriber;
    ros::Publisher      m_stiched_img_pub;
    ros::Publisher      m_pt_cloud_pub;

    sensor_msgs::PointCloud2 m_ros_pt_cloud;
    PCLPtCloud::Ptr m_PCL_pt_cloud_ptr;
    custom_msgs::StitchedImageCompressed m_ros_stitched_img;
    cv::Mat m_cv_stitched_img;

    void photoScanCb(const std_msgs::String::ConstPtr& msg);
    void irphotoScanCb(const std_msgs::String::ConstPtr& msg);
    bool startPhotoScanProcess(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void pubStitchedImage();
    void pubPointCloud();
    bool readResults();
    void runPhotoScan();

  public:
    PhotoScan();
    ~PhotoScan(){}
    void Publish();
};

#include "photo_scan_helper.h"

#endif // PHOTO_SCAN_H
