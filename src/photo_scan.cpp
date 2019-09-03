#include "../include/photo_scan/photo_scan.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "photo_scan_node");
    PhotoScan lPhotoScan;
    ros::Rate rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        lPhotoScan.Publish();
        rate.sleep();
    }
    return 0;
}

PhotoScan::PhotoScan(): local_nh_("~"), m_PCL_pt_cloud_ptr(new PCLPtCloud)
{
  //- Reading Node Parameters from Launch file
  std::string image_save_loc_topic, photo_scan_start_srv, stitched_image_topic, point_cloud_topic,photo_scan_start_sub_topic;
  std::string ir_photo_scan_start_sub_topic;
  local_nh_.param("image_save_loc_topic", image_save_loc_topic, std::string("uninitialised"));
  local_nh_.param("ir_image_save_loc_topic", image_save_loc_topic, std::string("uninitialised"));
  local_nh_.param("photo_scan_start_srv", photo_scan_start_srv, std::string("uninitialised"));
  local_nh_.param("stitched_image_topic", stitched_image_topic, std::string("uninitialised"));
  local_nh_.param("point_cloud_topic", point_cloud_topic, std::string("uninitialised"));
  local_nh_.param("photo_scan_start_sub_topic", photo_scan_start_sub_topic, std::string("uninitialised"));
  local_nh_.param("ir_photo_scan_start_sub_topic", ir_photo_scan_start_sub_topic, std::string("uninitialised"));

  //- Node services,publishers,subscribers initialisation
  m_start_service     	= nh.advertiseService(photo_scan_start_srv, &PhotoScan::startPhotoScanProcess, this);
  m_PS_info_subscriber  = nh.subscribe(photo_scan_start_sub_topic,10,&PhotoScan::photoScanCb, this);
  m_ir_PS_info_subscriber  = nh.subscribe(ir_photo_scan_start_sub_topic,10,&PhotoScan::irphotoScanCb, this);
    // The messages are latched, so will only be published once
  m_stiched_img_pub 	= nh.advertise<custom_msgs::StitchedImageCompressed>(stitched_image_topic, 5,true);
  m_pt_cloud_pub   		= nh.advertise<PCLPtCloud>(point_cloud_topic, 5,true);
  m_node_src_loc   		= ros::package::getPath("photo_scan");

  ROS_INFO_STREAM("\n" << "Info of PhotoScan Node    :" <<                             "\n" \
  				  "Input images for PS location topic:" << image_save_loc_topic    	<< "\n" \
  				  "Name of PS start service          :" << photo_scan_start_srv     << "\n" \
  				  "PS Stitched Image publish topic   :" << stitched_image_topic 	<< "\n" \
  				  "PS pt Cloud publish topic         :" << point_cloud_topic      	<< "\n" \
  				  "PS start info subscribe topic     :" << photo_scan_start_sub_topic);

  //- Initialise the locations/names of PS related scripts.
  m_PS_auto_script_loc       = m_node_src_loc + "/scripts/multispectral.py";
  m_PS_start_script_loc      = "~/metashape-pro/metashape.sh";
  m_PS_start_cmd = "sh " + m_PS_start_script_loc + " -r " + m_PS_auto_script_loc + " " + m_img_save_loc + " " + m_ir_img_save_loc;
}

bool PhotoScan::startPhotoScanProcess(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_INFO("Received start saving request");
  if (m_img_save_loc == "" || m_ir_img_save_loc == "")
  {
    ROS_ERROR("Cannot start photoscan. m_img_save_loc is not initialised.");
    res.success = false;
    return false;
  }
  res.success = true;
  m_PS_start_cmd = "sh " + m_PS_start_script_loc + " -r " + m_PS_auto_script_loc + " " + m_img_save_loc + " " + m_ir_img_save_loc;
  ROS_WARN_STREAM("Running command for PS: " << m_PS_start_cmd);
  runPhotoScan();
  return true;
}

void PhotoScan::photoScanCb(const std_msgs::String::ConstPtr& msg)
{
  m_img_save_loc = msg->data;
  ROS_INFO_STREAM_THROTTLE(15, "Save Location is: " << m_img_save_loc);
}

void PhotoScan::irphotoScanCb(const std_msgs::String::ConstPtr& msg)
{
    m_ir_img_save_loc = msg->data;
    ROS_INFO_STREAM_THROTTLE(15, "Save Location is: " << m_ir_img_save_loc);
}

void PhotoScan::runPhotoScan()
{
  ROS_INFO_STREAM("Image Save Location is: " << m_img_save_loc.c_str() );
  system(m_PS_start_cmd.c_str());
  m_initialised= false;
}

void PhotoScan::pubStitchedImage()
{
    if (m_stiched_img_pub.getNumSubscribers() == 0) { return; }

    std_msgs::Header header;
    header.frame_id = "utm";
    header.stamp = ros::Time::now();

    geodesy::UTMPoint utm(m_GTiff_data.center_east, m_GTiff_data.center_north, m_GTiff_data.alt, m_GTiff_data.zone, m_GTiff_data.band);
    geographic_msgs::GeoPoint wgs = geodesy::toMsg(utm);
    if (!m_cv_stitched_img.data)
    {
      ROS_ERROR("GeoTiff Image read has no data to publish. Check it in the PSResults directory.");
      return;
    }

    m_ros_stitched_img.imagedata = *cv_bridge::CvImage(header, "bgra8", m_cv_stitched_img).toCompressedImageMsg(cv_bridge::PNG);

    m_ros_stitched_img.gpsdata.latitude = wgs.latitude;
  	m_ros_stitched_img.gpsdata.longitude = wgs.longitude;
  	m_ros_stitched_img.gpsdata.altitude = m_GTiff_data.alt;

  	m_ros_stitched_img.orientation.w = 1.0;
  	m_ros_stitched_img.orientation.x = 0.0;
  	m_ros_stitched_img.orientation.y = 0.0;
  	m_ros_stitched_img.orientation.z = 0.0;

  	m_ros_stitched_img.scale = m_GTiff_data.scale;

  	//ROS_INFO_STREAM_THROTTLE(10,"Geo Info from GeoTiff: " << wgs.latitude << ", " << wgs.longitude << "; Alt: " << m_GTiff_data.alt);
    //ROS_INFO_STREAM_THROTTLE(10,"OutGeo: " << m_GTiff_data.center_north << ", " << m_GTiff_data.center_east << "; Alt: " << m_GTiff_data.heading);

    m_stiched_img_pub.publish(m_ros_stitched_img);
    ROS_INFO_THROTTLE(10,"Publishing GTiff Image. Note that this message prints every 10secs.");
}

void PhotoScan::pubPointCloud()
{
  if (m_pt_cloud_pub.getNumSubscribers() == 0) { return; }

  pcl::toROSMsg(*m_PCL_pt_cloud_ptr, m_ros_pt_cloud);
  m_ros_pt_cloud.header.frame_id = "utm";
  m_pt_cloud_pub.publish(m_ros_pt_cloud);
  ROS_INFO_THROTTLE(10,"Publishing Point cloud. Note that this message prints every 10secs.");
}

bool PhotoScan::readResults()
{
  if (!m_initialised)
  {
    bool lPtCldFileExists( boost::filesystem::exists( m_img_save_loc + "/PSResults/pw_ascii_low.ply" ) );
    bool lStitchImgExists( boost::filesystem::exists( m_img_save_loc + "/PSResults/ortho_low.jpg" ) );
    if (lPtCldFileExists && lStitchImgExists)
    {
      readGeoTiff(m_img_save_loc, m_cv_stitched_img, m_GTiff_data);
      readPCLFromFile(m_img_save_loc, *m_PCL_pt_cloud_ptr);
      m_initialised = true;
      ROS_INFO("PhotoScan result files loaded successfully for publishing.");
      return true;
    }
    ROS_ERROR_THROTTLE(10,"PhotoScan result files for publishing do not exist yet. Waiting... Could you check if PS has finished?");
    return false;
  }
  return true;
}

void PhotoScan::Publish()
{
  if (readResults() && !m_pub_once)
  {
    //pubPointCloud();
    pubStitchedImage();
    m_pub_once = true;
  }
}
