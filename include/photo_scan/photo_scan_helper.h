#ifndef PHOTO_SCAN_HELPER_H
#define PHOTO_SCAN_HELPER_H

//- DECLARATIONS

//- Reads a point cloud file from PhotoScan into PCL Pt cloud object
inline void readPCLFromFile( const std::string& afilePath, PCLPtCloud &acloud );

//- Reads a GTiff file and returns important data in GTiffData structure
GTiffData GetGtiffData( const std::string& image_path );

//- Reads a GTiff file and loads data into cv::Mat image and GTiffData structure
inline void readGeoTiff(const std::string& afilePath, cv::Mat& mStitchimg, GTiffData& mGTiffData );

//- DEFINITIONS
inline void readPCLFromFile( const std::string& afilePath, PCLPtCloud &acloud )
{
  std::string plyFile_path(afilePath+"/PSResults/pw_ascii_low.ply");
  if ( pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>(plyFile_path, acloud) == -1 ) //* load the file
  {
    ROS_INFO("Couldn't read file ply file");
    return;
  }
  ROS_INFO_STREAM("Loaded " << acloud.width * acloud.height << " data points with the following fields: ");
}      

inline void readGeoTiff(const std::string& afilePath, cv::Mat& mStitchimg, GTiffData& mGTiffData )
{
  std::string image_path(afilePath+"/PSResults/ortho_low.tif");
  ROS_INFO_STREAM("Trying to read GTiff image: " << image_path);
  mGTiffData = GetGtiffData(image_path);
  std::string image_path2(afilePath+"/PSResults/ortho_low.jpg");
  mStitchimg = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
  if (! mStitchimg.data)
  {
    ROS_INFO_STREAM("Image has no data to publish in ReadGTif." << image_path);
  }
  ROS_INFO_STREAM("Image has data. in ReadGTif");
}

GTiffData GetGtiffData( const std::string& image_path )
{
  // easting --- from GTiff
  // northing --- from GTiff
  // altitude  --- ? Probably not required
  // heading --- Do we need?
  // zone --- From GTiff
  // band --- From GTiff
  // scale --- From GTiff
  // orientation --- ?
  GTiffData gTiffData;
  GDALDatasetH  stitchdimg;
  GDALAllRegister();
  
  stitchdimg = GDALOpen( image_path.c_str(), GA_ReadOnly );
  if( stitchdimg == NULL )
  {
    ROS_INFO_STREAM("Could not open GeoTiff file in GetGtiffData() function.");
  }
  ROS_INFO_STREAM("GDAL was able to open the Gtiff  for processing.");

  OGRSpatialReference oSRS(GDALGetProjectionRef(stitchdimg));

  uint8_t x = GDALGetRasterXSize(stitchdimg)/2.0;
  uint8_t y = GDALGetRasterYSize(stitchdimg)/2.0;

  double geoTransform[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double dfGeoX = 0.0;
  double dfGeoY = 0.0;

  if( GDALGetGeoTransform( stitchdimg, geoTransform ) != 0 )
  {
   ROS_INFO_STREAM("Could not get geoTransform from the Raster.");
  }
  dfGeoX = geoTransform[0] + geoTransform[1] * x + geoTransform[2] * y;
  dfGeoY = geoTransform[3] + geoTransform[4] * x + geoTransform[5] * y;
  gTiffData.scale = geoTransform[1]; // Or geoTransform[4] ?
  //std::cout << "Scale is " << gTiffData.scale << std::endl; 

  int tmpBand(-2);
  uint8_t zone = oSRS.GetUTMZone(&tmpBand);
  char band='x';
  if      (tmpBand==1) { band = 'N'; }
  else if (tmpBand==0) { band = 'S'; }
  else                 { std::cout << "Band undefined." << std::endl; } 

  ROS_INFO_STREAM("Coordinates of center points are:" << dfGeoX << " x " << dfGeoY);
  ROS_INFO_STREAM("UTM Zone is:" << zone << " and band is:" << tmpBand);
  
  gTiffData.center_east = dfGeoX;
  gTiffData.center_north= dfGeoY;
  gTiffData.zone = zone;
  gTiffData.band = band;
  gTiffData.alt  = 0;   // TODO Change it to generalise.

  return gTiffData;
}

#endif // PHOTO_SCAN_HELPER_H
