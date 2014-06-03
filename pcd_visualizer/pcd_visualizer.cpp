/* \author Aaron Pederson */


#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


boost::shared_ptr<pcl::visualization::PCLVisualizer> pcdVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();
  viewer->setBackgroundColor (1.0, 1.0, 1.0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  viewer->addCoordinateSystem (1.0, "global");

  return (viewer);
}

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
  for(int k = 1; k < argc; k++)
  {

  // ----------------------------------------------------------------
  // -----Read point cloud from PCD and put in memory ---------------
  // ----------------------------------------------------------------
  //could be updated for multiple files
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "reading from pcd file: " << argv[k] << "\n\n" std::endl ;
  pcl::io::loadPCDFile (argv[k], *pcd_cloud_ptr);

  
 
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = pcdVis(pcd_cloud_ptr);
  //--------------------
  // -----Main loop-----
  //--------------------
  
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (10000));
  }
  
}
}

