#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <math.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 100;
  cloud.height   = 100;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);
  double r = 50.0;
size_t k = 0;
  for (double i = 0.0; i <= 2.0 * M_PI; i = i + (2.0 * M_PI)/(sqrt( cloud.points.size () )+1) )   // i = theta, j = phi
  {
	for (double j = 0.0; j < M_PI; j = j + (M_PI)/sqrt( cloud.points.size() ) )   // i = theta, j = phi
  	{
	
    		cloud.points[k].x = r * cos(i) * sin(j);
    		cloud.points[k].y = r * sin(i) * sin(j);
    		cloud.points[k].z = r * cos(j);
		k++;
  	}
  }
  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}

