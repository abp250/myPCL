#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>); 

void save_pcd (int num)
{
  char buffer [50];
  sprintf(buffer, "pcd_saves/pcd_output_%d.pcd", num);
  pcl::io::savePCDFileASCII (buffer, *save_cloud);
  std::cerr << "Saved " << (*save_cloud).points.size () << " data points to pcd_output_" << num << ".pcd" << std::endl;
}

 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
     {
       if (!viewer.wasStopped()){
	pcl::copyPointCloud<pcl::PointXYZRGBA>(*cloud, *save_cloud); 
	viewer.showCloud (cloud);
	}
     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function <void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback (f);

       interface->start ();
       int inc = 0;
       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (5));
	 save_pcd (inc);
	 inc++;
       }
       interface->stop ();
     }
     pcl::visualization::CloudViewer viewer;
 };

 int main ()
 {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }


