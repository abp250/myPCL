/* \author Aaron Pederson */


#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-m           Visualize File ( ./pcl_visualizer -m ${filename}.pcd \n"
	    << "-n           Visualize File(s) with normals (first argument as radius)\n"
            << "\n\n";
}

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

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals, double radius)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();
 // int v1(0);
 // viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0 /*,v1*/);
 // viewer->addText("Radius:", radius , 10, 10, "cloud" /*,v1*/);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud"/*, v1*/);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  viewer->addCoordinateSystem (1.0, "global");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals"/*, v1*/);

  return (viewer);
}




unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0, "global");

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

  return (viewer);
}


// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  double radius = 0.1;
  int files = 0;
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  bool normals(false), display_pcd(false);
	 if (pcl::console::find_argument (argc, argv, "-n") >= 0)
	  {
	    normals = true;
	    std::cout << "display pcd files with normals \n";
	    radius = (double)atoi(argv[ argc - 1 ]);
	    std::cout << "setting radius to: " << argv[argc - 1] << "\n";

	    		for(int i = 2; i < argc - 1; i++){
			std::cout << argv[i] << "\n";
			files++;
			}

	  }
	  else if(pcl::console::find_argument (argc, argv, "-m") >= 0)
          {
	  	display_pcd = true;
		std::cout << "Display pcd files\n";
			for(int i = 2; i < argc; i++){
			std::cout << argv[i] << "\n";
			files++;
			}
	  }
	  else
  	  {
   	    printUsage (argv[0]);
   	    return 0;
  	  }

  // ----------------------------------------------------------------
  // -----Read point cloud from PCD and put in memory ---------------
  // ----------------------------------------------------------------
  //could be updated for multiple files
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "reading from pcd file.\n\n";
  pcl::io::loadPCDFile (argv[2], *pcd_cloud_ptr);

  
  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius ----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (pcd_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (radius);
  ne.compute (*cloud_normals);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if (display_pcd)
  {
    viewer = pcdVis(pcd_cloud_ptr);
  }
  else if (normals)
  {
    viewer = normalsVis(pcd_cloud_ptr, cloud_normals, radius);
  }
  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (10000));
  }
}

