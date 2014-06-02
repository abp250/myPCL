#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv)
{
std::vector < pcl::PointCloud <pcl::PointXYZRGBA> > cloud ( (argc-1), pcl::PointCloud<pcl::PointXYZRGBA>() );

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGBA>);

for(int i = 0; i < argc-1; i++)
{
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  (new pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);
	pcl::io::loadPCDFile (argv[i+1], cloud.at(i));
	(*output).resize(cloud.at(i).size() + (*output).size());
	(*output) += cloud.at(i);
	//pcl::concatenateFields (*output, cloud.at(i), *output);
}
pcl::io::savePCDFileASCII ("output.pcd", *output);

//for (size_t i = 0; i < (*output).points.size (); ++i)
//std::cerr << "    " <<(*output).points[i].x << " " << (*output).points[i].y << " " <<(*output).points[i].z << " " << std::endl;
return (0);
}
