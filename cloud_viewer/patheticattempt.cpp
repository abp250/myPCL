#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <sstream>
#include <fstream>
//#include <string>
#include <stdexcept>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
    
int user_data;


class my_CV
{
      void CV_func (int num, char* filename){
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    	pcl::io::loadPCDFile (filename, *cloud);
 	pcl::visualization::CloudViewer CV("Cloud Viewer " num);
	CV.showCloud(cloud);
	//private static boost::mutex CV_mutex;
        //boost::mutex my_CV::CV_mutex; //storage for static lock 
    }

    
} 




int
init_readFiles()
{
    ifstream inFile("files.txt");
    if(inFile.fail())
    {
        std::cout << "could not find text file" << std::endl; 
	return 0;
    }
    char[20] nextToken;
    int counted = 0;
    while (inFile >> nextToken){
       counted++;
    }
    std::vector<char[20]> Files(instances);
    counted = 0;
    while (inFile >> nextToken){
       Files[counted] = nextToken;
       counted++;
    }

    return counted;
}


    
int 
main ()
{
    int instances = 0;
    instances = init_readFiles();
    if(instances == 0)
    {
    	return 0;
    }
    
    std::vector<my_CV> CV(instances);

    for(int i = 0; i < instances; i++)
    {
        CV[i].CV_func(i);
	//CV[i].execute();
    }
    return 0;
}

