#include "pcd_creator.hpp"
#include <chrono>
#include <ctime>

/**
*function to convert the point cloud to a pcd file
*/
int pcloud_to_pcd(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pcloud,int c){
 
//auto t1 = Clock::now();
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
  
  pcl::PointCloud<pcl::PointXYZRGB> cloud = *pcloud;
  //cloud.width = cloud.points.size ();
  //cloud.height = 1; 

  std::string filename = boost::lexical_cast<std::string>(c) + "_.pcd";  
  pcl::io::savePCDFileASCII (filename, cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to " << filename << std::endl;

     end = std::chrono::system_clock::now();
 
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
 
    std::cout << "finished cloud saving at " << std::ctime(&end_time)
              << "elapsed time: " << elapsed_seconds.count() << "s\n";


  return (0);
}

