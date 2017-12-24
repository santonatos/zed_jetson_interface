// ZED includes
#include <sl/Camera.hpp>

// PCL includes
// Undef on Win32 min/max for PCL
#ifdef _WIN32
#undef max
#undef min
#endif
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

//CV includes
#include <opencv2/opencv.hpp>

// Sample includes
#include <thread>
#include <mutex>
#include <vector>

//other includes 

#include "opencv_inter.hpp" //INCLUDE LATER AGAIN

// Namespace
using namespace sl;
using namespace std;

//type definitions
typedef pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr color_cloud_const_pointer;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_pointer; 
typedef pcl::PointCloud<pcl::PointXYZRGB> color_cloud; 
typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr non_color_cloud_const_pointer;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr non_color_cloud_pointer; 
typedef pcl::PointCloud<pcl::PointXYZ> non_color_cloud; 

// Sample functions
void startZED(bool& stop_signal,bool &has_data,thread& zed_callback,Camera& zed,mutex& mutex_input,Mat& data_cloud,color_cloud_pointer my_point_cloud_pointer);
void zed_run(bool& stop_signal,bool &has_data,Camera& zed,mutex& mutex_input,Mat& data_cloud,color_cloud_pointer my_point_cloud_pointer);
void closeZED(bool& stop_signal,Camera& zed,thread& zed_callback);
//visualizer_pointer createXYZRGBVisualizer(color_cloud_const_pointer cloud);
//visualizer_pointer createXYZVisualizer(non_color_cloud_const_pointer cloud);
//visualizer_pointer createXYZRGBShapeVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
inline float convertColor(float colorIn);
void transfer_mat_pcloud(Mat& data_cloud,color_cloud_pointer my_point_cloud_pointer,mutex& mutex_input);
int collect_pclouds();
//void transfer_mat_non_color(Mat data_cloud,visualizer_pointer viewer,color_cloud_pointer my_point_cloud_pointer);

