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
typedef shared_ptr<pcl::visualization::PCLVisualizer> visualizer_pointer;
typedef pcl::visualization::PCLVisualizer visualizer;
/*typedef struct mouseOCVStruct { //for open cv
    Mat depth;
    cv::Size _resize;
} mouseOCV;
*/
// Global instance (ZED, Mat, callback)
Camera zed;
Mat data_cloud;
Mat data_cloud2;
Mat data_cloud3;
std::thread zed_callback;
std::mutex mutex_input;
std::mutex mutex_input2;
bool stop_signal;
bool has_data;
Mat show_sl_mat;


// Sample functions
void startZED();
void zed_run();
void closeZED();
visualizer_pointer createXYZRGBVisualizer(color_cloud_const_pointer cloud);
visualizer_pointer createXYZVisualizer(non_color_cloud_const_pointer cloud);
visualizer_pointer createXYZRGBShapeVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
inline float convertColor(float colorIn);
void transfer_mat_color(Mat data_cloud,visualizer_pointer viewer,color_cloud_pointer my_point_cloud_pointer);
void transfer_mat_non_color(Mat data_cloud,visualizer_pointer viewer,color_cloud_pointer my_point_cloud_pointer);
//cv::Mat slMat2cvMat(sl::Mat& input);
//static void onMouseCallback(int32_t event, int32_t x, int32_t y, int32_t flag, void * param);



// Main process
int main3345(int argc, char** argv) {

    if (argc > 2) {
        cout << "accepts only up tp one argument, the path of an svo file " << endl;
        return -1;
    }

// Set configuration parameters - change that to a function and use pointers
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD720;
    if (argc == 2) init_params.svo_input_filename = argv[1];
    init_params.coordinate_units = UNIT_METER;
    init_params.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;

// open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
    {
        cout << "an error occured while opening: " << errorCode2str(err) << endl;
        return 1;
    }


// create the point cloud and allocate enough memory, at the camera resolution
    color_cloud_pointer my_point_cloud_pointer(new color_cloud);
    my_point_cloud_pointer->points.resize(zed.getResolution().area());

    color_cloud_pointer my_point_cloud_pointer2(new color_cloud);
    my_point_cloud_pointer2->points.resize(zed.getResolution().area());

    color_cloud_pointer my_point_cloud_pointer3(new color_cloud);
    my_point_cloud_pointer3->points.resize(zed.getResolution().area());

//create visualizer
    visualizer_pointer viewer = createXYZRGBVisualizer(my_point_cloud_pointer);
    visualizer_pointer viewer2 = createXYZRGBVisualizer(my_point_cloud_pointer2);
    visualizer_pointer viewer3 = createXYZRGBShapeVisualizer(my_point_cloud_pointer3);

//start zed
    startZED();

//MAIN LOOP main loop, loop while viewer does not stop
    while(!viewer->wasStopped() && !viewer2->wasStopped()){
		
	
       // Try to lock the data if possible (not in use). Otherwise, do nothing.
	transfer_mat_color(data_cloud,viewer,my_point_cloud_pointer);
	transfer_mat_non_color(data_cloud2,viewer2,my_point_cloud_pointer2);
	transfer_mat_color(data_cloud3,viewer3,my_point_cloud_pointer3);

    }

    // Close the viewers
    viewer->close();
    viewer2->close();
    viewer3->close();
    
    // Close the zed
    closeZED();
    return 0;
}

//from pcl web site
visualizer_pointer createXYZRGBVisualizer(color_cloud_const_pointer cloud) {
    // Open 3D viewer and add point cloud
    visualizer_pointer viewer(new visualizer("ZED 3D COLOR PCL Visualizer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

visualizer_pointer createXYZVisualizer (color_cloud_const_pointer cloud)
{
  // -----Open 3D viewer and add point cloud-----
  visualizer_pointer viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (200, 200, 200);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


visualizer_pointer createXYZRGBShapeVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  visualizer_pointer viewer (new visualizer ("3D Viewer"));
  viewer->setBackgroundColor (255, 200, 10);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
                                     cloud->points[cloud->size() - 1], "line");
  viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations-----
  //---------------------------------------
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  viewer->addPlane (coeffs, "plane");
  coeffs.values.clear ();
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (5.0);
  viewer->addCone (coeffs, "cone");

  return (viewer);
}

/**
*  This function convert a RGBA color packed into a packed RGBA PCL compatible format
**/
inline float convertColor(float colorIn) {
    uint32_t color_uint = *(uint32_t*)& colorIn;
    unsigned char* color_uchar = (unsigned char*)&color_uint;
    color_uint = ((uint32_t)color_uchar[0] << 16 | (uint32_t)color_uchar[1] << 8 | (uint32_t)color_uchar[2]);
    return *reinterpret_cast<float*> (&color_uint);
}
/**
*  This functions start the ZED's thread that grab images and data.
**/

void startZED()
{
    // Start the thread for grabbing ZED data
    stop_signal = false;
    has_data = false;
    zed_callback = std::thread(zed_run);
  
    //Wait for data to be grabbed
    while(!has_data)
	sleep_ms(1);
}
   
/**
*  This function loops to get the point cloud from the ZED. It can be considered as a callback.
**/
void zed_run() {
    //start_opencv(); ADD AFTER
    //show_image(zed);
    while (!stop_signal) {
        if (zed.grab(SENSING_MODE_STANDARD) == SUCCESS) {
            mutex_input.lock(); // To prevent from data corruption
            zed.retrieveMeasure(data_cloud, MEASURE_XYZRGBA);  //MOST IMPORTANT
	    //cv::imshow("my data cloud",data_cloud);
            mutex_input.unlock();
           mutex_input2.lock(); // To prevent from data corruption
            zed.retrieveMeasure(data_cloud2, MEASURE_XYZRGBA);  //MOST IMPORTANT
	    //cv::imshow("my data cloud",data_cloud);
            mutex_input2.unlock();
	    has_data=true;
	    //cout << "here"<< endl;


 
        }
        sleep_ms(1);
    }
}

/**
*  This function closes thread and its callback thread
**/
void closeZED()
{
    // Stop the thread
    stop_signal = true;
    zed_callback.join();
    zed.close();
}

/**
* this function...
**/
void transfer_mat_color(Mat data_cloud,visualizer_pointer viewer,color_cloud_pointer my_point_cloud_pointer3){
        if (mutex_input.try_lock()) { //IT STARTS HERE
	  float *p_data_cloud = data_cloud.getPtr<float>();
	  int index = 0;
	  
	  // Check and adjust points for PCL format
	  for (auto &it : my_point_cloud_pointer3->points) {
	      float X = p_data_cloud[index];
	      if (!isValidMeasure(X)) // Checking if it's a valid point
		  it.x = it.y = it.z = it.rgb = 0;
	      else {
		  it.x = X;
		  it.y = p_data_cloud[index + 1];
		  it.z = p_data_cloud[index + 2];
		  it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format
	      }
	      index+=4;
	  }
           
          // Unlock data and update Point cloud
	  mutex_input.unlock();
	  viewer->updatePointCloud(my_point_cloud_pointer3);
	  viewer->spinOnce(15);
        }
        else
        sleep_ms(1);
}

/**
* this function...
**/
void transfer_mat_non_color(Mat data_cloud,visualizer_pointer viewer,color_cloud_pointer my_point_cloud_pointer4){
        if (mutex_input2.try_lock()) { //IT STARTS HERE
	  float *p_data_cloud = data_cloud.getPtr<float>();
	  int index = 0;
	  
	  // Check and adjust points for PCL format
	  for (auto &it : my_point_cloud_pointer4->points) {
	      float X = p_data_cloud[index];
	      if (!isValidMeasure(X)) // Checking if it's a valid point
		  it.x = it.y = it.z = 0;
	      else {
		  it.x = X;
		  it.y = p_data_cloud[index + 1];
		  it.z = p_data_cloud[index + 2];
		  //it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format
	      }
	      index+=4;
	  }
           
          // Unlock data and update Point cloud
	  mutex_input2.unlock();
	  viewer->updatePointCloud(my_point_cloud_pointer4);
	  viewer->spinOnce(15);
        }
        else
        sleep_ms(1);
}


