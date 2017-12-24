#include "pcloud_collector.hpp"
//#include "opencv_inter.hpp"
#include <pcl/io/vtk_lib_io.h>
/*#include "pcd_creator.hpp"*/


int pcloud_to_pcd(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pcloud,int c);
Mat data_cloud1;
// Main process
int collect_pclouds() {
    Camera zed;
    Mat data_cloud;
    std::thread zed_callback;
    std::mutex mutex_input;
    bool stop_signal;
    bool has_data;

    vector<color_cloud_pointer> my_pclouds;

// Set configuration parameters - change that to a function and use pointers
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD720;
    //if (argc == 2) init_params.svo_input_filename = argv[1];
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




//POINTER DECLARED HERE create the point cloud and allocate enough memory, at the camera resolution
    color_cloud_pointer my_point_cloud_pointer(new color_cloud);
    my_point_cloud_pointer->points.resize(zed.getResolution().area());



//    color_cloud_pointer my_point_cloud_pointer2(new color_cloud);
//    my_point_cloud_pointer2->points.resize(zed.getResolution().area());

//    color_cloud_pointer my_point_cloud_pointer3(new color_cloud);
//    my_point_cloud_pointer3->points.resize(zed.getResolution().area());

//create visualizer
//    visualizer_pointer viewer = createXYZRGBVisualizer(my_point_cloud_pointer);
//    visualizer_pointer viewer2 = createXYZRGBVisualizer(my_point_cloud_pointer2);
//    visualizer_pointer viewer3 = createXYZRGBShapeVisualizer(my_point_cloud_pointer3);

//start zed
    startZED(stop_signal,has_data,zed_callback,zed,mutex_input,data_cloud,my_point_cloud_pointer);

//MAIN LOOP main loop, loop while viewer does not stop
    /*while(1){
    	color_cloud_pointer my_point_cloud_pointer(new color_cloud);
    	my_point_cloud_pointer->points.resize(zed.getResolution().area());
       // Try to lock the data if possible (not in use). Otherwise, do nothing.
	//transfer_mat_color(data_cloud,viewer,my_point_cloud_pointer);
	//transfer_mat_non_color(data_cloud2,viewer2,my_point_cloud_pointer2);
	//transfer_mat_color(data_cloud3,viewer3,my_point_cloud_pointer3);
	transfer_mat_pcloud(data_cloud,my_point_cloud_pointer,mutex_input);
	my_pclouds.push_back (my_point_cloud_pointer);
    }*/

    // Close the viewers
    //viewer->close();
    //viewer2->close();
    //viewer3->close();
    
    // Close the zed
    closeZED(stop_signal,zed,zed_callback);
    return 0;
}


void startZED(bool& stop_signal,bool &has_data,thread& zed_callback,Camera& zed,mutex& mutex_input,Mat& data_cloud,color_cloud_pointer my_point_cloud_pointer)
{
    // Start the thread for grabbing ZED data
    stop_signal = false;
    has_data = false;
    //zed_callback = std::thread(zed_run,stop_signal,has_data,zed,mutex_input,data_cloud);
    zed_run(stop_signal,has_data,zed,mutex_input,data_cloud,my_point_cloud_pointer);
    //zed_callback = std::thread(start_opencv);
    //Wait for data to be grabbed
    /*while(!has_data)
	sleep_ms(1);*/
}
   
/**
*  This function loops to get the point cloud from the ZED. It can be considered as a callback.
**/
void zed_run(bool& stop_signal,bool &has_data,Camera& zed,mutex& mutex_input,Mat& data_cloud,color_cloud_pointer my_point_cloud_pointer) {
    //start_opencv(); ADD AFTER
    //show_image(zed);
    int c = 0;
    while (!stop_signal && c < 1000*20) {
        if (zed.grab(SENSING_MODE_STANDARD) == SUCCESS) {
            //mutex_input.lock(); // To prevent from data corruption
	    cout << "will sleep for 3" << endl;
	    sleep(4);


            zed.retrieveMeasure(data_cloud, MEASURE_XYZRGBA);  //MOST IMPORTANT
	    cout << "stopped retrieving" << endl;
	    sleep(1);
	    //cv::imshow("my data cloud",data_cloud);
            //mutex_input.unlock();
   	//color_cloud_pointer my_point_cloud_pointer(new color_cloud);
    	//my_point_cloud_pointer->points.resize(zed.getResolution().area());
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr p1cl;
	transfer_mat_pcloud(data_cloud,my_point_cloud_pointer,mutex_input);
	pcloud_to_pcd(my_point_cloud_pointer,c);
	//pcloud_to_pcd(p1cl,c);
	
	//my_pclouds.push_back (my_point_cloud_pointer);
           //mutex_input2.lock(); // To prevent from data corruption
           // zed.retrieveMeasure(data_cloud2, MEASURE_XYZRGBA);  //MOST IMPORTANT
	    //cv::imshow("my data cloud",data_cloud);
           // mutex_input2.unlock();
	    has_data=true;
	    //cout << "here"<< endl; 
        }else{
		cout << "could not grabbed zed in sensing mode" << endl;
	}
        //sleep_ms(1);
	if((c % 1000) == 0){
		cout << " %d second passed" << c << endl;
	}
	c++;
    }
}

/**
*  This function closes thread and its callback thread
**/
void closeZED(bool& stop_signal,Camera& zed,thread& zed_callback)
{
    // Stop the thread
    stop_signal = true;
    zed_callback.join();
    zed.close();
}


/**
* this function...
**/
void transfer_mat_pcloud(Mat& data_cloud,color_cloud_pointer my_point_cloud_pointer,mutex& mutex_input){
        if (mutex_input.try_lock()) { //IT STARTS HERE
	  float *p_data_cloud = data_cloud.getPtr<float>();
	  int index = 0;
	  
	  // Check and adjust points for PCL format
	  for (auto &it : my_point_cloud_pointer->points) {
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
	  //viewer->updatePointCloud(my_point_cloud_pointer3);
	  //viewer->spinOnce(15);
        }
        else
        sleep_ms(1);
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


int pcloud_to_pcd(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pcloud,int c){
  pcloud->width = pcloud->points.size ();
  pcloud->height = 1; 

  pcl::PointCloud<pcl::PointXYZRGB> cloud = *pcloud;

  std::string filename = std::to_string(c) + "_.pcd";  
  pcl::io::savePCDFile(filename, cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to " << filename << std::endl;

  /*for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
*/
  return (0);
}

