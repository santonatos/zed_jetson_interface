#include "pcloud_collector2.hpp"
#include "pcd_creator.hpp"
#include <pcl/io/vtk_lib_io.h>

/*#include "pcd_creator.hpp"*/

void save_pcd();

// Global variables, for the threads and mat clouds
Camera zed;
Mat data_cloud;
Mat data_cloud2;
Mat data_cloud3;
std::thread zed_callback;
std::thread save_cloud;
std::mutex mutex_input;
std::mutex mutex_clpointer;
bool stop_signal;
bool has_data;
Mat show_sl_mat;
color_cloud_pointer my_point_cloud_pointer(new color_cloud);
int cloud_counter;


//int pcloud_to_pcd2(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pcloud,int c);
//Mat data_cloud1;
// Main process
int collect_pclouds2() {
    /*!!!!Camera zed;
    Mat data_cloud;
    std::thread zed_callback;
    std::mutex mutex_input;
    bool stop_signal;
    bool has_data;*/

    //vector<color_cloud_pointer> my_pclouds;

// Set configuration parameters - change that to a function and use pointers
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD720;
    //if (argc == 2) init_params.svo_input_filename = argv[1];
    init_params.coordinate_units = UNIT_METER;
    //init_params.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;

    cloud_counter = 0;

// open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
    {
        cout << "an error occured while opening: " << errorCode2str(err) << endl;
        return 1;
    }


// create the point cloud and allocate enough memory, at the camera resolution
    //color_cloud_pointer my_point_cloud_pointer(new color_cloud);
    my_point_cloud_pointer->points.resize(zed.getResolution().area());
    my_point_cloud_pointer->width = zed.getResolution().width;
    my_point_cloud_pointer->height = zed.getResolution().height;
    cout << "is organized1: "<< my_point_cloud_pointer->isOrganized() << endl;


//create visualizer
    visualizer_pointer viewer = createXYZRGBVisualizer(my_point_cloud_pointer);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0.140176, -0.188087, -3.58694, -0.0165125, -0.999615, 0.0222822);
//    visualizer_pointer viewer2 = createXYZRGBVisualizer(my_point_cloud_pointer2);
//    visualizer_pointer viewer3 = createXYZRGBShapeVisualizer(my_point_cloud_pointer3);

//start zed
    startZED();

//MAIN LOOP main loop, loop while viewer does not stop
    while(!viewer->wasStopped()){
		
	
       // Try to lock the data if possible (not in use). Otherwise, do nothing.
    cout << "is organized2: "<< my_point_cloud_pointer->isOrganized() << endl;
	transfer_mat_color(data_cloud,viewer,my_point_cloud_pointer);
	//transfer_mat_non_color(data_cloud2,viewer2,my_point_cloud_pointer2);
	//transfer_mat_color(data_cloud3,viewer3,my_point_cloud_pointer3);
	sleep_ms(1);

    }

    // Close the viewers
    viewer->close();
    //viewer2->close();
    //viewer3->close();
    
    // Close the zed
    closeZED();
    return 0;




//POINTER DECLARED HERE create the point cloud and allocate enough memory, at the camera resolution
    //!!!!color_cloud_pointer my_point_cloud_pointer(new color_cloud);
    //!!!!my_point_cloud_pointer->points.resize(zed.getResolution().area());



//    color_cloud_pointer my_point_cloud_pointer2(new color_cloud);
//    my_point_cloud_pointer2->points.resize(zed.getResolution().area());

//    color_cloud_pointer my_point_cloud_pointer3(new color_cloud);
//    my_point_cloud_pointer3->points.resize(zed.getResolution().area());

//create visualizer
//    visualizer_pointer viewer = createXYZRGBVisualizer(my_point_cloud_pointer);
//    visualizer_pointer viewer2 = createXYZRGBVisualizer(my_point_cloud_pointer2);
//    visualizer_pointer viewer3 = createXYZRGBShapeVisualizer(my_point_cloud_pointer3);

//start zed
  //!!!!  startZED(stop_signal,has_data,zed_callback,zed,mutex_input,data_cloud,my_point_cloud_pointer);

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
    //!!!!closeZED(stop_signal,zed,zed_callback);
    //!!!!return 0;
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
    save_cloud = std::thread(save_pcd);
  
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
		cout << "retrieve ,waiting for input lock" << endl;
            mutex_input.lock(); // To prevent from data corruption
            zed.retrieveMeasure(data_cloud, MEASURE_XYZRGBA);  //MOST IMPORTANT
		cout << "retrieve ,acquired input lock" << endl;
	    //cv::imshow("my data cloud",data_cloud);
            mutex_input.unlock();
		cout << "retrieve ,released input lock" << endl;
           //mutex_input2.lock(); // To prevent from data corruption
           // zed.retrieveMeasure(data_cloud2, MEASURE_XYZRGBA);  //MOST IMPORTANT
	    //cv::imshow("my data cloud",data_cloud);
           // mutex_input2.unlock();
	    has_data=true;
	    //cout << "here"<< endl;


 
        }
        sleep_ms(1);
    }
}

/**
*this function
*/




/**
*  This function closes thread and its callback thread
**/
void closeZED()
{
    // Stop the thread
    stop_signal = true;
    zed_callback.join();
    zed.close();
    cout << "is organized: "<< my_point_cloud_pointer->isOrganized() << endl;
}

/**
* this function...
**/
void transfer_mat_color(Mat data_cloud,visualizer_pointer viewer,color_cloud_pointer my_point_cloud_pointer3){
        //if (mutex_input.try_lock()) { //IT STARTS HERE
	cout << "transfer ,waiting for 2locks" << endl;
	mutex_input.lock();
	cout << "transfer ,acquired input lock" << endl;
	mutex_clpointer.lock();
	cout << "transfer ,acquired pointer lock" << endl;
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
	  mutex_clpointer.unlock();
	  cout << "transfer, released for 2locks" << endl;
	  viewer->updatePointCloud(my_point_cloud_pointer3);
	  //pcloud_to_pcd(my_point_cloud_pointer3,cloud_counter);
	  //cloud_counter++;
	  viewer->spinOnce(15);
/*        }

        else
        sleep_ms(1);*/
}

/**
*
*/
void save_pcd(){

	while(!stop_signal){
		cout << "save ,waiting for pointer lock" << endl; 
		//mutex_input.lock();
		mutex_clpointer.lock();
	cout << "save ,acquired pointer lock" << endl;

		pcloud_to_pcd(my_point_cloud_pointer,cloud_counter);
		cloud_counter++;

		//mutex_input.unlock();
		mutex_clpointer.unlock();
		cout << "save ,released pointer lock" << endl;
		sleep_ms(1);

	}


}

/*
int pcloud_to_pcd2(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pcloud,int c){
  pcloud->width = pcloud->points.size ();
  pcloud->height = 1; 

  //pcl::PointCloud<pcl::PointXYZRGB> cloud = *pcloud;
  pcl::PointCloud<pcl::PointXYZRGB> cloud =  *my_point_cloud_pointer;
  std::string filename = std::to_string(c) + "_.pcd";  
  pcl::io::savePCDFile(filename, cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to " << filename << std::endl;
*/
  /*for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
*/
 // return (0);
//}

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

