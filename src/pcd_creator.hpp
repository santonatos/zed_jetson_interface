#ifndef PCD_CREATOR_HPP_
#define PCD_CREATOR_HPP_

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>


int pcloud_to_pcd(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pcloud,int c);


#endif /*PCD_CREATOR_HPP*/
