#ifndef _EXTRACT_POINT_CLOUD_
#define _EXTRACT_POINT_CLOUD_

#include<ctime>
 
#include<chrono>
#include<string>
#include<vector>
#include<fstream>
#include<sstream>
#include<iostream>
#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/common/common.h>
#include<pcl/filters/passthrough.h>
#include<pcl/filters/crop_box.h>
#include<boost/geometry.hpp>
#include<boost/geometry/geometries/point_xy.hpp>
#include<boost/geometry/geometries/polygon.hpp>
//#include<python3.5m>

struct rec_3d_data
{
    float ry;
    float t1,t2,t3;
    float h,w,l;
    rec_3d_data(float ry_,float t1_,float t2_,float t3_,float h_,float w_,float l_);
};

void to_8corners(const rec_3d_data &r);

class extract_point
{
private:
std::string pcd_file_;
std::string out_file_;
//std::chrono::milliseconds dur_;
std::chrono::duration<double,std::ratio<1,1000>> dur_;
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> res_;
public:
extract_point(const std::string inf,const std::string outf);

std::chrono::duration<double,std::ratio<1,1000>> extract_time();

};

#endif