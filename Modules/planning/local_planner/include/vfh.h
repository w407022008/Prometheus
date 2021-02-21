#ifndef VFH_H
#define VFH_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tools.h"
#include "local_planning_alg.h"
#include "message_utils.h"

using namespace std;

namespace Local_Planning
{

extern ros::Publisher message_pub;

class VFH: public local_planning_alg
{
private:
    //　参数
    double inflate_distance;
    double sensor_max_range;
    double safe_distance;
    double inflate_plus_safe_distance;
    double ground_height;
    double limit_v_norm;
    double  Hres;
    int Hcnt;  // 直方图横向个数
    double  Vres;
    int Vcnt;  // 直方图纵向个数

/*    double goalWeight, obstacle_weight;*/

	// bool 参数
    bool has_local_map_;
    bool has_odom_;
    bool is_2D,isCylindrical,isSpherical;


	// Histogram
    double* Histogram_2d;
    double** Histogram_3d;

    pcl::PointCloud<pcl::PointXYZ> latest_local_pcl_;
    sensor_msgs::PointCloud2ConstPtr  local_map_ptr_;
    nav_msgs::Odometry cur_odom_;

    void PolarCoordinateVFH(double angle_cen, double angle_range, double val);
    void CylindricalCoordinateVFH(double hor_angle_cen, double angle_range, double val);
    void SphericalCoordinateVFH(double hor_angle_cen, double ver_angle_cen, double angle_range, double val);

public:

    virtual void set_odom(nav_msgs::Odometry cur_odom);
    virtual void set_local_map(sensor_msgs::PointCloud2ConstPtr &local_map_ptr);
    virtual void set_local_map_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_ptr);
    virtual int compute_force(Eigen::Vector3d  &goal, Eigen::Vector3d &desired_vel);
    virtual void init(ros::NodeHandle& nh);

    VFH(){}
    ~VFH(){
        delete Histogram_2d;
        if (is_2D){
		    for(int i = 0; i < Vcnt; i++)
				delete[] Histogram_3d[i];
			delete[] Histogram_3d;
        }
    }

    typedef shared_ptr<VFH> Ptr;

};

}

#endif 
