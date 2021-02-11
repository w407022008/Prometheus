#include "apf.h"
#include "math.h"

namespace Local_Planning
{
Eigen::Vector3d last_desired_vel;
    	        
auto min=[](double v1, double v2)->double
{
    return v1<v2 ? v1 : v2;
};    	        
auto max=[](double v1, double v2)->double
{
    return v1<v2 ? v1 : v2;
};
auto sign=[](double v)->double
{
	return v<0.0? -1.0:1.0;
};

void APF::init(ros::NodeHandle& nh)
{
    has_local_map_ = false;

    nh.param("local_planner/inflate_distance", inflate_distance, 0.20);  // 最小障碍物距离
    nh.param("local_planner/sensor_max_range", sensor_max_range, 2.5);  // 感知障碍物距离
    nh.param("local_planner/max_planning_vel", max_planning_vel, 0.5); // 最大飞行速度
    nh.param("apf/k_push", k_push, 0.8);                         // 推力增益
    nh.param("apf/k_att", k_att, 0.4);                                  // 引力增益
    nh.param("apf/max_att_dist", max_att_dist, 5.0);             // 最大吸引距离
    nh.param("local_planner/ground_height", ground_height, 0.1);  // 地面高度
    nh.param("apf/ground_safe_height", ground_safe_height, 0.2);  // 地面安全距离
    nh.param("local_plannerapf/safe_distance", safe_distance, 0.15); // 安全停止距离

    // TRUE代表2D平面规划及搜索,FALSE代表3D 
    nh.param("local_planner/is_2D", is_2D, true); 
    
    sensor_max_range = max(sensor_max_range,3*inflate_distance);
}

void APF::set_local_map(sensor_msgs::PointCloud2ConstPtr &local_map_ptr)
{
    local_map_ptr_ = local_map_ptr;

    pcl::fromROSMsg(*local_map_ptr, latest_local_pcl_);

    has_local_map_=true;
}

void APF::set_local_map_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_ptr)
{
    latest_local_pcl_ = *pcl_ptr;
    has_local_map_=true;
}

void APF::set_odom(nav_msgs::Odometry cur_odom)
{
    cur_odom_ = cur_odom;
    has_odom_=true;
}

int APF::compute_force(Eigen::Vector3d &goal, Eigen::Vector3d &desired_vel)
{
    // 0 for not init; 1for safe; 2 for dangerous
    int local_planner_state=0;  
    int safe_cnt=0;

    if(!has_local_map_|| !has_odom_)
        return 0;

    if ((int)latest_local_pcl_.points.size() == 0) 
        return 0;

    if (isnan(goal(0)) || isnan(goal(1)) || isnan(goal(2)))
        return 0;

    //　当前状态
    Eigen::Vector3d current_pos;
    current_pos[0] = cur_odom_.pose.pose.position.x;
    current_pos[1] = cur_odom_.pose.pose.position.y;
    current_pos[2] = cur_odom_.pose.pose.position.z;
    Eigen::Vector3d current_vel;
    current_vel[0] = cur_odom_.twist.twist.linear.x;
    current_vel[1] = cur_odom_.twist.twist.linear.y;
    current_vel[2] = cur_odom_.twist.twist.linear.z;
    float  current_vel_norm = current_vel.norm();
    ros::Time begin_collision = ros::Time::now();

    // 引力
    Eigen::Vector3d uav2goal = goal - current_pos;
    double dist_att = uav2goal.norm();
    if(dist_att > max_att_dist)
    {
        uav2goal = max_att_dist * uav2goal/dist_att ;
    }
    //　计算吸引力
    attractive_force = k_att * uav2goal;

    // 排斥力
    double uav_height = cur_odom_.pose.pose.position.z;
    repulsive_force = Eigen::Vector3d(0.0, 0.0, 0.0);
    guid_force = Eigen::Vector3d(0.0, 0.0, 0.0);
    int count;
    double max_guid_force = 0.0;

    Eigen::Vector3d p3d;
    vector<Eigen::Vector3d> obstacles;
    
    //　根据局部点云计算排斥力（是否可以考虑对点云进行降采样？）
    for (size_t i = 0; i < latest_local_pcl_.points.size(); ++i) 
    {
        p3d(0) = latest_local_pcl_.points[i].x;
        p3d(1) = latest_local_pcl_.points[i].y;
        p3d(2) = latest_local_pcl_.points[i].z; // World-ENU frame

        Eigen::Vector3d uav2obs = p3d - current_pos;

        //　低速悬浮时不考虑
		if(current_vel_norm<0.3)
			continue;
        //　不考虑地面上的点的排斥力
        if(fabs(p3d(2))<ground_height)
            continue;

        //　超出感知范围，则不考虑该点的排斥力
        double dist_push = (uav2obs).norm();

		
    	obs_angle = acos(uav2obs.dot(current_vel) / uav2obs.norm() / current_vel_norm);
        if(isnan(dist_push) || (dist_push > min(3*inflate_distance,sensor_max_range/2) && obs_angle > M_PI/6)){
        	continue;            
		}
		
        // 如果当前的观测点中，包含小于安全停止距离的点，进行计数
        if(dist_push < safe_distance+inflate_distance)
        {
            safe_cnt++;
            desired_vel += 1000 * (-uav2obs)/dist_push;
            if(is_2D)
            {
                desired_vel[2] = 0.0;
            }
            if(safe_cnt>3)
            {
                desired_vel /= safe_cnt;
                return 2;  //成功规划，但是飞机不安全
            }
        }

        obstacles.push_back(p3d);
        double push_gain = k_push * (1/(max(inflate_distance,dist_push) - inflate_distance + 1e-6) - 1/(sensor_max_range-inflate_distance));

        if(dist_att<1.0)
        {
            push_gain *= pow(dist_att,2);  // to gaurantee to reach the goal.
        }

		if(dist_push < min(3*inflate_distance,sensor_max_range/2)){
            repulsive_force += push_gain * (-uav2obs)/dist_push;
			count ++;
		}
		if (obs_angle < M_PI/6)
			guid_force += current_vel.cross((-uav2obs).cross(current_vel)) / pow(current_vel_norm,2) / dist_push * 1/max(dist_push*sin(obs_angle), 0.05); // or / pow(dist_push,2)
			if (max_guid_force<min(3,1/(max(inflate_distance,dist_push) - inflate_distance + 1e-6))) max_guid_force = 0.6*max_guid_force + 0.4*min(3,1/(max(inflate_distance,dist_push) - inflate_distance + 1e-6));
    }

    //　平均排斥力
    if(count != 0)
    {
        repulsive_force=repulsive_force/count; //obstacles.size();
    }
	if(count==int(obstacles.size()))
	{
		guid_force=Eigen::Vector3d(0.0,0.0,0.0);
	}else{
		double guid_force_norm = guid_force.norm();
		if(guid_force_norm<1e-6) 
			guid_force += Eigen::Vector3d(current_vel(1),-current_vel(0),current_vel(2))/current_vel_norm*1e-6;
		guid_force = k_push*max_guid_force*guid_force/guid_force.norm();
	}

    // 地面排斥力
    if (current_pos[2] <2*ground_safe_height)
		repulsive_force += Eigen::Vector3d(0.0, 0.0, 1.0) * k_push * (1/(max(ground_safe_height,current_pos[2]) - ground_safe_height + 1e-6) - 1/(2*ground_safe_height));

    // 合力
    desired_vel = 0.3*repulsive_force + 0.4*attractive_force + 0.3*guid_force; // ENU frame

    if(is_2D)
        desired_vel[2] = 0.0;

	if(max_planning_vel<desired_vel.norm())
		desired_vel = desired_vel/desired_vel.norm()*max_planning_vel;
		
	desired_vel = 0.5*last_desired_vel + 0.5*desired_vel;
    last_desired_vel = desired_vel;
    
	cout << "guid_force: " << guid_force(0) << "," << guid_force(1) << "," << guid_force(2) << endl;
	cout << "repulsive_force: " << repulsive_force(0) << "," << repulsive_force(1) << "," << repulsive_force(2) << endl;
	cout << "attractive_force: " << attractive_force(0) << "," << attractive_force(1) << "," << attractive_force(2) << endl;
	cout << "desired_vel: " << desired_vel(0) << "," << desired_vel(1) << "," << desired_vel(2) << endl;
	cout << " " << endl;
	
    local_planner_state =1;  //成功规划， 安全

    static int exec_num=0;
    exec_num++;

    // 此处改为根据循环时间计算的数值
    if(exec_num == 100)
    {
        printf("APF calculate take %f [s].\n",   (ros::Time::now()-begin_collision).toSec());
        exec_num=0;
    }  

    return local_planner_state;
}



}
