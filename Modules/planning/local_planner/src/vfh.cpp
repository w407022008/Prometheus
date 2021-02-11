#include "vfh.h"
#include "string.h"
#include "math.h"
#include "iostream"

namespace Local_Planning
{

void VFH::init(ros::NodeHandle& nh)
{
    has_local_map_ = false;

    nh.param("local_planner/inflate_distance", inflate_distance, 0.50);  // 障碍物影响距离
    nh.param("local_planner/sensor_max_range", sensor_max_range, 3.0);  // 探测最大距离
    nh.param("local_planner/ground_height", ground_height, 0.1);  // 地面高度
    nh.param("vfh/goalWeight", goalWeight, 0.2); // 目标权重
    nh.param("vfh/h_res", Hcnt, 180); // 直方图 个数
    nh.param("vfh/obstacle_weight", obstacle_weight, 0.0); // 障碍物权重
    nh.param("local_planner/safe_distance", safe_distance, 0.2); // 安全停止距离

    nh.param("local_planner/max_planning_vel", limit_v_norm, 0.4);
    // TRUE代表2D平面规划及搜索,FALSE代表3D 
    nh.param("local_planner/is_2D", is_2D, false); 
    inflate_and_safe_distance = safe_distance + inflate_distance;
    
    Hres = 2*M_PI/Hcnt;
    Cost_hist = new double[Hcnt]();
    for(int i(0); i<Hcnt; i++)
    {
        Cost_hist[i] =0.0;       
    }
}

// get the map
void VFH::set_local_map(sensor_msgs::PointCloud2ConstPtr &local_map_ptr)
{
    local_map_ptr_ = local_map_ptr;
    ros::Time begin_load_point_cloud = ros::Time::now();

    pcl::fromROSMsg(*local_map_ptr_, latest_local_pcl_);

    has_local_map_=true;
}

void VFH::set_local_map_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_ptr)
{
    latest_local_pcl_ = *pcl_ptr;
    has_local_map_=true;
}

void VFH::set_odom(nav_msgs::Odometry cur_odom)
{
    cur_odom_ = cur_odom;
    has_odom_=true;
}

int VFH::compute_force(Eigen::Vector3d  &goal, Eigen::Vector3d &desired_vel)
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

    // clear the 2D Cost_hist
    if(is_2D || true)
		for(int i =0; i<Hcnt; i++)
		    Cost_hist[i]=0;

    // 状态量
    ros::Time begin_collision = ros::Time::now();
    Eigen::Vector3d current_pos;
    current_pos[0] = cur_odom_.pose.pose.position.x;
    current_pos[1] = cur_odom_.pose.pose.position.y;
    current_pos[2] = cur_odom_.pose.pose.position.z;
    Eigen::Vector3d current_vel;
    current_vel[0] = cur_odom_.twist.twist.linear.x;
    current_vel[1] = cur_odom_.twist.twist.linear.y;
    current_vel[2] = cur_odom_.twist.twist.linear.z;
    Eigen::Vector3d uav2goal = goal - current_pos;


    auto sign=[](double v)->double
    {
        return v<0.0? -1.0:1.0;
    };




//    Eigen::Quaterniond cur_rotation_local_to_global(cur_odom_.pose.pose.orientation.w, cur_odom_.pose.pose.orientation.x, cur_odom_.pose.pose.orientation.y, cur_odom_.pose.pose.orientation.z); 
//    Eigen::Matrix<double,3,3> rotation_mat_local_to_global = cur_rotation_local_to_global.toRotationMatrix();
//    Eigen::Vector3d eulerAngle_yrp = rotation_mat_local_to_global.eulerAngles(2, 1, 0);// R_z*R_y*R_x
//    rotation_mat_local_to_global = Eigen::AngleAxisd(eulerAngle_yrp(0), Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // 遍历点云中的所有点
    Eigen::Vector3d p3d;
    vector<Eigen::Vector3d> obstacles;
    for (size_t i = 0; i < latest_local_pcl_.points.size(); ++i) 
    {
        // 提取障碍物点
        p3d(0) = latest_local_pcl_.points[i].x;
        p3d(1) = latest_local_pcl_.points[i].y;
        p3d(2) = latest_local_pcl_.points[i].z; // World_ENU frame

        // World_ENU frame to Body_ENU frame
        Eigen::Vector3d uav2obs = p3d - current_pos; 

        //　不考虑地面上的点的排斥力
        if(fabs(p3d(2))<ground_height)
            continue;

        //　超出最大考虑距离，则忽略该点
        double dist_push = (uav2obs).norm();
        if(dist_push > sensor_max_range || isnan(dist_push))
            continue;


        double obs_dist = uav2obs.norm();
        if (is_2D || true){
		    if (fabs(uav2obs(2))>0.5) continue;
		    uav2obs(2) = 0.0;
		    double obs_angle = sign(uav2obs(1)) * acos(uav2obs(0) / uav2obs.norm());
		    double angle_range;
		    if(obs_dist>inflate_and_safe_distance)
		    {
		        angle_range = asin(inflate_and_safe_distance/obs_dist);
		    }else if (obs_dist<=inflate_and_safe_distance)
		    {
		        angle_range = M_PI/2;
		        safe_cnt++;  // 非常危险
		    }
        
		    double obstacle_cost = obstacle_weight / (1/inflate_distance - 1/(inflate_distance>obs_dist ? inflate_distance+1e-3 : obs_dist) );
		    generate_voxel_data(obs_angle, angle_range, obstacle_cost);
		}

        obstacles.push_back(p3d);
    }


    // 与目标点相关cost计算
    // 不考虑高度影响
    double dist_att = uav2goal.norm();
    uav2goal(2) = 0.0;
    double goal_heading = sign(uav2goal(1)) * acos(uav2goal(0)/uav2goal.norm());
    string s;
    for(int i=0; i<Hcnt; i++)
    {
        // angle_i 为当前角度
        double angle_i = find_angle(i);

        double angle_er = angle_error(angle_i, goal_heading); // angle_i in 0 ~ 2pi; heading in -pi ~ pi
        float goal_gain;
        if(dist_att>3.0) 
        {
            goal_gain = 3.0;
        }
        else if(dist_att<0.5) 
        {
            goal_gain = 0.5;
        } 
        else{
            goal_gain = dist_att;
        }
        // 当前角度与目标角度差的越多,则该代价越大
        Cost_hist[i] += goalWeight * angle_er * goal_gain;
    }

    // 与当前速度相关cost计算
    double hor_current_vel_norm = Eigen::Vector3d(current_vel(0),current_vel(1),0.0).norm();
    if (hor_current_vel_norm>1e-6){
		double current_heading = sign(current_vel(1)) *acos(current_vel(0)/hor_current_vel_norm);
		
		for(int i=0; i<Hcnt; i++)
		{
		    // angle_i 为当前角度
		    double angle_i = find_angle(i);

		    double angle_er = angle_error(angle_i, current_heading); // angle_i in 0 ~ 2pi; heading in -pi ~ pi
		    // 当前角度与目标角度差的越多,则该代价越大
		    Cost_hist[i] += goalWeight * angle_er * 0.5;
		}
	}



    // 寻找cost最小的方向
    int best_ind = find_optimization_path();   // direction 
    // 提取最优路径的航向角
    double best_heading  = find_angle(best_ind);
    // 指导速度
    desired_vel(0) = cos(best_heading)*limit_v_norm;
    desired_vel(1) = sin(best_heading)*limit_v_norm;
    desired_vel(2) = 0.0;



    // 如果不安全的点超出指定数量
    if(safe_cnt>5)
    {
        local_planner_state = 2;  //成功规划，但是飞机不安全
    }else
    {
        local_planner_state =1;  //成功规划， 安全
    }

    static int exec_num=0;
    exec_num++;

    // 此处改为根据循环时间计算的数值
    if(exec_num == 10)
    {
        printf("VFH calculate take %f [s].\n",   (ros::Time::now()-begin_collision).toSec());
        exec_num=0;
    }  
    

    return local_planner_state;
}

// 寻找最小
int VFH::find_optimization_path(void)
{
    int bset_ind = 10000;
    double best_cost = 100000;
    for(int i=0; i<Hcnt; i++){
        if(Cost_hist[i]<best_cost){
            best_cost = Cost_hist[i];
            bset_ind = i;
        }
    }
    return bset_ind;
}

// angle_cen: -pi ~ pi; angle_range: 0~pi/2
void VFH::generate_voxel_data(double angle_cen, double angle_range, double val)  // set the map obstacle into the H data
{
	angle_cen = angle_cen<0 ? angle_cen+2*M_PI : angle_cen; // 0 ~ 2pi
    double angle_max = angle_cen + angle_range;
    double angle_min = angle_cen - angle_range;
    int cnt_min = find_Hcnt(angle_min);
    int cnt_max = find_Hcnt(angle_max);
    if(cnt_min>cnt_max)
    {
        for(int i=cnt_min; i<Hcnt; i++)
        {
            Cost_hist[i] += val;
        }
        for(int i=0;i<cnt_max; i++)
        {
            Cost_hist[i] +=val;
        }
    }else
    {
        for(int i=cnt_min; i<=cnt_max; i++)
        {
            Cost_hist[i] += val;
        }
    }
     
}

// angle: deg within -pi/2 ~ 5/2*pi
int VFH::find_Hcnt(double angle){
	angle = angle<0 ? angle+2*M_PI : angle>2*M_PI ? angle-2*M_PI : angle; // 0 ~ 2pi
    int cnt = floor(angle/Hres);
    return cnt;
}

double VFH::find_angle(int cnt){
    double angle = (cnt +0.5)* Hres;
    return angle;
}

// angle_1 in 0 ~ 2pi; angle_2 in -pi ~ pi
double VFH::angle_error(double angle1, double angle2){
	angle2 = angle2<0 ? angle2+2*M_PI : angle2;
    double angle_er = angle1 - angle2;
    angle_er = fabs(angle_er);
    
    return angle_er>M_PI ? 2*M_PI-angle_er : angle_er;
}

}
