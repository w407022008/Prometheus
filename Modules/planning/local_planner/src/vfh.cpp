#include "vfh.h"
#include "string.h"
#include "math.h"
#include "chrono" 
#include "ctime"
#include "iostream"

using namespace std::chrono; 
namespace Local_Planning
{

auto sign=[](double v)->double
{
    return v<0.0? -1.0:1.0;
};

void VFH::init(ros::NodeHandle& nh)
{
    has_local_map_ = false;

    nh.param("local_planner/inflate_distance", inflate_distance, 0.50);  // 障碍物影响距离
    nh.param("local_planner/sensor_max_range", sensor_max_range, 3.0);  // 探测最大距离
    nh.param("local_planner/ground_height", ground_height, 0.1);  // 地面高度
//    nh.param("vfh/goalWeight", goalWeight, 1.0); // 目标权重
//    nh.param("vfh/obstacle_weight", obstacle_weight, 1.0); // 障碍物权重
    nh.param("vfh/h_cnt", Hcnt, 180); // 直方图横向个数
    nh.param("vfh/v_cnt", Vcnt, 90); // 直方图纵向个数
    nh.param("local_planner/safe_distance", safe_distance, 0.2); // 安全停止距离
    nh.param("local_planner/max_planning_vel", limit_v_norm, 0.4);
    // TRUE代表2D平面规划及搜索,FALSE代表3D 
    nh.param("local_planner/is_2D", is_2D, false); 
    
    inflate_and_safe_distance = safe_distance + inflate_distance;
    Hres = 2*M_PI/Hcnt;
    Vres = M_PI/Vcnt;
    
    if(is_2D || true)
	    Histogram = new double[Hcnt]();
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
    // 0 for not init; 1 for safe; 2 for ok but dangerous
    int local_planner_state=0;  
    int safe_cnt=0;

    if(!has_local_map_|| !has_odom_){
    	cout << "[Err] Check map input and odom input!" << endl;
        return 0;
	}
    if ((int)latest_local_pcl_.points.size() == 0) 
	    cout << "[Wrn] no point cloud" << endl;

    if (isnan(goal(0)) || isnan(goal(1)) || isnan(goal(2))){
    	cout << "[Err] Goal Unkown!" << endl;
        return 0;
	}
    // reset the Histogram
    if(is_2D || true)
		for(int i=0; i<Hcnt; i++)
		    Histogram[i] = 1000;






    // 状态量
    std::chrono::time_point<std::chrono::system_clock> start, end; 
    start = std::chrono::system_clock::now(); 
    static int exec_num=0;
    
    Eigen::Vector3d current_pos;
    current_pos[0] = cur_odom_.pose.pose.position.x;
    current_pos[1] = cur_odom_.pose.pose.position.y;
    current_pos[2] = cur_odom_.pose.pose.position.z;
    Eigen::Vector3d current_vel;
    current_vel[0] = cur_odom_.twist.twist.linear.x;
    current_vel[1] = cur_odom_.twist.twist.linear.y;
    current_vel[2] = cur_odom_.twist.twist.linear.z;
    Eigen::Vector3d uav2goal = goal - current_pos;






//    Eigen::Quaterniond cur_rotation_local_to_global(cur_odom_.pose.pose.orientation.w, cur_odom_.pose.pose.orientation.x, cur_odom_.pose.pose.orientation.y, cur_odom_.pose.pose.orientation.z); 
//    Eigen::Matrix<double,3,3> rotation_mat_local_to_global = cur_rotation_local_to_global.toRotationMatrix();
//    Eigen::Vector3d eulerAngle_yrp = rotation_mat_local_to_global.eulerAngles(2, 1, 0);// R_z*R_y*R_x
//    rotation_mat_local_to_global = Eigen::AngleAxisd(eulerAngle_yrp(0), Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // 遍历点云中的所有点
    Eigen::Vector3d p3d;
    vector<Eigen::Vector3d> obstacles;
    for (size_t i = 0; i < latest_local_pcl_.points.size(); ++i) 
    {
        p3d(0) = latest_local_pcl_.points[i].x;
        p3d(1) = latest_local_pcl_.points[i].y;
        p3d(2) = latest_local_pcl_.points[i].z; // World_ENU frame

        //　不考虑地面上的点的排斥力
        if(fabs(p3d(2))<ground_height)
            continue;

        //　超出最大考虑距离，则忽略该点
        Eigen::Vector3d uav2obs = p3d - current_pos; 
        double dist_push = (uav2obs).norm();
        if(dist_push > sensor_max_range || isnan(dist_push))
            continue;

        double obs_dist = uav2obs.norm();
        if (is_2D || true){
		    if (fabs(uav2obs(2))>0.5) continue;
		    uav2obs(2) = 0.0;
		    double obs_angle = sign(uav2obs(1)) * acos(uav2obs(0) / uav2obs.norm());// angle_cen: -pi ~ pi
		    double angle_range;// angle_range: 0~pi/2
		    if(obs_dist>inflate_and_safe_distance)
		        angle_range = asin(inflate_and_safe_distance/obs_dist);
		    else if (obs_dist<=inflate_and_safe_distance)
		    {
		        angle_range = M_PI/2;
		        safe_cnt++;  // 非常危险
		    }
		    PolarCoordinateVFH(obs_angle, angle_range, max(obs_dist-inflate_and_safe_distance,0.0));
		}

        obstacles.push_back(p3d);
    }
    cout << "size of obs considered: " << obstacles.size() << " w.s.t: " << latest_local_pcl_.points.size() << endl;


    // 目标点&当前速度 相关
    if (is_2D || true){
		uav2goal(2) = 0.0;
		double goal_heading = sign(uav2goal(1)) * acos(uav2goal(0)/uav2goal.norm()); // -pi ~ pi
		double hor_current_vel_norm = Eigen::Vector3d(current_vel(0),current_vel(1),0.0).norm();
		double current_heading = sign(current_vel(1)) *acos(current_vel(0)/hor_current_vel_norm); // -pi ~ pi

		for(int i=0; i<Hcnt; i++)
		{
		    double angle_i = (i + 0.5)* Hres; // 0 ~ 2pi
		    double angle_err_goal = fabs(angle_i - (goal_heading<0 ? goal_heading+2*M_PI : goal_heading)); // 0 ~ pi
			double angle_err_vel = fabs(angle_i - (current_heading<0 ? current_heading+2*M_PI : current_heading)); // 0 ~ pi
		    // 角度差的越小越好
			if (hor_current_vel_norm>1e-6)
		    	Histogram[i] *= sqrt((cos(angle_err_goal)+1)/2 + (cos(angle_err_vel)+1)/2 * pow(hor_current_vel_norm/limit_v_norm,2));
			else
				Histogram[i] *= sqrt((cos(angle_err_goal)+1)/2);
		}

		// 寻找最优方向， 2d速度恒定
		int best_ind = find_optimization_path();
		if (best_ind == -1)
			desired_vel = Eigen::Vector3d(0.0,0.0,0.0);
		else {
			double best_heading  = (best_ind + 0.5)* Hres;
			desired_vel(0) = cos(best_heading)*limit_v_norm;
			desired_vel(1) = sin(best_heading)*limit_v_norm;
			desired_vel(2) = 0.0;
		}
	}






    // 如果不安全的点超出指定数量
    if(safe_cnt>5)
        local_planner_state = 2;  //成功规划，但是飞机不安全
    else
        local_planner_state =1;  //成功规划， 安全

    exec_num++;
	end = std::chrono::system_clock::now();
    if(exec_num == 10)
    {
    	std::chrono::duration<double> elapsed_seconds = end - start; 
        printf("VFH calculation takes %f [us].\n", elapsed_seconds.count()/10*1e6);
        exec_num=0;
    }
    return local_planner_state;
}

// 寻找最小
int VFH::find_optimization_path(void)
{
    int bset_ind = -1;
    double best_value = 0;
    for(int i=0; i<Hcnt; i++){
        if(Histogram[i]>best_value){
            best_value = Histogram[i];
            bset_ind = i;
        }
    }
    return bset_ind;
}

// angle_cen: -pi ~ pi; angle_range: 0~pi/2
void VFH::PolarCoordinateVFH(double angle_cen, double angle_range, double val)
{
	angle_cen = angle_cen<0 ? angle_cen+2*M_PI : angle_cen; // 0 ~ 2pi
    double angle_max = angle_cen + angle_range;
    double angle_min = angle_cen - angle_range; // -pi/2 ~ 5 pi/2
    int cnt_min = floor((angle_min<0 ? angle_min+2*M_PI : angle_min>2*M_PI ? angle_min-2*M_PI : angle_min)/Hres);
    int cnt_max = floor((angle_max<0 ? angle_max+2*M_PI : angle_max>2*M_PI ? angle_max-2*M_PI : angle_max)/Hres);
    if(cnt_min>cnt_max)
    {
        for(int i=cnt_min; i<Hcnt; i++)
        {
            if (Histogram[i] > val) Histogram[i] = val;
        }
        for(int i=0;i<cnt_max; i++)
        {
            if (Histogram[i] > val) Histogram[i] =val;
        }
    }else
    {
        for(int i=cnt_min; i<=cnt_max; i++)
        {
            if (Histogram[i] > val) Histogram[i] = val;
        }
    }
     
}

}
