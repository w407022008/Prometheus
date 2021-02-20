#include "local_planning.h"
#include <string> 	
#include <time.h>

namespace Local_Planning
{

// 局部规划算法 初始化函数
void Local_Planner::init(ros::NodeHandle& nh)
{
    // 参数读取
    nh.param("local_planner/inflate_distance", inflate_distance, 0.20);  // 最小障碍物距离
    // 规划器使能
    nh.param("local_planner/planner_enable", planner_enable_default, false);
    // 根据参数 planning/algorithm_mode 选择局部避障算法: [0]: APF,[1]: VFH
    nh.param("local_planner/algorithm_mode", algorithm_mode, 0);
	// 0代表建图数据类型octomap<sensor_msgs::PointCloud2>,1代表2d传感器数据类型<sensor_msgs::LaserScan>,2代表3d传感器数据类型<sensor_msgs::PointCloud2>
    nh.param("local_planner/map_input", map_input, 0);
    nh.param("local_planner/ground_removal", flag_pcl_ground_removal, false);
    nh.param("local_planner/max_ground_height", max_ground_height, 0.1);
    nh.param("local_planner/downsampling", flag_pcl_downsampling, false);
    nh.param("local_planner/resolution", size_of_voxel_grid, 0.1);
    nh.param("local_planner/timeSteps_fusingSamples", timeSteps_fusingSamples, 4);
    // TRUE代表2D平面规划及搜索,FALSE代表3D 
    nh.param("local_planner/is_2D", is_2D, false); 
    nh.param("local_planner/is_rgbd", is_rgbd, false); 
    nh.param("local_planner/is_lidar", is_lidar, false); 
    if (is_2D) {
    	is_rgbd = false; is_lidar = false;
    }
    // 如果采用2维Lidar，需要一定的yawRate来探测地图
    nh.param("local_planner/control_yaw_flag", control_yaw_flag, false); 
    // 2D规划时,定高高度
    nh.param("local_planner/fly_height_2D", fly_height_2D, 1.0);  
    // 是否为仿真模式
    nh.param("local_planner/sim_mode", sim_mode, false); 
    // 最大速度
    nh.param("local_planner/max_planning_vel", max_planning_vel, 0.4);

    // 订阅目标点
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/planning/goal", 1, &Local_Planner::goal_cb, this);

    // 订阅开关
    planner_enable = planner_enable_default;
    planner_switch_sub = nh.subscribe<std_msgs::Bool>("/prometheus/switch/local_planner", 10, &Local_Planner::planner_switch_cb, this);

    // 订阅 无人机状态
    drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, &Local_Planner::drone_state_cb, this);

    // 订阅传感器点云信息,该话题名字可在launch文件中任意指定
    if (map_input == 0)
    {
        local_point_clound_sub = nh.subscribe<sensor_msgs::PointCloud2>("/prometheus/planning/local_pcl", 1, &Local_Planner::localcloudCallback, this);
    }else if (map_input == 1)
    {
        local_point_clound_sub = nh.subscribe<sensor_msgs::LaserScan>("/prometheus/planning/local_pcl", 1, &Local_Planner::Callback_2dlaserscan, this);
    }else if (map_input == 2)
    {
        local_point_clound_sub = nh.subscribe<sensor_msgs::PointCloud2>("/prometheus/planning/local_pcl", 1, &Local_Planner::Callback_3dpointcloud, this);
    }

    // 发布 期望速度
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
    
    // 发布提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/local_planner", 10);

    // 发布速度用于显示
    rviz_vel_pub = nh.advertise<geometry_msgs::Point>("/prometheus/local_planner/desired_vel", 10); 
    
    // 发布局部点云用于显示
    point_cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/prometheus/local_planner/local_point_cloud", 10); 

    // 定时函数,执行周期为1Hz
    mainloop_timer = nh.createTimer(ros::Duration(0.2), &Local_Planner::mainloop_cb, this);

    // 控制定时器
    control_timer = nh.createTimer(ros::Duration(0.05), &Local_Planner::control_cb, this);

    // 选择避障算法
    if(algorithm_mode==0){
        local_alg_ptr.reset(new APF);
        local_alg_ptr->init(nh);
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "APF init.");
    }
    else if(algorithm_mode==1)
    {
        local_alg_ptr.reset(new VFH);
        local_alg_ptr->init(nh);
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "VFH init.");
    }

    // 规划器状态参数初始化
    exec_state = EXEC_STATE::WAIT_GOAL;
    odom_ready = false;
    drone_ready = false;
    goal_ready = false;
    sensor_ready = false;
    path_ok = false;

    // 初始化发布的指令
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
    Command_Now.Command_ID = 0;
    Command_Now.source = NODE_NAME;
    desired_yaw = 0.0;

    //　仿真模式下直接发送切换模式与起飞指令
    if(sim_mode == true)
    {
        // Waiting for input
        int start_flag = 0;
        while(start_flag == 0)
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Local Planner<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please input 1 for start:"<<endl;
            cin >> start_flag;
        }
        // 起飞
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(3.0).sleep();
        
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = prometheus_msgs::ControlCommand::Takeoff;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();
    }else
    {
        //　真实飞行情况：等待飞机状态变为offboard模式，然后发送起飞指令
    }

    // 地图初始化
    sensor_msgs::PointCloud2ConstPtr init_local_map(new sensor_msgs::PointCloud2());
    local_map_ptr_ = init_local_map;

    ros::spin();
}

void Local_Planner::planner_switch_cb(const std_msgs::Bool::ConstPtr& msg)
{
    if (!planner_enable && msg->data){
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,"Planner is enable.");
    }else if (planner_enable && !msg->data){
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,"Planner is disable.");
        exec_state = EXEC_STATE::WAIT_GOAL;
    }
    planner_enable = msg->data;
}

void Local_Planner::goal_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if (is_2D == true)
    {
        goal_pos << msg->pose.position.x, msg->pose.position.y, fly_height_2D;
    }else
    {
    	if(msg->pose.position.z<1)
        	goal_pos << msg->pose.position.x, msg->pose.position.y, 1.0;
        else
        	goal_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    }

    goal_ready = true;

    // 获得新目标点
    if(planner_enable){
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,"Get a new goal point");

        cout << "Get a new goal point:"<< goal_pos(0) << " [m] "  << goal_pos(1) << " [m] "  << goal_pos(2)<< " [m] "   <<endl;

        if(goal_pos(0) == 99 && goal_pos(1) == 99 )
        {
            path_ok = false;
            goal_ready = false;
            exec_state = EXEC_STATE::LANDING;
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,"Land");
        }
    }
}

void Local_Planner::drone_state_cb(const prometheus_msgs::DroneStateConstPtr& msg)
{
    _DroneState = *msg; // ENU系

    if (is_2D == true)
    {
        start_pos << msg->position[0], msg->position[1], fly_height_2D;
        start_vel << msg->velocity[0], msg->velocity[1], 0.0;

        if(abs(fly_height_2D - msg->position[2]) > 0.2)
        {
            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME,"Drone is not in the desired height.");
        }
    }else
    {
        start_pos << msg->position[0], msg->position[1], msg->position[2];
        start_vel << msg->velocity[0], msg->velocity[1], msg->velocity[2];
    }

    odom_ready = true;

    if (_DroneState.connected == true && _DroneState.armed == true )
    {
        drone_ready = true;
    }else
    {
        drone_ready = false;
    }

    Drone_odom.header = _DroneState.header;
    Drone_odom.child_frame_id = "base_link";

    Drone_odom.pose.pose.position.x = _DroneState.position[0];
    Drone_odom.pose.pose.position.y = _DroneState.position[1];
    Drone_odom.pose.pose.position.z = _DroneState.position[2];

    Drone_odom.pose.pose.orientation = _DroneState.attitude_q;
    Drone_odom.twist.twist.linear.x = _DroneState.velocity[0];
    Drone_odom.twist.twist.linear.y = _DroneState.velocity[1];
    Drone_odom.twist.twist.linear.z = _DroneState.velocity[2];

    local_alg_ptr->set_odom(Drone_odom);

}


void Local_Planner::Callback_2dlaserscan(const sensor_msgs::LaserScanConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }
    
	tf::StampedTransform transform;
	if (is_2D)
		try{
			tfListener.waitForTransform("/map","/lidar_link",msg->header.stamp,ros::Duration(4.0));
			tfListener.lookupTransform("/map", "/lidar_link", msg->header.stamp, transform);
		}
			catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
	
	tf::Quaternion q = transform.getRotation();
	tf::Vector3 Origin = tf::Vector3(transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ());
	
    double roll,pitch,yaw;
    tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
    Eigen::Matrix3f R_Body_to_ENU = get_rotation_matrix(roll, pitch, yaw);
    
    sensor_msgs::LaserScan::ConstPtr _laser_scan;
    _laser_scan = msg;

    pcl::PointCloud<pcl::PointXYZ> _pointcloud;
    _pointcloud.clear();
    
    pcl::PointXYZ newPoint;
    Eigen::Vector3f _laser_point_body_body_frame,_laser_point_body_ENU_frame;
    
    double newPointAngle;
    int beamNum = _laser_scan->ranges.size();
    for (int i = 0; i < beamNum; i++)
    {
    	if(_laser_scan->ranges[i] < inflate_distance) continue;
        newPointAngle = _laser_scan->angle_min + _laser_scan->angle_increment * i;
        _laser_point_body_body_frame[0] = _laser_scan->ranges[i] * cos(newPointAngle);
        _laser_point_body_body_frame[1] = _laser_scan->ranges[i] * sin(newPointAngle);
        _laser_point_body_body_frame[2] = 0.0;
        _laser_point_body_ENU_frame = R_Body_to_ENU * _laser_point_body_body_frame;
        newPoint.x = Origin.getX() + _laser_point_body_ENU_frame[0];
        newPoint.y = Origin.getY() + _laser_point_body_ENU_frame[1];
        newPoint.z = Origin.getZ() + _laser_point_body_ENU_frame[2];
        
        _pointcloud.push_back(newPoint);
    }
	concatenate_PointCloud += _pointcloud;
	
	static int frame_id = 0;
	if (frame_id == timeSteps_fusingSamples){
//		cout << "point cloud size: " << ", " << (int)concatenate_PointCloud.points.size();
		if(flag_pcl_ground_removal){
			
			pcl::PassThrough<pcl::PointXYZ> ground_removal;
			ground_removal.setInputCloud (concatenate_PointCloud.makeShared());
			ground_removal.setFilterFieldName ("z");
			ground_removal.setFilterLimits (-1.0, max_ground_height);
			ground_removal.setFilterLimitsNegative (true);
			ground_removal.filter (concatenate_PointCloud);
		}
		
		if (flag_pcl_downsampling){
			pcl::VoxelGrid<pcl::PointXYZ> sor;
			sor.setInputCloud(concatenate_PointCloud.makeShared());
			sor.setLeafSize(size_of_voxel_grid, size_of_voxel_grid, size_of_voxel_grid);
			sor.filter(concatenate_PointCloud);
		}
//		cout << " to " << (int)concatenate_PointCloud.points.size() << endl;
		
		local_point_cloud = concatenate_PointCloud;
		frame_id = 0;
		concatenate_PointCloud.clear();
	} else {
		local_point_cloud = local_point_cloud;
		frame_id++;
	}

	local_point_cloud.header.seq++;
	local_point_cloud.header.stamp = (msg->header.stamp).toNSec()/1e3;
	local_point_cloud.header.frame_id = "/map";
	point_cloud_pub.publish(local_point_cloud);

	pcl_ptr = local_point_cloud.makeShared();
    local_alg_ptr->set_local_map_pcl(pcl_ptr);
    
    sensor_ready = true;

}

void Local_Planner::Callback_3dpointcloud(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready || (!is_rgbd && !is_lidar)) 
    {
        return;
    }

	tf::StampedTransform transform;
	if (is_rgbd)
		try{
			tfListener.waitForTransform("/map","/realsense_camera_link",msg->header.stamp,ros::Duration(4.0));
			tfListener.lookupTransform("/map", "/realsense_camera_link", msg->header.stamp, transform);
		}
			catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

	if (is_lidar)
		try{
			tfListener.waitForTransform("/map","/3Dlidar_link",msg->header.stamp,ros::Duration(4.0));
			tfListener.lookupTransform("/map", "/3Dlidar_link", msg->header.stamp, transform);
		}
			catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
	

	tf::Quaternion q = transform.getRotation();
	tf::Matrix3x3 Rotation(q);

	pcl::fromROSMsg(*msg, latest_local_pcl_);
	
    pcl::PointCloud<pcl::PointXYZ> _pointcloud;

    _pointcloud.clear();
    pcl::PointXYZ newPoint;
    tf::Vector3 _laser_point_body_body_frame,_laser_point_body_ENU_frame;
    
    for (int i = 0; i < (int)latest_local_pcl_.points.size(); i++)
    {
        _laser_point_body_body_frame[0] = latest_local_pcl_.points[i].x;
        _laser_point_body_body_frame[1] = latest_local_pcl_.points[i].y;
        _laser_point_body_body_frame[2] = latest_local_pcl_.points[i].z;
        _laser_point_body_ENU_frame = Rotation * _laser_point_body_body_frame;
        newPoint.x = transform.getOrigin().getX() + _laser_point_body_ENU_frame[0];
        newPoint.y = transform.getOrigin().getY() + _laser_point_body_ENU_frame[1];
        newPoint.z = transform.getOrigin().getZ() + _laser_point_body_ENU_frame[2];
        
        _pointcloud.push_back(newPoint);
    }
	concatenate_PointCloud += _pointcloud;
	
	static int frame_id = 0;
	if (frame_id == timeSteps_fusingSamples){
		if(flag_pcl_ground_removal){
			
			pcl::PassThrough<pcl::PointXYZ> ground_removal;
			ground_removal.setInputCloud (concatenate_PointCloud.makeShared());
			ground_removal.setFilterFieldName ("z");
			ground_removal.setFilterLimits (-1.0, max_ground_height);
			ground_removal.setFilterLimitsNegative (true);
			ground_removal.filter (concatenate_PointCloud);
		}
		
		if (flag_pcl_downsampling){
			pcl::VoxelGrid<pcl::PointXYZ> sor;
			sor.setInputCloud(concatenate_PointCloud.makeShared());
			sor.setLeafSize(size_of_voxel_grid, size_of_voxel_grid, size_of_voxel_grid);
			sor.filter(concatenate_PointCloud);
		}
		
		local_pcl_tm1 = concatenate_PointCloud;
		frame_id = 0;
		concatenate_PointCloud.clear();
		
		local_point_cloud = local_pcl_tm1;
		local_point_cloud += local_pcl_tm2;
		local_point_cloud += local_pcl_tm3;
		local_pcl_tm3 = local_pcl_tm2;
		local_pcl_tm2 = local_pcl_tm1;
		local_pcl_tm1.clear();
	} else {
		local_point_cloud = local_point_cloud;
		frame_id++;
	}

	
	local_point_cloud.header = latest_local_pcl_.header;
	local_point_cloud.header.frame_id = "/map";
//	local_point_cloud.height = 1;
//	local_point_cloud.width = local_point_cloud.points.size();
	point_cloud_pub.publish(local_point_cloud);

	
//	cout << "Rotation: " << endl;
//	cout << "[ [" << Rotation[0][0] << ", " << Rotation[0][1] << ", " << Rotation[0][2] << "]," << endl;
//	cout << "  [" << Rotation[1][0] << ", " << Rotation[1][1] << ", " << Rotation[1][2] << "]," << endl;
//	cout << "  [" << Rotation[2][0] << ", " << Rotation[2][1] << ", " << Rotation[2][2] << "] ]" << endl;
	cout << "local obstacle points size: " << (int)local_point_cloud.points.size() << endl;
//	cout << "header.seq: " << (int)local_point_cloud.header.seq << endl;
//	cout << "header.stamp: " << (int)local_point_cloud.header.stamp << endl;
//	cout << "header.frame_id: " << local_point_cloud.header.frame_id << endl;
//	cout << "height: " << (int)local_point_cloud.height << endl;
//	cout << "width: " << (int)local_point_cloud.width << endl;

	pcl_ptr = local_point_cloud.makeShared();
    local_alg_ptr->set_local_map_pcl(pcl_ptr);
    
    sensor_ready = true;

}

void Local_Planner::localcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }

    sensor_ready = true;

    local_map_ptr_ = msg;
    local_alg_ptr->set_local_map(local_map_ptr_);
}

void Local_Planner::control_cb(const ros::TimerEvent& e)
{
	if (!path_ok || !planner_enable_default){
		if (control_yaw_flag){
			Command_Now.header.stamp                        = ros::Time::now();
			Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
			Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
			Command_Now.source                              = NODE_NAME;
			Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XY_VEL_Z_POS;
			Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
		    Command_Now.Reference_State.position_ref[2]     = _DroneState.position[2];
			Command_Now.Reference_State.velocity_ref[0]     = 0.0;
			Command_Now.Reference_State.velocity_ref[1]     = 0.0;
			
			desired_yaw = desired_yaw + 0.05;
			Command_Now.Reference_State.yaw_ref             = desired_yaw;
		    command_pub.publish(Command_Now);
		}
        return;
	}

    distance_to_goal = Eigen::Vector3d((start_pos - goal_pos)[0],(start_pos - goal_pos)[1],0.0).norm();

    // 抵达终点
    if(distance_to_goal < MIN_DIS)
    {
        Command_Now.header.stamp                        = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = goal_pos[0];
        Command_Now.Reference_State.position_ref[1]     = goal_pos[1];
        Command_Now.Reference_State.position_ref[2]     = goal_pos[2];

        Command_Now.Reference_State.yaw_ref             = desired_yaw;
        command_pub.publish(Command_Now);
        
        if (!planner_enable_default)
            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Reach the goal! The planner will be disable automatically.");
        else
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Reach the goal! The planner is still enable.");
        
        // 停止执行
        path_ok = false;
        planner_enable = planner_enable_default;
        // 转换状态为等待目标
        exec_state = EXEC_STATE::WAIT_GOAL;
        return;
    }

    if (is_2D)
    {
		Command_Now.header.stamp                        = ros::Time::now();
		Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
		Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
		Command_Now.source                              = NODE_NAME;
		Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XY_VEL_Z_POS;
		Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
		Command_Now.Reference_State.velocity_ref[0]     = desired_vel[0];
		Command_Now.Reference_State.velocity_ref[1]     = desired_vel[1];
		Command_Now.Reference_State.position_ref[2]     = fly_height_2D;
    }else{
		Command_Now.header.stamp                        = ros::Time::now();
		Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
		Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
		Command_Now.source                              = NODE_NAME;
		Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_VEL;
		Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
		Command_Now.Reference_State.velocity_ref[0]     = desired_vel[0];
		Command_Now.Reference_State.velocity_ref[1]     = desired_vel[1];
		Command_Now.Reference_State.velocity_ref[2]     = desired_vel[2];
    }
		

    // 更新期望偏航角
    if (control_yaw_flag)
    {
        auto sign=[](double v)->double
        {
            return v<0.0? -1.0:1.0;
        };
        Eigen::Vector3d ref_vel;
        ref_vel[0] = desired_vel[0];
        ref_vel[1] = desired_vel[1];
        //ref_vel[2] = desired_vel[2];

        if( sqrt( ref_vel[1]* ref_vel[1] + ref_vel[0]* ref_vel[0])  >  0.05  )
        {
        	float next_desired_yaw_vel = sign(ref_vel(1)) * acos(ref_vel(0) / ref_vel.norm());
//			cout << "desired_yaw " << desired_yaw << ", next_desired_yaw_vel " << next_desired_yaw_vel << endl;
	
            if (fabs(desired_yaw-next_desired_yaw_vel)<M_PI)
            	desired_yaw = (0.3*desired_yaw + 0.7*next_desired_yaw_vel);
            else
            	desired_yaw = next_desired_yaw_vel + sign(next_desired_yaw_vel) * 0.3/(0.3+0.7)*(2*M_PI-fabs(desired_yaw-next_desired_yaw_vel));
        } else {
            desired_yaw = desired_yaw + 0.05;
        }
    	if(desired_yaw>M_PI)
    		desired_yaw -= 2*M_PI;
    	else if (desired_yaw<-M_PI)
    		desired_yaw += 2*M_PI;
    		
    }else
    {
        desired_yaw = 0.0;
    }

    Command_Now.Reference_State.yaw_ref             = desired_yaw;

    command_pub.publish(Command_Now);

}

void Local_Planner::mainloop_cb(const ros::TimerEvent& e)
{
    static int exec_num=0;
    exec_num++;

    // 检查当前状态，不满足规划条件则直接退出主循环
    // 此处打印消息与后面的冲突了，逻辑上存在问题
    if(!odom_ready || !drone_ready || !sensor_ready || !planner_enable)
    {
        // 此处改为根据循环时间计算的数值
        if(exec_num == 10)
        {
            if(!planner_enable)
            {
                message = "Planner is disable by default! If you want to enable it, pls set the param [local_planner/enable] as true!";
            }else if(!odom_ready)
            {
                message = "Need Odom.";
            }else if(!drone_ready)
            {
                message = "Drone is not ready.";
            }else if(!sensor_ready)
            {
                message = "Need sensor info.";
            } 

            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
            exec_num=0;
        }  

        return;
    }else
    {
        // 对检查的状态进行重置
        odom_ready = false;
        drone_ready = false;
        sensor_ready = false;
    }

    switch (exec_state)
    {
        case WAIT_GOAL:
        {
            path_ok = false;
            if(!goal_ready)
            {
                if(exec_num == 20)
                {
                    message = "Waiting for a new goal.";
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME,message);
                    exec_num=0;
                }
            }else
            {
                // 获取到目标点后，生成新轨迹
                exec_state = EXEC_STATE::PLANNING;
                goal_ready = false;
            }
            
            break;
        }
        case PLANNING:
        {
            // desired_vel是返回的规划速度；如果planner_state为2时,飞机不安全(距离障碍物太近)
            planner_state = local_alg_ptr->compute_force(goal_pos, desired_vel);

            path_ok = true;

            //　对规划的速度进行限幅处理
            if(desired_vel.norm() > max_planning_vel)
            {
                desired_vel = desired_vel / desired_vel.norm() * max_planning_vel; 
            }

            //　发布rviz显示
            vel_rviz.x = desired_vel(0);
            vel_rviz.y = desired_vel(1);
            vel_rviz.z = desired_vel(2);
            rviz_vel_pub.publish(vel_rviz);

            if(exec_num==100)
            {
                if(planner_state == 1)
                {
                    message = "local planning desired vel: [" + std::to_string(desired_vel(0)) + "," + std::to_string(desired_vel(1)) + "," + std::to_string(desired_vel(2)) + "]";
                }else if(planner_state == 2)
                {
                    message = "Dangerous!";
                }
                
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);
                exec_num=0;
            }

            break;
        }
        case  LANDING:
        {
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode         = prometheus_msgs::ControlCommand::Land;
            Command_Now.Command_ID   = Command_Now.Command_ID + 1;
            Command_Now.source = NODE_NAME;

            command_pub.publish(Command_Now);
            break;
        }
    }

}

}


