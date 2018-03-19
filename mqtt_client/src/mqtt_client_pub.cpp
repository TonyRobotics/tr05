#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/serialization.h>

#include <iostream>
#include <cstdlib>
#include <string>
#include <thread>	// For sleep
#include <chrono>
#include <cstring>
#include "mqtt/async_client.h"
#include <fstream>

#include "pthread.h"
#include <stdlib.h>

#include "std_msgs/Empty.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> SetGoalClient;

#include <actionlib_msgs/GoalID.h>

#include <nav_msgs/Path.h>
#include <patrol_msgs/patrol_data.h>

ros::NodeHandle *nh;
ros::Subscriber sub_map;
ros::Subscriber sub_scan;
ros::Subscriber sub_status_vel;
ros::Subscriber sub_odom;
ros::Subscriber sub_motor_status;
ros::Publisher pub_control_vel;
ros::Publisher pub_nav_goal;
ros::Publisher pub_set_pose;
ros::Publisher pub_artificial_path;
// internals
ros::Publisher pub_get_map;
ros::Subscriber sub_get_map;
ros::Publisher pub_arrived;
ros::Subscriber sub_arrived;
ros::Publisher pub_reading;
ros::Subscriber sub_reading;

ros::Publisher pub_get_task_list;
ros::Subscriber sub_get_task_list;
ros::Publisher pub_get_task;
ros::Subscriber sub_get_task;

ros::Publisher pub_cancel_goal;

bool stop_task_flag;
bool finish_reading_flag;

laser_geometry::LaserProjection *projector_;
tf::TransformListener *listener_;
SetGoalClient *set_goal_client;

geometry_msgs::Point current_point_;

const int  QOS = 0;
const long TIMEOUT = 10000L;

class action_listener : public virtual mqtt::iaction_listener{
    std::string name_;
    virtual void on_failure(const mqtt::itoken& tok) {
    std::cout << name_ << " failure";
    if (tok.get_message_id() != 0)
        std::cout << " (token: " << tok.get_message_id() << ")" << std::endl;
    std::cout << std::endl;
}
virtual void on_success(const mqtt::itoken& tok) {
    std::cout << name_ << " success";
    if (tok.get_message_id() != 0)
        std::cout << " (token: " << tok.get_message_id() << ")" << std::endl;
    if (!tok.get_topics().empty())
        std::cout << "\ttoken topic: '" << tok.get_topics()[0] << "', ..." << std::endl;
    std::cout << std::endl;
}
public:
    action_listener(const std::string& name) : name_(name) {}
};

struct thread_data{
    int thread_id;
    mqtt::message_ptr message;
};

#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
int scanFileNames(std::string dir, std::vector<std::string> &file_names){
    struct dirent *ent = NULL;  
    DIR *pDir;
    if((pDir = opendir(dir.c_str())) != NULL){  
        while(NULL != (ent = readdir(pDir))){  
            if(ent->d_type == 8){
                //printf("File:\t%s/%s\n", dir.c_str(), ent->d_name);
                file_names.push_back(ent->d_name);
            }
        }  
        closedir(pDir);  
    }  
    else{  
        return -1;
    }
    return 0;
}
#include <unistd.h>
bool is_file_exist(const char* file_path){
    if (file_path ==NULL){
        return false;
    }
    if (access(file_path, F_OK) == 0)
        return true;
    return false;
}

void *thread_nav_goal(void *threadarg){
    struct thread_data *my_data;
    my_data = (struct thread_data *) threadarg;
    mqtt::message_ptr msg = my_data->message; 
    /********************/
    uint8_t * payloadptr = (uint8_t *)msg->get_payload().c_str();
    uint32_t serial_size = msg->get_payload().size();
    ros::serialization::IStream stream(payloadptr, serial_size);
    geometry_msgs::Pose pose;
    ros::serialization::deserialize(stream, pose);
    geometry_msgs::PoseStamped nav_goal;
    nav_goal.pose = pose;
    nav_goal.header.frame_id = "map";
    nav_goal.header.stamp = ros::Time::now();
    ROS_INFO("Received nav_goal: (%f, %f)", nav_goal.pose.position.x, nav_goal.pose.position.y);
    // pub_nav_goal.publish(nav_goal);
    set_goal_client->waitForServer();
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = nav_goal;
    set_goal_client->sendGoal(goal);
    set_goal_client->waitForResult(ros::Duration(1000.0));
    if (set_goal_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        std_msgs::Empty msg_arrived;
        pub_arrived.publish(msg_arrived);
    }
    /********************/
    my_data->thread_id = 0;
    pthread_exit(NULL);
}

void *thread_set_pose(void *threadarg){
    struct thread_data *my_data;
    my_data = (struct thread_data *) threadarg;
    mqtt::message_ptr msg = my_data->message; 
    /********************/
    uint8_t * payloadptr = (uint8_t *)msg->get_payload().c_str();
    uint32_t serial_size = msg->get_payload().size();
    ros::serialization::IStream stream(payloadptr, serial_size);
    geometry_msgs::Pose pose;
    ros::serialization::deserialize(stream, pose);
    std::string ss = msg->get_payload();
    geometry_msgs::PoseWithCovarianceStamped robot_pose;
    robot_pose.header.frame_id = "map";
    robot_pose.header.stamp = ros::Time::now();
    robot_pose.pose.pose = pose;
    pub_set_pose.publish(robot_pose);
    /********************/
    my_data->thread_id = 0;
    pthread_exit(NULL);
}

int deserializePatrolData(char *sp, patrol_msgs::patrol_data &patrol_data){
    uint32_t pi = 0;
    uint32_t name_size = *(uint32_t *) sp; pi+=4;
    std::string name_str(sp+pi, name_size); pi+=name_size;
    patrol_data.name.data = name_str;
    uint32_t pt_size = *(uint32_t *) (sp+pi); pi+=4;
    geometry_msgs::Point ptmp;
    for(unsigned int i = 0; i < pt_size; ++i){
       ptmp.x = *(double *) (sp+pi); pi+=8;
       ptmp.y = *(double *) (sp+pi); pi+=8;
       ptmp.z = *(double *) (sp+pi); pi+=8;
       patrol_data.positions.push_back(ptmp);
    }
    patrol_msgs::patrol_node ntmp;
    uint32_t node_size = *(uint32_t *) (sp+pi); pi += 4;
    //uint32_t node_size;
    //node_size = ((uint32_t)(*(sp + pi)));
    //node_size |= ((uint32_t)(*(sp + pi + 1))) << (8 * 1);
    //node_size |= ((uint32_t)(*(sp + pi + 2))) << (8 * 2);
    //node_size |= ((uint32_t)(*(sp + pi + 3))) << (8 * 3);

    printf("point size: %d\n", pt_size);
    printf("node size: %d\n", node_size);
    for(unsigned int i = 0; i < node_size; ++i){
        ntmp.index = *(uint32_t *) (sp+pi); pi+=4;
	ntmp.orientation.x = *(double *) (sp+pi); pi+=8;
	ntmp.orientation.y = *(double *) (sp+pi); pi+=8;
	ntmp.orientation.z = *(double *) (sp+pi); pi+=8;
	ntmp.orientation.w = *(double *) (sp+pi); pi+=8;
	uint32_t str_size = *(uint32_t *) (sp+pi); pi+=4;
	std::string stmp(sp+pi, str_size); pi+=str_size;
        ntmp.reading.data = stmp;
	patrol_data.nodes.push_back(ntmp);
    }
    return 0;
}

int loadTask(std::string task_name, patrol_msgs::patrol_data &patrol_data){
    printf("start load task file: %s\n", task_name.c_str());  
    std::ifstream fin(task_name, std::ios::binary);
    std::istream::pos_type current_pos = fin.tellg();
    fin.seekg(0, std::ios_base::end);
    std::istream::pos_type file_size = fin.tellg();
    fin.seekg(current_pos);
    char *outbuffer = new char[file_size];
    fin.read((char*)outbuffer, file_size);
    fin.close();
    
    printf("read task file, size: %d, content: ", (int)file_size);  
    for(unsigned int i = 0; i< file_size; ++i){
        printf("%d,", outbuffer[i]);
    }
    printf("end\n");
    
    deserializePatrolData(outbuffer, patrol_data);
    delete [] outbuffer;
    return 0;
}

int saveTask(std::string str){
    const char *sp = str.c_str();
    uint32_t pi = 0;
    uint32_t name_size = *(uint32_t *) sp; pi+=4;
    std::string name_str(sp+pi, name_size);
    ROS_INFO("save task to: /home/odroid/trxb/tasks/%s.", name_str.c_str());
    // save task to file
    if(name_size>0){
	std::ofstream out("/home/odroid/trxb/tasks/" + name_str);  
        if (out.is_open()){  
            out << str;
            out.close();  
        }  
    }
    return 0;
}

void *thread_get_task(void *threadarg){
    struct thread_data *my_data;
    my_data = (struct thread_data *) threadarg;
    mqtt::message_ptr msg = my_data->message; 
    /********************/    
    std::string str = msg->get_payload();
    const char *sp = str.c_str();
    uint32_t pi = 0;
    uint32_t name_size = *(uint32_t *) sp; pi+=4;
    std::string name_str(sp+pi, name_size); pi+=name_size;
    //
    std_msgs::String msg_str;
    msg_str.data = name_str;
    pub_get_task.publish(msg_str); 
    /********************/
    my_data->thread_id = 0;
    pthread_exit(NULL);
}

void *thread_post_task(void *threadarg){
    struct thread_data *my_data;
    my_data = (struct thread_data *) threadarg;
    mqtt::message_ptr msg = my_data->message; 
    /********************/
    saveTask(msg->get_payload());
    ROS_INFO("save post task OK.");
    /********************/
    my_data->thread_id = 0;
    pthread_exit(NULL);
}
void *thread_do_task(void *threadarg){
    struct thread_data *my_data;
    my_data = (struct thread_data *) threadarg;
    mqtt::message_ptr msg = my_data->message; 
    /********************/    
    std::string str = msg->get_payload();
    const char *sp = str.c_str();
    uint32_t pi = 0;
    uint32_t name_size = *(uint32_t *) sp; pi+=4;
    std::string name_str(sp+pi, name_size); pi+=name_size;
    std::string file_name = "/home/odroid/trxb/tasks/" + name_str;
    printf("received task_name: %s\n", file_name.c_str());
    if(!is_file_exist(file_name.c_str())){
        printf("File does not exist: %s\n.", file_name.c_str());
        /********************/
        my_data->thread_id = 0;
        pthread_exit(NULL);
        printf("Exit without doing task.\n.");
	return NULL;
    }

    patrol_msgs::patrol_data patrol_data;
    loadTask(file_name, patrol_data);
    printf("deserialize task ok\n");

    printf("pt size: %d\n", (int)patrol_data.positions.size());
    for(unsigned int i = 0; i < patrol_data.positions.size(); ++i){
        printf("point %d: %f,%f\n", i, patrol_data.positions[i].x, patrol_data.positions[i].y);
    }
    
    printf("node size: %d\n", (int)patrol_data.nodes.size());
    for(unsigned int i = 0; i < patrol_data.nodes.size(); ++i){
        printf("text %d: %s, index: %d\n", i, patrol_data.nodes[i].reading.data.c_str(), patrol_data.nodes[i].index);
    }
    unsigned int curj = 0;
    stop_task_flag = false;
    for(unsigned int i = 0; i < patrol_data.nodes.size(); ++i){
	    if(stop_task_flag){
                ROS_INFO("patrol task is stopped.");
	        break;
	    }
        // path
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";
        geometry_msgs::PoseStamped p_tmp;
        p_tmp.header.frame_id = "map";
        p_tmp.pose.orientation.w = 1;
        p_tmp.pose.position = current_point_;
        path.poses.push_back(p_tmp);
        for(unsigned int j = curj; j <= patrol_data.nodes[i].index; ++j){
            p_tmp.pose.position = patrol_data.positions[j];
            p_tmp.pose.orientation = patrol_data.nodes[i].orientation;
            path.poses.push_back(p_tmp);
        }
	    curj = patrol_data.nodes[i].index + 1;
        ros::Rate loop_rate(2);
        int count = 0;
        ROS_INFO("start pub");
        while(ros::ok()){
            if(count > 1){
                break;
            }
            else{
                ROS_INFO("pub artificial path");
                pub_artificial_path.publish(path);
            }
            ros::spinOnce();
            loop_rate.sleep();
            ++count;
        }
 
        geometry_msgs::PoseStamped nav_goal;
        nav_goal.pose.position = patrol_data.positions[patrol_data.nodes[i].index];
        nav_goal.pose.orientation = patrol_data.nodes[i].orientation;
        nav_goal.header.frame_id = "map";
        nav_goal.header.stamp = ros::Time::now();
        set_goal_client->waitForServer();
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = nav_goal;
	    ROS_INFO("moving to nav_goal %d of %d: (%f, %f)", (int)i+1, (int)patrol_data.nodes.size(), goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
	    printf("orientation %d: (%f, %f, %f, %f)\n", (int)i, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w);
        set_goal_client->sendGoal(goal);
        set_goal_client->waitForResult(ros::Duration(1000.0));
        if (set_goal_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("arrived nav_goal");
    	    std_msgs::String msg = patrol_data.nodes[i].reading;
    	    msg.data = "words:[" + msg.data + "]";
            ROS_INFO("send reading.");
    	    pub_reading.publish(msg);
	    while(!finish_reading_flag && !stop_task_flag){
            ros::Duration(1.0).sleep();
	    }
	    finish_reading_flag = false;
            ROS_INFO("reading finished.");
        }
    }
    ROS_INFO("patrol task finished.");

    /********************/
    my_data->thread_id = 0;
    pthread_exit(NULL);
}

void *thread_set_nav_mode(void *threadarg){
    struct thread_data *my_data;
    my_data = (struct thread_data *) threadarg;
    mqtt::message_ptr msg = my_data->message; 
    /********************/
    std::string str = msg->get_payload();
    const char *sp = str.c_str();
    uint32_t pi = 0;
    uint32_t name_size = *(uint32_t *) sp; pi+=4;
    std::string name_str(sp+pi, name_size);
    ROS_INFO("Received set_nav_mode: %s", name_str.c_str());
    if("auto" == name_str){
        nh->setParam("/move_base/base_global_planner", "navfn/NavfnROS");
        ROS_INFO("set nav_mode to navfn/NavfnROS");
    }
    else if("patrol" == name_str){
        nh->setParam("/move_base/base_global_planner", "patrol_planner/PatrolPlanner");
        ROS_INFO("set nav_mode to patrol_planner/PatrolPlanner");
    }
    /********************/
    my_data->thread_id = 0;
    pthread_exit(NULL);
}

void *thread_xiaobai_adb(void *threadarg){
    struct thread_data *my_data;
    my_data = (struct thread_data *) threadarg;
    mqtt::message_ptr msg = my_data->message; 
    /********************/
    std::string str = msg->get_payload();
    const char *sp = str.c_str();
    uint32_t pi = 0;
    uint32_t cmd_size = *(uint32_t *) sp; pi+=4;
    std::string cmd_str(sp+pi, cmd_size);
    cmd_str = "adb " + cmd_str;
    ROS_INFO("Execute xiaobai_adb command: %s", cmd_str.c_str());
    system(cmd_str.c_str());
    ROS_INFO("Execute command ok.");
    /********************/
    my_data->thread_id = 0;
    pthread_exit(NULL);
}

void *thread_cast_ctrl_ready(void *threadarg){
    struct thread_data *my_data;
    my_data = (struct thread_data *) threadarg;
    mqtt::message_ptr msg = my_data->message; 
    /********************/

    std::string str = msg->get_payload();
    const char *sp = str.c_str();
    uint32_t pi = 0;
    uint32_t cmd_size = *(uint32_t *) sp; pi+=4;
    std::string cmd_str(sp+pi, cmd_size);
    cmd_str = "adb " + cmd_str;
    ROS_INFO("Execute cast_ctrl_ready command: %s", cmd_str.c_str());
    system(cmd_str.c_str());
    ROS_INFO("Execute command ok.");
    /********************/
    my_data->thread_id = 0;
    pthread_exit(NULL);
}

class callback : public virtual mqtt::callback,
		         public virtual mqtt::iaction_listener{
    int nretry_;
    mqtt::async_client& cli_;
    action_listener& listener_;
    pthread_t threads[7];
    struct thread_data td[7];

    void reconnect() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        mqtt::connect_options connOpts;
        connOpts.set_keep_alive_interval(20);
        connOpts.set_clean_session(true);
        try {
            cli_.connect(connOpts, nullptr, *this);
        }
        catch (const mqtt::exception& exc) {
            std::cerr << "Error: " << exc.what() << std::endl;
            exit(1);
        }
    }
    // Re-connection failure
    virtual void on_failure(const mqtt::itoken& tok) {
        std::cout << "Reconnection failed." << std::endl;
        if (++nretry_ > 5)
            exit(1);
        reconnect();
    }
    // Re-connection success
    virtual void on_success(const mqtt::itoken& tok) {
        std::cout << "Reconnection success" << std::endl;;
        cli_.subscribe("helloooooo", QOS, nullptr, listener_);
    }
    virtual void connection_lost(const std::string& cause) {
        std::cout << "\nConnection lost" << std::endl;
        if (!cause.empty())
            std::cout << "\tcause: " << cause << std::endl;
        std::cout << "Reconnecting." << std::endl;
        nretry_ = 0;
        reconnect();
    }
    virtual void message_arrived(const std::string& topic, mqtt::message_ptr msg) {
        if(topic == "control_vel"){
            ROS_INFO("Received control_vel");
            /********************/
            uint8_t * payloadptr = (uint8_t *)msg->get_payload().c_str();
            uint32_t serial_size = msg->get_payload().size();
            ros::serialization::IStream stream(payloadptr, serial_size);
            geometry_msgs::Twist control_vel;
            ros::serialization::deserialize(stream, control_vel);
            control_vel.linear.x -= 100;
            control_vel.angular.z -= 100;
            ROS_INFO("deserialized vel: (%f, %f)", control_vel.linear.x, control_vel.angular.z);
            pub_control_vel.publish(control_vel); 
        }
        int i = 0;//0
        if(topic == "nav_goal"){
            ROS_INFO("Received nav_goal");
            if(td[i].thread_id == 0){
                td[i].thread_id = i+1;
                td[i].message = msg;
	            pthread_create(&threads[i], NULL, thread_nav_goal, (void*)&td[i]);
            }
        }
        if(topic == "get_map"){
            ROS_INFO("Received get_map");
            std_msgs::Empty msg_empty;
            pub_get_map.publish(msg_empty);
	    }
        if(topic == "set_pose"){
            ROS_INFO("Received set_pose topic.");
            uint8_t * payloadptr = (uint8_t *)msg->get_payload().c_str();
            uint32_t serial_size = msg->get_payload().size();
            ros::serialization::IStream stream(payloadptr, serial_size);
            geometry_msgs::Pose pose;
            ros::serialization::deserialize(stream, pose);
            std::string ss = msg->get_payload();
            geometry_msgs::PoseWithCovarianceStamped robot_pose;
            robot_pose.header.frame_id = "map";
            robot_pose.header.stamp = ros::Time::now();
            robot_pose.pose.pose = pose;
            pub_set_pose.publish(robot_pose);
	    }
        if(topic == "cancel_goal"){
            ROS_INFO("Received cancel_goal topic.");
            ros::Publisher cancle_pub_ = nh->advertise<actionlib_msgs::GoalID>("move_base/cancel", 5);
            actionlib_msgs::GoalID first_goal;
            cancle_pub_.publish(first_goal);
            ROS_INFO("Publish move_base/cancel OK.");
        }
        if(topic == "get_task_list"){
            ROS_INFO("Received get_task_list topic.");
            std_msgs::Empty msg_empty;
            pub_get_task_list.publish(msg_empty);
	    }
        ++i;//1
        if(topic == "get_task"){
            ROS_INFO("Received get_task topic.");
            if(td[i].thread_id == 0){
                td[i].thread_id = i+1;
                td[i].message = msg;
	            pthread_create(&threads[i], NULL, thread_get_task, (void*)&td[i]);
            }
	    }
        ++i;//2
        if(topic == "post_task"){
            ROS_INFO("Received post_task topic.");
            if(td[i].thread_id == 0){
                td[i].thread_id = i+1;
                td[i].message = msg;
	            pthread_create(&threads[i], NULL, thread_post_task, (void*)&td[i]);
            }
	    }
	    ++i;//3
        if(topic == "do_task"){
            ROS_INFO("Received do_task topic.");
            if(td[i].thread_id == 0){
                td[i].thread_id = i+1;
                td[i].message = msg;
	            pthread_create(&threads[i], NULL, thread_do_task, (void*)&td[i]);
            }
	    }
        if(topic == "stop_task"){
            ROS_INFO("Received stop_task topic.");
            stop_task_flag = true;
            actionlib_msgs::GoalID first_goal;
            pub_cancel_goal.publish(first_goal);
            ROS_INFO("stop task finished.");
	    }
	    ++i;//4
        if(topic == "set_nav_mode"){
            ROS_INFO("Received set_nav_mode topic.");
            if(td[i].thread_id == 0){
                td[i].thread_id = i+1;
                td[i].message = msg;
	            pthread_create(&threads[i], NULL, thread_set_nav_mode, (void*)&td[i]);
            }
	    }
	    ++i;//5
        if(topic == "xiaobai_adb"){
            ROS_INFO("Received xiaobai_adb topic.");
            if(td[i].thread_id == 0){
                td[i].thread_id = i+1;
                td[i].message = msg;
	            pthread_create(&threads[i], NULL, thread_xiaobai_adb, (void*)&td[i]);
            }
	    }
	    ++i;//6
        if(topic == "cast_ctrl_ready"){
            ROS_INFO("Received cast_ctrl_ready topic.");
            if(td[i].thread_id == 0){
                td[i].thread_id = i+1;
                td[i].message = msg;
	            pthread_create(&threads[i], NULL, thread_cast_ctrl_ready, (void*)&td[i]);
            }
	    }
        if(topic == "finish_reading"){
            ROS_INFO("Received finish_reading topic.");
        	finish_reading_flag = true;
        }
    }
    virtual void delivery_complete(mqtt::idelivery_token_ptr token) {}
public:
    callback(mqtt::async_client& cli, action_listener& listener) : cli_(cli), listener_(listener) {
        td[0].thread_id = 0;
        td[1].thread_id = 0;
        td[2].thread_id = 0;
        td[3].thread_id = 0;
        td[4].thread_id = 0;
        td[5].thread_id = 0;
        td[6].thread_id = 0;
        td[7].thread_id = 0;
        finish_reading_flag  = false;
    }
};

void toEulerianAngle(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw){
	double ysqr = q.y * q.y;

	// roll (x-axis rotation)
	double t0 = +2.0 * (q.w * q.x + q.y * q.z);
	double t1 = +1.0 - 2.0 * (q.x * q.x + ysqr);
	roll = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (q.w * q.y - q.z * q.x);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	pitch = std::asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (q.w * q.z + q.x * q.y);
	double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);  
	yaw = std::atan2(t3, t4);
}

class TopicMan{
public:
    // constructor
    TopicMan(std::string address, std::string client_id);
    ~TopicMan();
    // get odom topic
    void mapCallback(const nav_msgs::OccupancyGrid& msg){
        current_map = msg;
    } 
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
        if(!listener_->waitForTransform(
                    scan_in->header.frame_id,
                    "/map",
                    scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                    ros::Duration(1.0))){
            return;
        }
        sensor_msgs::PointCloud cloud;
        projector_->transformLaserScanToPointCloud("/map", *scan_in, cloud, *listener_);        
        // serialize and send message
        uint32_t serial_size = ros::serialization::serializationLength(cloud);
        boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
        ros::serialization::OStream stream(buffer.get(), serial_size);
        ros::serialization::Serializer<sensor_msgs::PointCloud>::write(stream, cloud);
        client->publish("cloud_obstacle", (char*)buffer.get(), serial_size, QOS, false)->wait_for_completion(TIMEOUT);
    } 
    void statusVelCallback(const geometry_msgs::Twist& msg){
        uint32_t serial_size = ros::serialization::serializationLength(msg);
        boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
        ros::serialization::OStream stream(buffer.get(), serial_size);
        ros::serialization::Serializer<geometry_msgs::Twist>::write(stream, msg);
        client->publish("status_vel", (char*)buffer.get(), serial_size, QOS, false)->wait_for_completion(TIMEOUT);
    } 
    void odomCallback(const nav_msgs::Odometry& msg){
        /********************/
        tf::Quaternion bt_orientation(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        tf::Vector3 bt_position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
        if (bt_orientation.x() == 0.0 && bt_orientation.y() == 0.0 && bt_orientation.z() == 0.0 && bt_orientation.w() == 0.0) {
          bt_orientation.setW(1.0);
        }
        tf::Stamped<tf::Pose> pose_in(tf::Transform(bt_orientation,bt_position), msg.header.stamp, "odom");
        tf::Stamped<tf::Pose> pose_out;
        // convert pose into new frame
        try {
          listener_->transformPose("map", pose_in, pose_out );
        }
        catch(std::runtime_error& e){
          ROS_DEBUG("My error transforming from frame 'odom' to frame 'map': %s", e.what());
          return;
        }
        geometry_msgs::Pose pose_current;
        pose_current.position.x = pose_out.getOrigin().x();
        pose_current.position.y = pose_out.getOrigin().y();
        pose_current.position.z = pose_out.getOrigin().z();
        pose_current.orientation.x = pose_out.getRotation().x();
        pose_current.orientation.y = pose_out.getRotation().y();
        pose_current.orientation.z = pose_out.getRotation().z();
        pose_current.orientation.w = pose_out.getRotation().w();
        current_point_ = pose_current.position;

        double roll, pitch, yaw;
        toEulerianAngle(pose_current.orientation, roll, pitch, yaw);
        pose_current.position.z = yaw;

        /********************/
        uint32_t serial_size = ros::serialization::serializationLength(pose_current);
        boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
        ros::serialization::OStream stream(buffer.get(), serial_size);
        ros::serialization::Serializer<geometry_msgs::Pose>::write(stream, pose_current);
        client->publish("odom", (char*)buffer.get(), serial_size, QOS, false)->wait_for_completion(TIMEOUT);
    }
    void motorStatusCallback(const geometry_msgs::Twist& msg){
        uint32_t serial_size = ros::serialization::serializationLength(msg);
        boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
        ros::serialization::OStream stream(buffer.get(), serial_size);
        ros::serialization::Serializer<geometry_msgs::Twist>::write(stream, msg);
        client->publish("motor_status", (char*)buffer.get(), serial_size, QOS, false)->wait_for_completion(TIMEOUT);
    } 
    void getMapCallback(const std_msgs::Empty& msg){
        uint32_t serial_size = ros::serialization::serializationLength(current_map);
        boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
        ros::serialization::OStream stream(buffer.get(), serial_size);
        ros::serialization::Serializer<nav_msgs::OccupancyGrid>::write(stream, current_map);
        ROS_INFO("Sending map message...");
        client->publish("map", (char*)buffer.get(), serial_size, QOS, false)->wait_for_completion(TIMEOUT);
        ROS_INFO("Map message sent OK.");
    }
    void arrivedCallback(const std_msgs::Empty& msg){
        ROS_INFO("Sending arrived message...");
        client->publish("arrived", "T", 1, QOS, false)->wait_for_completion(TIMEOUT);
        ROS_INFO("Arrived message sent OK.");
    }
    void readingCallback(const std_msgs::String& msg){
        ROS_INFO("Sending reading message...");
	    client->publish("reading", msg.data.c_str(), msg.data.size(), QOS, false)->wait_for_completion(TIMEOUT);
	    ROS_INFO("Reading message sent OK.");
    }
    void getTaskListCallback(const std_msgs::Empty& msg){
        ROS_INFO("Sending task list message...");
        std::vector<std::string> file_names;
        scanFileNames("/home/odroid/trxb/tasks/", file_names);

	    std::string task_list_content = "";
        for(unsigned int i = 0; i < file_names.size(); ++i){
            task_list_content = task_list_content + file_names[i];
            if(file_names.size()-1!=i){
                task_list_content = task_list_content + ",";
            }            
        }
        ROS_INFO("list:%s", task_list_content.c_str());
        std_msgs::String send_str;
        send_str.data = task_list_content;

        uint32_t serial_size = ros::serialization::serializationLength(send_str);
        boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
        ros::serialization::OStream stream(buffer.get(), serial_size);
        ros::serialization::Serializer<std_msgs::String>::write(stream, send_str);

        client->publish("task_list", (char*) buffer.get(), serial_size, QOS, false)->wait_for_completion(TIMEOUT);
        ROS_INFO("Task list message sent OK.");
    }
    void getTaskCallback(const std_msgs::String& msg){
        //
	std::string file_name = "/home/odroid/trxb/tasks/" + msg.data;
        ROS_INFO("Reading file: %s.", file_name.c_str());
	    if(!is_file_exist(file_name.c_str())){
            ROS_INFO("File does not exist: %s.", file_name.c_str());
	        return;
        }
        std::ifstream fin(file_name, std::ios::binary);
        std::istream::pos_type current_pos = fin.tellg();
        fin.seekg(0, std::ios_base::end);
        std::istream::pos_type file_size = fin.tellg();
        fin.seekg(current_pos);
        char *outbuffer = new char[file_size];
        fin.read((char*)outbuffer, file_size);
        fin.close();
        //
        ROS_INFO("Sending task content message...");
        client->publish("task_content", outbuffer, file_size, QOS, false)->wait_for_completion(TIMEOUT);
        delete []outbuffer;
        ROS_INFO("Task content message sent OK. size: %d", (int)file_size);
    }
private:
    mqtt::async_client *client;
    mqtt::itoken_ptr conntok;
    action_listener *subListener;
    callback *cb;
    nav_msgs::OccupancyGrid current_map;
};

TopicMan::TopicMan(std::string address, std::string client_id){
    nh = new ros::NodeHandle();  
    sub_map = nh->subscribe("map", 5, &TopicMan::mapCallback, this);
    sub_scan = nh->subscribe("scan", 5, &TopicMan::scanCallback, this);
    sub_status_vel = nh->subscribe("status_vel", 5, &TopicMan::statusVelCallback, this);
    sub_odom = nh->subscribe("odom", 5, &TopicMan::odomCallback, this);
    sub_motor_status = nh->subscribe("motor_status", 5, &TopicMan::motorStatusCallback, this);
    pub_control_vel = nh->advertise<geometry_msgs::Twist>("cmd_vel", 5);
    pub_nav_goal = nh->advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 5);    
    pub_set_pose = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 5);    
    pub_artificial_path = nh->advertise<nav_msgs::Path>("move_base/PatrolPlanner/artificial_path", 5);

    pub_get_map = nh->advertise<std_msgs::Empty>("get_map", 5);
    sub_get_map = nh->subscribe("get_map", 5, &TopicMan::getMapCallback, this);
    pub_arrived = nh->advertise<std_msgs::Empty>("arrived", 5);
    sub_arrived = nh->subscribe("arrived", 5, &TopicMan::arrivedCallback, this);
    pub_reading = nh->advertise<std_msgs::String>("reading", 5);
    sub_reading = nh->subscribe("reading", 5, &TopicMan::readingCallback, this);

    pub_get_task_list = nh->advertise<std_msgs::Empty>("get_task_list", 5);
    sub_get_task_list = nh->subscribe("get_task_list", 5, &TopicMan::getTaskListCallback, this);
    pub_get_task = nh->advertise<std_msgs::String>("get_task", 5);
    sub_get_task = nh->subscribe("get_task", 5, &TopicMan::getTaskCallback, this);

    pub_cancel_goal =  nh->advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    
    set_goal_client = new SetGoalClient("move_base");

    client = new mqtt::async_client(address.c_str(), client_id.c_str());
    subListener = new action_listener("Subscription");
    cb = new callback(*client, *subListener);
    client->set_callback(*cb);

    projector_ = new laser_geometry::LaserProjection();
    listener_ = new tf::TransformListener();
    
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);
    conntok = client->connect(connOpts);
    ROS_INFO("Waiting for the connection...");
    conntok->wait_for_completion();
    ROS_INFO("Connected.");
    client->subscribe("control_vel", QOS, nullptr, *subListener);
    client->subscribe("nav_goal", QOS, nullptr, *subListener);
    client->subscribe("get_map", QOS, nullptr, *subListener);
    client->subscribe("set_pose", QOS, nullptr, *subListener);
    client->subscribe("cancel_goal", QOS, nullptr, *subListener);
    client->subscribe("finish_reading", QOS, nullptr, *subListener);

    client->subscribe("get_task_list", QOS, nullptr, *subListener);
    client->subscribe("get_task", QOS, nullptr, *subListener);
    client->subscribe("post_task", QOS, nullptr, *subListener);
    client->subscribe("do_task", QOS, nullptr, *subListener);
    client->subscribe("stop_task", QOS, nullptr, *subListener);

    client->subscribe("set_nav_mode", QOS, nullptr, *subListener);

    client->subscribe("xiaobai_adb", QOS, nullptr, *subListener);
    client->subscribe("cast_ctrl_ready", QOS, nullptr, *subListener);
}

TopicMan::~TopicMan(){
    ROS_INFO("Disconnecting...");
    conntok = client->disconnect();
    conntok->wait_for_completion();
    ROS_INFO("Disconnected OK.");
}

int main(int argc, char **argv){
    //if(argc != 2){
    //    printf("argument is not enough. example: mqtt_client pub tcp://112.230.206.158:1883\n");
    //    return 1;
    //} 
    ros::init(argc, argv, "mqtt_client_pub");
    TopicMan topic_man("tcp://localhost:1883", "robot_base_xb001");
    //TopicMan topic_man("tcp://112.230.206.158:1883", "robot_base_xb001");
    //TopicMan topic_man("tcp://192.168.100.103:1883", "robot_base_xb001");
    //TopicMan topic_man("tcp://192.168.2.92:1883", "robot_base_xb001");
    ros::spin();
    return 0;
}
