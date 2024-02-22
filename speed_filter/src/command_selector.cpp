#include "ros/ros.h"
#include "brain_msgs/VehicleState.h"
#include "brain_msgs/LongitudinalCmd.h"
#include "brain_msgs/Object.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"
#include "speed_filter/Locations.h"
#include "std_msgs/Float32.h"

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include <cmath>
using namespace std;

void* start_sp; // 프로그램 시작 시 스택 포인터 주소
void* max_sp; // 관찰된 최대 스택 포인터 주소
long max_stack_usage; // 최대 스택 사용량


static void* getStackPointer() { // Stackpointer c
    void* sp;
    asm ("mov %%rsp, %0" : "=r"(sp)); // x86_64 아키텍처용
    return sp;
}

void updateStackUsage() {
    void* current_sp = getStackPointer();
    if (static_cast<char*>(start_sp) - static_cast<char*>(current_sp) > max_stack_usage) {
        max_sp = current_sp;
        max_stack_usage = static_cast<char*>(start_sp) - static_cast<char*>(current_sp);
    }
}

struct Node {
    float x = 0;
    float y = 0;
    float v = 0;
    float a = 0;
    Node* parent = nullptr;
};

class PathFinder {
public:
    vector<Node*> nodeList;
    vector<pair<float, float>> objectList;
    Node* ptNode = nullptr;
    bool pathfound;

    PathFinder() : pathfound(false) {}

    void Sampling(Node* sample,Node*& parent, float accel);
    bool collision(Node* sample, Node*& parent, float distance_object_to_line);
    float Filter(float cmd_acceleration, float current_acceleration, float velocity, float velocity_limit, float distance_object_to_line);
    float findPath(float start_x, float start_y, float start_v, float start_a, float distance_object_to_line);
    void AddLinearFunctionPoints(float a, float b, float m, float verticalHeight, float xEnd, float gap1, float gap2);
    
    ~PathFinder() {
        for (Node* node : nodeList) delete node;
        nodeList.clear();
        objectList.clear();
    }
};

void PathFinder::Sampling(Node* sample, Node*& parent, float accel) {
    float delta_t = (parent->v/5) * (-0.1 * accel + 0.5);
    if (delta_t < 0.4) delta_t = 0.4;
    sample->x = parent->x + delta_t;
    sample->y = parent->y + 0.5*(accel)*(delta_t)*(delta_t) + (parent->v)*delta_t;
    sample->v = parent->v + accel*delta_t;
    sample->a = accel;
    sample->parent = parent;
    //updateStackUsage();
}

//check collision graph vs obstacle at T-S diagram
bool PathFinder::collision(Node* sample, Node*& parent, float distance_object_to_line) {
    float tmpDist;
    float tmpY;

    for (auto o : objectList) {
        if (o.first >= parent->x && o.first <= sample->x && o.second > parent->y) {
            tmpY = 0.5*(sample->a)*(o.first-parent->x)*(o.first-parent->x) + (parent->v)*(o.first-parent->x) + parent->y;
            tmpDist = abs(o.second - tmpY);
            if (tmpDist <= distance_object_to_line) {  
                delete sample;
                return false;
            }
        }
        //updateStackUsage();
    }
    return true;
}


float PathFinder::Filter(float cmd_acceleration, float current_acceleration, float velocity, float velocity_limit, float distance_object_to_line) {
    float x_new = 0.8;
    float y_new = (0.5*cmd_acceleration)*x_new*x_new + velocity*x_new;
    float v_new = velocity + cmd_acceleration*x_new;
    float Target_A;
    float tmpY;
    float tmpDist;
    bool new_check = true;
    for (auto o : objectList) {
        if (o.first <= x_new) {
            tmpY = 0.5*(cmd_acceleration)*(o.first)*(o.first) + (velocity)*(o.first);
            tmpDist = abs(o.second - tmpY);
            if (tmpDist <= distance_object_to_line) {
                new_check = false;
                break;
            }
        }
    }

    //  
    if (!new_check){
        // ROS_INFO("Collision Warning");
        Target_A = min(findPath(0, 0, velocity, min(current_acceleration, 0.0f), distance_object_to_line), cmd_acceleration);
        // updateStackUsage();
        return Target_A;
    }

    // velocity limit
    else if (velocity > velocity_limit) {
        // ROS_INFO("Velocity Limit");
        Target_A = findPath(x_new, y_new, v_new, - 0.5f, distance_object_to_line);
        if (Target_A != -0.5) {
            if (Target_A < -5) {
                // Start at (0, 0)
                Target_A = findPath(0, 0, velocity, min(current_acceleration - 0.5f, -0.5f), distance_object_to_line);
            }
            else {
                // 
                Target_A = min(min((Target_A+2*cmd_acceleration)/3, current_acceleration - 0.5f), 0.0f);
            }
            return Target_A;
        }
        //updateStackUsage();
        return Target_A;
    }

    else {
        Target_A = findPath(x_new, y_new, v_new, cmd_acceleration, distance_object_to_line);
        if (Target_A != cmd_acceleration) {
            if (Target_A < -5) {
                // Start at (0, 0) consider jerk
                Target_A = findPath(0, 0, velocity, min(current_acceleration - 0.5f, -0.5f), distance_object_to_line);
            }
            else if (Target_A >= 0) {
                Target_A = 0;
            }
            else {
                Target_A = min(min(min((Target_A+2*cmd_acceleration)/3, cmd_acceleration - 0.5f), current_acceleration - 0.5f), 0.0f);
            }
            //updateStackUsage();
            return Target_A;
        }
    }
    //updateStackUsage();
    Target_A = cmd_acceleration;
    return Target_A;
}

float PathFinder::findPath(float start_x, float start_y, float start_v, float start_a, float distance_object_to_line) {
    Node* head = new Node;
    head->x = start_x;
    head->y = start_y;
    head->v = start_v;
    nodeList.push_back(head);

    float width = start_v/5 + 3;
    float delta_t = 0.4;
    float iterator = start_a;
    bool result = false;
    //updateStackUsage();
    while (pathfound == false)
    {   
        Node* parent = nodeList.back();
    
        for (float i = iterator ; i >= -5 ; i--) {
            ptNode = new Node;
            Sampling(ptNode, parent , i);
            result = collision(ptNode, parent, distance_object_to_line);

            if (result) {
                ptNode->x = ptNode->parent->x + delta_t;
                ptNode->y = 0.5*(i)*delta_t*delta_t + (ptNode->parent->v)*delta_t + ptNode->parent->y;
                ptNode->v = ptNode->parent->v + i*delta_t;
                ptNode->a = i;
                nodeList.push_back(ptNode);
                iterator = 0;

                if (nodeList.back()->x >= width || nodeList.back()->v <= 0) pathfound = true;
                break;
            }

            else if (i < -4) {
                for (int j = nodeList.size() - 1; j >= 0; j--) {
                    if (nodeList.back()->a < -4.5) {
                        delete nodeList.back();
                        nodeList.pop_back();
                    }
                    else {
                        iterator = (nodeList.back()->a) - 1;
                        delete nodeList.back();
                        nodeList.pop_back();
                        break;
                    }
                }
            }
        }
        // 
        if (nodeList.empty()) {
            return -5.001;
        }
    }
    //updateStackUsage();
    return nodeList.at(1)->a;
}

// Add collision into T-S
void PathFinder::AddLinearFunctionPoints(float a, float b, float m, float verticalHeight, float xEnd, float gap1, float gap2) {
    if (gap1 > 0.4f) gap1 = 0.4f;
    if (a < 0) a = 0;
    for (float i = b; i < b + verticalHeight; i += gap2) {
        for (float x = a; x < xEnd + gap1; x += gap1) {
            float y = m * (x-a) + i;
            objectList.push_back(make_pair(x, y));
            //updateStackUsage();
        }
    }
}

void saveData(const vector<Node*>& nodeList, const vector<pair<float, float>>& objectList, float input_speed) {
    ofstream fobject("object.txt");
    ofstream fpath("path.txt");
    ofstream faccel("cmd_acceleration.txt");

    for (auto a : objectList)   fobject << a.first << "," << a.second << endl;
    fobject.close();

    fpath << "0" << "," << "0" << endl;

    if (nodeList.size() == 0) {
        float x = 0.3;
        float y_p = 0;
        while (x <= 8) {
            float y = -2.5 * x * x + input_speed * x;
            if (y - y_p < 0) y = y_p;
            fpath << x << "," << y << endl;
            x += 0.3;
            y_p = y;
        }
        
    }

    for (Node* node : nodeList) {
        fpath << node->x << "," << node->y << endl;
    }
    fpath.close();

    for (Node* node : nodeList) faccel << node->x << "," << node->a << endl;
    faccel.close();
}

class SubAndPub {
public:
    ros::NodeHandle nh;
    ros::Subscriber sub_vehicle_state;
    ros::Subscriber sub_longitudinal_cmd;
    ros::Subscriber sub_collison;
    ros::Publisher pub_longitudinal_cmd;

    float distance_object_to_line, velocity_limit;
    string vehicle_state_topic, command_longitudinal_input_topic, collision_info_topic, command_longitudinal_output_topic;

    SubAndPub() : nh("/") {
        start_sp = getStackPointer(); // 생성자에서 시작 스택 포인터 주소 저장
        max_sp = start_sp;
        max_stack_usage = 0;
        nh.getParam("/config/command_selector/parameters/distance_object_to_line", distance_object_to_line);
        nh.getParam("/config/command_selector/parameters/velocity_limit", velocity_limit);
        
        nh.getParam("/config/command_selector/topics/vehicle_state", vehicle_state_topic);
        nh.getParam("/config/command_selector/topics/command_longitudinal_input", command_longitudinal_input_topic);
        nh.getParam("/config/command_selector/topics/collision_info", collision_info_topic);
        nh.getParam("/config/command_selector/topics/command_longitudinal_output", command_longitudinal_output_topic);
        
        sub_vehicle_state = nh.subscribe(vehicle_state_topic, 1, &SubAndPub::Callback1, this);;
        sub_longitudinal_cmd = nh.subscribe(command_longitudinal_input_topic, 10, &SubAndPub::Callback2, this);
        sub_collison = nh.subscribe(collision_info_topic, 1, &SubAndPub::Callback3, this);
        pub_longitudinal_cmd = nh.advertise<brain_msgs::LongitudinalCmd>(command_longitudinal_output_topic, 10);
    }

    void Callback1(const brain_msgs::VehicleState::ConstPtr& msg) {
        input_speed = msg->speed;
        input_longitudinal_acceleration = msg->longitudinal_acceleration;
        input_steering_angle = msg->steering_angle;
    }
    
    void Callback2(const brain_msgs::LongitudinalCmd::ConstPtr& msg) {
        input_cmd_acceleration = msg->acceleration;
        LongitudinalCmd = *msg;
    }
     
	void Callback3(const speed_filter::Locations::ConstPtr& msg) {
    	PathFinder pathFinder;

        for (int i = 0; i < 6 ; i++) {
        if (msg->front_object_station > 0) pathFinder.AddLinearFunctionPoints(0, msg->front_object_station, msg->front_object_velocity, 1, input_speed/5 + 3, 2*distance_object_to_line/input_speed, 2*distance_object_to_line);
        }
        
        for (int i = 0 ; i < msg->objectList.size() ; ++i) {
            pathFinder.AddLinearFunctionPoints(msg->objectList[i].x, msg->objectList[i].y, msg->objectList[i].m, 2*msg->objectList[i].r,  msg->objectList[i].x + 2, 2*distance_object_to_line/input_speed , 2*distance_object_to_line);
        }

        LongitudinalCmd.acceleration = pathFinder.Filter(input_cmd_acceleration, input_longitudinal_acceleration, input_speed, velocity_limit, distance_object_to_line);
    	pub_longitudinal_cmd.publish(LongitudinalCmd);

        //HW spec check
        // updateStackUsage();
        // max_stack_usage = static_cast<char*>(start_sp) - static_cast<char*>(max_sp);
        // std::cout << "Maximum stack usage: " << max_stack_usage << " bytes" << std::endl;

        // size_t nodeMemoryUsage = pathFinder.nodeList.size() * sizeof(Node);
        // size_t objectListMemoryUsage = pathFinder.objectList.capacity() * sizeof(std::pair<float, float>);

        // std::cout << "Node list memory usage: " << nodeMemoryUsage << " bytes" << std::endl;
        // std::cout << "Object list memory usage: " << objectListMemoryUsage << " bytes" << std::endl;

        // saveData(pathFinder.nodeList, pathFinder.objectList, input_speed);
        if (input_cmd_acceleration != LongitudinalCmd.acceleration) {
            ROS_INFO("INPUT CMD : %10f     OUTPUT CMD :%10f", input_cmd_acceleration, LongitudinalCmd.acceleration);
        }
        else ROS_INFO("INPUT CMD : %10f", input_cmd_acceleration);
        cout << " " << endl;
    }
    
private:
    brain_msgs::LongitudinalCmd LongitudinalCmd;
    float input_speed;
    float input_longitudinal_acceleration;
    float input_steering_angle;
    float input_cmd_acceleration;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "command_selector");
    SubAndPub SnP;
    ros::spin();
    return 0;
}