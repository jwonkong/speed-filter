#include "ros/ros.h"
#include "brain_msgs/VehicleState.h"
#include "brain_msgs/ObjectArray.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "speed_filter/Locations.h"
#include "std_msgs/Time.h"
#include <visualization_msgs/MarkerArray.h>

#include <ctime>
#include <vector>
#include <cmath>
#include <fstream>
#include <stdlib.h>

using namespace std;

const float PI = 3.141592653589793;

void* getStackPointer() {
    void* sp;
    asm ("mov %%rsp, %0" : "=r"(sp)); // x86_64 아키텍처용
    return sp;
}

void saveData(const geometry_msgs::Polygon& polygon, const geometry_msgs::Polygon& object_point) {
    ofstream outFile("vehicle_trajectory.csv");
    ofstream objFile("object_point.csv");

    if (!outFile.is_open()) {
        cerr << "Error opening vehicle_trajectory.csv" << endl;
        return;
    }

    if (!objFile.is_open()) {
        cerr << "Error opening object_point.csv" << endl;
        return;
    }

    outFile << "x,y\n";
    objFile << "x,y\n";

    for (const auto& point : polygon.points) outFile << point.x << "," << point.y << "\n";
    outFile.close();

    for (const auto& point : object_point.points) objFile << point.x << "," << point.y << "\n";
    objFile.close();
}

class SubAndPub {
public:
    ros::NodeHandle nh;
    ros::Subscriber sub_vehicle_state;
    ros::Subscriber sub_objects;
    ros::Publisher pub_collision;
    ros::Publisher marker_pub;

    void* start_sp; // 프로그램 시작 시 스택 포인터 주소
    void* max_sp; // 관찰된 최대 스택 포인터 주소
    long max_stack_usage; // 최대 스택 사용량

    float L, Lf, dt, Cf, Cr, Iz, m, W, S;
    string vehicle_state_topic, objects_topic, collision_info_topic;

    SubAndPub() : nh("/") {
        start_sp = getStackPointer(); // 생성자에서 시작 스택 포인터 주소 저장
        max_sp = start_sp;
        max_stack_usage = 0;

        nh.getParam("/config/object_processor/vehicle_parameters/wheel_base", L);
        nh.getParam("/config/object_processor/vehicle_parameters/track_width", W);
        nh.getParam("/config/object_processor/vehicle_parameters/distance_from_cg_to_front_axle", Lf);
        nh.getParam("/config/object_processor/vehicle_parameters/time_step", dt);
        nh.getParam("/config/object_processor/vehicle_parameters/cornering_stiffness_front", Cf);
        nh.getParam("/config/object_processor/vehicle_parameters/cornering_stiffness_rear", Cr);
        nh.getParam("/config/object_processor/vehicle_parameters/yaw_inertia", Iz);
        nh.getParam("/config/object_processor/vehicle_parameters/vehicle_mass", m);
        nh.getParam("/config/object_processor/vehicle_parameters/steering_ratio", S);

        nh.getParam("/config/object_processor/topics/vehicle_state", vehicle_state_topic);
        nh.getParam("/config/object_processor/topics/objects", objects_topic);
        nh.getParam("/config/object_processor/topics/collision_info", collision_info_topic);
                
        sub_vehicle_state = nh.subscribe(vehicle_state_topic, 1, &SubAndPub::vehicleStateCallback, this);
        sub_objects = nh.subscribe(objects_topic, 1, &SubAndPub::objectArrayCallback, this);
        pub_collision = nh.advertise<speed_filter::Locations>(collision_info_topic, 1);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    }

    void publishMarkers(const brain_msgs::ObjectArray::ConstPtr& msg, const geometry_msgs::Polygon& predicted_my_path) {
        visualization_msgs::MarkerArray marker_array;
        int id = 0;

        // Ego path marker
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "path";
        line_strip.id = id++;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        for (const auto& point : predicted_my_path.points) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            line_strip.points.push_back(p);
        }
        line_strip.scale.x = 0.1;
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;
        line_strip.lifetime = ros::Duration();
        marker_array.markers.push_back(line_strip);

        // Origin marker
        visualization_msgs::Marker origin_marker;
        origin_marker.header.frame_id = "map";
        origin_marker.header.stamp = ros::Time::now();
        origin_marker.ns = "origin_object";
        origin_marker.id = 999; // Unique ID to avoid collision with existing markers
        origin_marker.type = visualization_msgs::Marker::CUBE;
        origin_marker.action = visualization_msgs::Marker::ADD;
        origin_marker.pose.position.x = 0.5;
        origin_marker.pose.position.y = 0.0;
        origin_marker.pose.position.z = 0.4;
        origin_marker.scale.x = 4.5;
        origin_marker.scale.y = 1.9;
        origin_marker.scale.z = 1.2;
        origin_marker.color.r = 0.0;
        origin_marker.color.g = 0.0;
        origin_marker.color.b = 1.0;
        origin_marker.color.a = 0.8;
        origin_marker.lifetime = ros::Duration();
        marker_array.markers.push_back(origin_marker);

        // Object markers
        for (const auto& object : msg->objects) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "objects";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = object.odometry.pose.pose.position.x;
            marker.pose.position.y = object.odometry.pose.pose.position.y;
            marker.pose.position.z = object.odometry.pose.pose.position.z;
            marker.pose.orientation = object.odometry.pose.pose.orientation;
            marker.scale.x = object.size.size[0];
            marker.scale.y = object.size.size[1];
            marker.scale.z = object.size.size[2];
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
            marker.lifetime = ros::Duration();
            marker_array.markers.push_back(marker);
        }

        // Publish the markers
        marker_pub.publish(marker_array);
    }



    void updateStackUsage() {
        void* current_sp = getStackPointer();
        if (static_cast<char*>(start_sp) - static_cast<char*>(current_sp) > max_stack_usage) {
            max_sp = current_sp;
            max_stack_usage = static_cast<char*>(start_sp) - static_cast<char*>(current_sp);
        }
    }

    class Vehicle {
    public:
        float x;  // X position [m]
        float y;  // Y position [m]
        float yaw;  // Yaw angle [rad]
        float vx;  // Longitudinal velocity [m/s]
        float vy;  // Lateral velocity [m/s]
        float omega;  // Yaw rate [rad/s]    

        Vehicle() : x(0.0), y(0.0), yaw(0.0), vx(0.0), vy(0.0), omega(0.0) {}

        float normalize_angle(float angle) {
        while (angle > PI) angle -= 2.0 * PI;
        while (angle < -PI) angle += 2.0 * PI;
        return angle;
        }

        void update_nonlinear(float vx, float delta, float L, float Lf, float dt, float m, float Cf, float Cr, float Iz) {
            float Lr = L - Lf;  // Distance from CG to rear axle [m]
            float beta = atan2(Lr / L * tan(delta), 1.0);
            float cos_yaw = cos(yaw);
            float sin_yaw = sin(yaw);

            // Update states
            x += vx * cos_yaw * dt - vy * sin_yaw * dt;
            y += vx * sin_yaw * dt + vy * cos_yaw * dt;
            yaw += omega * dt;
            yaw = normalize_angle(yaw);

            float Ffy = -Cf * atan2((vy + Lf * omega) / vx - delta, 1.0);
            float Fry = -Cr * atan2((vy - Lr * omega) / vx, 1.0);

            // Update velocities
            vx = vx + dt * (Ffy * sin(delta) / m);
            vy = vy + dt * ((Fry + Ffy * cos(delta)) / m - vx * omega);
            omega += dt * (Ffy * Lf * cos(delta) - Fry * Lr) / Iz;
        }

        void update_linear(float vx, float delta, float L, float Lf, float dt, float m, float Cf, float Cr, float Iz) {
            float Lr = L - Lf;
            float beta = atan2((Lr / L) * tan(delta), 1.0);

            // Update state
            x += vx * cos(yaw+beta) * dt;
            y += vx * sin(yaw+beta) * dt;
            yaw += vx / L * cos(beta) * tan(delta) * dt;
            yaw = normalize_angle(yaw);
            
        }
    };

    void vehicleStateCallback(const brain_msgs::VehicleState::ConstPtr& msg) {
        input_speed = msg->speed;
        input_steering_angle = msg->steering_angle;
    }


    void objectArrayCallback(const brain_msgs::ObjectArray::ConstPtr& msg) {
        speed_filter::Locations loc;
        loc.stamp = ros::Time::now();
        Vehicle vehicle;
        // My path
        N = static_cast<int>((input_speed / 5 + 3) / dt);
        if (input_speed < 6) N = static_cast<int>(8 / dt);

        predicted_my_path.points.clear();
        object_point.points.clear();
        geometry_msgs::Point32 point;
        point.x = 0.0;
        point.y = 0.0;
        predicted_my_path.points.push_back(point);

        if (input_speed < 6) {
            for (int i = 0; i < N; ++i) {
                geometry_msgs::Point32 point;
                vehicle.update_linear(input_speed, input_steering_angle*(PI/180)/S, L, Lf, dt, m, Cf, Cr, Iz);
                point.x = vehicle.x;
                point.y = vehicle.y;
                if (point.x < predicted_my_path.points.back().x) break;
                predicted_my_path.points.push_back(point);
                //updateStackUsage();
            }
        }

        else {
            for (int i = 0; i < N; ++i) {
                geometry_msgs::Point32 point;
                vehicle.update_nonlinear(input_speed, input_steering_angle*(PI/180)/S, L, Lf, dt, m, Cf, Cr, Iz);
                point.x = vehicle.x;
                point.y = vehicle.y;
                if (point.x < predicted_my_path.points.back().x) break;
                predicted_my_path.points.push_back(point);
                //updateStackUsage();
            }
        }

        // To show my path in Rviz
        publishMarkers(msg, predicted_my_path);

        for (const auto& object : msg->objects) {
            // 앞에 있는 객체
            if (object.odometry.pose.pose.position.x > 0) {
                geometry_msgs::Point32 point;
                point.x = object.odometry.pose.pose.position.x;
                point.y = object.odometry.pose.pose.position.y; 
                object_point.points.push_back(point);

                // 절대속도
                float object_vx = object.odometry.twist.twist.linear.x;
                float object_vy = object.odometry.twist.twist.linear.y;

                // 객체의 3초간의 움직임을 등속 모델로 추정하여 직사각형으로 표현
                geometry_msgs::Point32 center_point;
                center_point.x = object.odometry.pose.pose.position.x + (3/2)*object_vx;
                center_point.y = object.odometry.pose.pose.position.y + (3/2)*object_vy;
                float theta = atan2(object_vy, object_vx);
                float v = sqrt(object_vx*object_vx + object_vy*object_vy);
                float rectHalfHeight = (object.size.size[0] / 2) + (3/2)*v;
                float rectHalfWidth = object.size.size[1] / 2;

                geometry_msgs::Point32 rotatedCenter;
                for (size_t i = 1; i < predicted_my_path.points.size(); ++i) {
                    //my_path 회전변환
                    rotatedCenter.x = cos(theta) * (predicted_my_path.points[i].x - center_point.x) + sin(theta) * (predicted_my_path.points[i].y - center_point.y);
                    rotatedCenter.y = -sin(theta) * (predicted_my_path.points[i].x - center_point.x) + cos(theta) * (predicted_my_path.points[i].y - center_point.y);
                    float distX = fabs(rotatedCenter.x);
                    float distY = fabs(rotatedCenter.y);
                    //충돌 감지
                    if (distY <= (rectHalfWidth + W/2) && distX <= (rectHalfHeight + W/2)) {
                        // 내 경로 방향으로의 객체 속도 내적
                        float dx = predicted_my_path.points[i].x - predicted_my_path.points[i-1].x;
                        float dy = predicted_my_path.points[i].y - predicted_my_path.points[i-1].y;
                        float length = sqrt(dx*dx + dy*dy);
                        float ux = dx / length;
                        float uy = dy / length;
                        // 내 경로상에 현재 장애물이 있을 때
                        if (-rectHalfHeight + object.size.size[0] + W/2 >= rotatedCenter.x) {
                            float s = sqrt(pow(predicted_my_path.points[i].x, 2) + pow(predicted_my_path.points[i].y, 2));
                            if (loc.front_object_station != 0 && s > loc.front_object_station) break;
                            loc.front_object_station = s;
                            loc.front_object_velocity = object_vx * ux + object_vy * uy;
                            //updateStackUsage();
                            break;
                        }
                        // 내 경로상에 미래에 장애물이 있을 때 
                        else {
                            speed_filter::Coordinate coord;
                            float t = ((3/2)*v + rotatedCenter.x - object.size.size[0] - W/2) / (3*v);
                            coord.x = 3*t;
                            coord.y = sqrt(pow(predicted_my_path.points[i].x, 2) + pow(predicted_my_path.points[i].y, 2));
                            coord.m = object_vx * ux + object_vy * uy;
                            coord.r = sqrt(object.size.size[0]*object.size.size[0] + object.size.size[1]*object.size.size[1])/2;
                            loc.objectList.push_back(coord);
                            //updateStackUsage();
                            break;
                        }
                    }
                }
            }
        }
        
        pub_collision.publish(loc);
        ///HW spec check
        //updateStackUsage();
        // max_stack_usage = static_cast<char*>(start_sp) - static_cast<char*>(max_sp);
        // std::cout << "Maximum stack usage: " << max_stack_usage << " bytes" << std::endl;

        // size_t memoryUsage = predicted_my_path.points.capacity() * sizeof(geometry_msgs::Point32);
        // std::cout << "Memory usage of predicted_my_path: " << memoryUsage << " bytes" << std::endl;

        // size_t memoryUsag = object_point.points.capacity() * sizeof(geometry_msgs::Point32);
        // std::cout << "Memory usage of object_point: " << memoryUsag << " bytes" << std::endl;

        // saveData(predicted_my_path, object_point);
        // if (loc.front_object_station > 0) {
        //     ROS_INFO("y = %ft + %f", loc.front_object_velocity, loc.front_object_station);
        // }

        // for (int i = 0 ; i < loc.objectList.size() ; ++i) {
        //     ROS_INFO("x, y, m, r : %f, %f, %f, %f", loc.objectList[i].x, loc.objectList[i].y, loc.objectList[i].m, loc.objectList[i].r);
        // }
        // cout << " " << endl;
    }

private:
    float input_speed;
    float input_steering_angle;
    int N;

    geometry_msgs::Polygon object_point;
    geometry_msgs::Polygon predicted_my_path;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_processor");
    SubAndPub SnP;
    ros::spin();
    return 0;
}