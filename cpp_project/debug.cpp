#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <random>
#include <unistd.h>
#include <thread>
using namespace std;

const float distance_object_to_line = 4;
const float velocity_limit = 22;

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
    if (delta_t < 0.5) delta_t = 0.5;
    sample->x = parent->x + delta_t;
    sample->y = parent->y + 0.5*(accel)*(delta_t)*(delta_t) + (parent->v)*delta_t;
    sample->v = parent->v + accel*delta_t;
    sample->a = accel;
    sample->parent = parent;
}

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
    }
    return true;
}

float PathFinder::Filter(float cmd_acceleration, float current_acceleration, float velocity, float velocity_limit, float distance_object_to_line) {
    float x_new = 0.4;
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
        // else if (o.first <= x_new + velocity/5){
        //     tmpY = (velocity + cmd_acceleration*x_new)*(o.first - x_new) + 0.5*(cmd_acceleration)*x_new*x_new + (velocity)*(x_new);
        //     tmpDist = abs(o.second - tmpY);
        //     if (tmpDist <= distance_object_to_line) {
        //         new_check = false;
        //         break;
        //     }
        // }
    }

    if (!new_check){
        // cout << "Collision warning" << endl;
        Target_A = min(findPath(0, 0, velocity, min(current_acceleration, 0.0f), distance_object_to_line), cmd_acceleration);
        return Target_A;
    }

    else if (velocity > velocity_limit) {
        // cout << "velocity limit" << endl;
        Target_A = findPath(0, 0, velocity, -0.5, distance_object_to_line);
        return Target_A;
    }

    else {
        Target_A = findPath(x_new, y_new, v_new, cmd_acceleration, distance_object_to_line);
        if (Target_A != cmd_acceleration) {
            // cout << "Cmd_A is dangerous" << endl;
            if (Target_A < -5) {
                // cout << "Target_A < -5" << endl;
                Target_A = findPath(0, 0, velocity, min(current_acceleration - 0.5f, -0.5f), distance_object_to_line);
            }
            else if (Target_A >= 0) {
                // cout << "Target_A >= 0;" << endl;
                Target_A = 0;
            }
            else {
                // cout << "else" << endl;
                Target_A = min(min(min((Target_A+2*cmd_acceleration)/3, cmd_acceleration - 0.5f), current_acceleration - 0.5f), - 0.0f);
            }
            return Target_A;
        }
    }

    Target_A = cmd_acceleration;
    // if (cmd_acceleration == Target_A) cout << "Target A = Input command" << endl;
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
                        iterator = (nodeList.back()->a) - 0.5;
                        delete nodeList.back();
                        nodeList.pop_back();
                        break;
                    }
                }
            }
        }
        if (nodeList.empty()) {
            return -5.01;
        }
    }
    return nodeList.at(1)->a;
}


void PathFinder::AddLinearFunctionPoints(float a, float b, float m, float verticalHeight, float xEnd, float gap1, float gap2) {
    if (gap1 > 0.2) gap1 = 0.2;
    float y;
    for (float i = b; i < b + verticalHeight; i += gap2) {
        for (float x = a; x < xEnd + gap1; x += gap1) {
            y = m * (x-a) + b;
            objectList.push_back(make_pair(x, y));
        }
    }
}

void saveData(const vector<Node*>& nodeList, const vector<pair<float, float>>& objectList) {
    ofstream fobject("object.txt");
    ofstream fpath("path.txt");
    ofstream faccel("cmd_acceleration.txt");

    for (auto a : objectList)   fobject << a.first << "," << a.second << endl;
    fobject.close();
    
    fpath << "0" << "," << "0" << endl;
    for (Node* node : nodeList) fpath << node->x << "," << node->y << endl;
    fpath.close();

    for (Node* node : nodeList) faccel << node->x << "," << node->a << endl;
    faccel.close();
}


int main() {
    const int iterations_per_second = 50;
    const std::chrono::milliseconds loop_duration(1000 / iterations_per_second);
    auto last_iteration_time = std::chrono::steady_clock::now();

    for (int i = 0 ; i < 1000000 ; i++) {
        float stackVariable; // 시작할 때의 변수
    std::cout << "Stack Variable Address at Start: " << &stackVariable << std::endl;

        auto start = std::chrono::high_resolution_clock::now();
        // Front Object Info
        float front_obj_v = 11;
        float front_obj_s = 7.0919;
        // Current State
        float VehicleState_speed = 15;
        float VehicleState_Acceleration = 0; // pid
        // Acceleration Input Command
        float LongitudinalCommand = 0;
        // Object
        PathFinder pathFinder;
        
        pathFinder.AddLinearFunctionPoints(0, front_obj_s, front_obj_v, 6, VehicleState_speed/5 + 3, 2*distance_object_to_line/VehicleState_speed, 2*distance_object_to_line);

        float Target_A = pathFinder.Filter(LongitudinalCommand, VehicleState_Acceleration, VehicleState_speed, velocity_limit, distance_object_to_line);


        // std::cout << "My Cmd: " << Target_A << std::endl;

        // auto finish = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double, std::milli> elapsed = finish - start;
        // std::cout << "Elapsed time: " << elapsed.count() << " ms\n";

        // Calculate time left until next iteration
        auto elapsed_since_last_iteration = std::chrono::steady_clock::now() - last_iteration_time;
        auto time_to_sleep = loop_duration - elapsed_since_last_iteration;

        // If there's time left, sleep
        if (time_to_sleep > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(time_to_sleep);
        } else {
            // If the iteration took longer than expected, consider it as an overrun
            // You may handle this situation as needed, e.g., log, adjust loop duration, etc.
        }

        last_iteration_time = std::chrono::steady_clock::now();
    }
    return 0;
}