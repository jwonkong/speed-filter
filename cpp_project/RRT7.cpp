#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <random>
#include <unistd.h>
using namespace std;

#define distance_object_to_line 4
#define velocity_limit 22

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
    bool collision(Node* sample, Node*& parent);
    float Filter(float cmd_acceleration, float current_acceleration, float velocity);
    float findPath(float start_x, float start_y, float start_v, float start_a);
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


bool PathFinder::collision(Node* sample, Node*& parent) {
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


float PathFinder::Filter(float cmd_acceleration, float current_acceleration, float velocity) {
    float x_new_start = velocity/5;
    float y_new_start = (0.5*cmd_acceleration)*x_new_start*x_new_start + velocity*x_new_start;
    float v_new_start = velocity + cmd_acceleration*x_new_start;
    float Target_A;
    float tmpY;
    float tmpDist;
    bool new_start_check = true;

    for (auto o : objectList) {
        if (o.first <= x_new_start) {
            tmpY = 0.5*(cmd_acceleration)*(o.first)*(o.first) + (velocity)*(o.first);
            tmpDist = abs(o.second - tmpY);
            if (tmpDist <= distance_object_to_line) {
                new_start_check = false;
                break;
            }
        }
    }

    if (velocity > velocity_limit || !new_start_check){
        Target_A = findPath(0, 0, velocity, 0);
        if (velocity > velocity_limit) cout << "velocity limit" << endl;
        else cout << "Collision warning" << endl;
        return Target_A;
    }

    else {
        Target_A = findPath(x_new_start, y_new_start, velocity, cmd_acceleration);
        if (Target_A < -5) {
            cout << "(0,0) gen" << endl;
            return Target_A = findPath(0, 0, velocity, 0);
        }
        else if (Target_A != cmd_acceleration) {
            if (Target_A < cmd_acceleration) cout << "More decel" << endl;
            else cout << "Target A = Input command" << endl;
            return min(Target_A, cmd_acceleration);
        }
    }

    cout << "Target A = Input command" << endl;
    Target_A = cmd_acceleration;
    return Target_A;
}

float PathFinder::findPath(float start_x, float start_y, float start_v, float start_a) {
    Node* head = new Node;
    head->x = start_x;
    head->y = start_y;
    head->v = start_v;
    nodeList.push_back(head);

    float width = start_v/5 + 3;
    float delta_t = 0.5;
    float iterator = start_a;
    bool result = false;
    while (pathfound == false)
    {   
        Node* parent = nodeList.back();
    
        for (float i = iterator ; i >= -5 ; i--) {
            ptNode = new Node;
            Sampling(ptNode, parent , i);
            result = collision(ptNode, parent);

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
    if (gap1 > 0.2) gap1 = 0.1;
    
    for (float i = b; i < b + verticalHeight; i += gap2) {
        for (float x = a; x < xEnd + gap1; x += gap1) {
            float y = m * (x-a) + b;
            objectList.push_back(make_pair(x, y));
            // cout << "x : " << x << "y : " << y << endl;
        }
    }
}


void saveData(const vector<Node*>& nodeList, const vector<pair<float, float>>& objectList) {
    ofstream fobject("object.txt");
    ofstream fpath("path.txt");
    ofstream faccel("cmd_acceleration.txt");

    for (auto a : objectList)   fobject << a.first << "," << a.second << endl;
    fobject.close();

    for (Node* node : nodeList) fpath << node->x << "," << node->y << endl;
    fpath.close();

    for (Node* node : nodeList) faccel << node->x << "," << node->a << endl;
    faccel.close();
}

int main() {
    for (int i = 0 ; i < 1000000000; i++) {
        auto start = std::chrono::high_resolution_clock::now();
///Front Object Info
        float front_obj_a = 0;
        float front_obj_v = 1.01238;
        float front_obj_s = 7.0919;
///Currunt State
        float VehicleState_speed = 3.94;
        float VehicleState_Acceleration = 3; // pid
///Acceleration Input Command
        float LongitudinalCommand = -0.1;
///Object
        PathFinder pathFinder;

        pathFinder.AddLinearFunctionPoints(0, front_obj_s, front_obj_v, 6, VehicleState_speed/5 + 3, 2*distance_object_to_line/VehicleState_speed, 2*distance_object_to_line);

        float Target_A = pathFinder.Filter(LongitudinalCommand, 0.5 , VehicleState_speed, velocity_limit, distance_object_to_line);

        cout << "My Cmd: " << Target_A << endl;

        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = finish - start;
        std::cout << "Elapsed time: " << elapsed.count() << " ms\n";
    
        saveData(pathFinder.nodeList, pathFinder.objectList);
    }
    return 0;
}