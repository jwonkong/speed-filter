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

#define distance_object_to_line 2
#define N 5

const float width = 6;
const float delta_t = 0.4;

struct Node {
    float x = 0;
    float y = 0;
    float v = 0;
    int a = 0;
    Node* parent = nullptr;
};

class PathFinder {
public:
    vector<Node*> nodeList;
    vector<pair<float, float>> objectList;
    Node* ptNode = nullptr;
    bool pathfound;



    PathFinder() : pathfound(false) {}

    void Sampling(Node* sample,Node*& parent, int accel, float delta_t);
    bool collision(Node* sample, Node*& parent, float delta_t);
    float Filter(float acceleration, float velocity, float x_new_start);
    float findPath(float start_x, float start_y, float start_v);
    void AddQuadraticFunctionPoints(float a, float b, float c, float xEnd, float gap);
    void AddPointsToLShape(float x, float y, float horizontalLength, float verticalHeight, float gap1, float gap2);
    void AddPointsSquare(float x, float y, float horizontalLength, float verticalHeight, float gap1, float gap2);

    ~PathFinder() {
        for (Node* node : nodeList) delete node;
        nodeList.clear();
        objectList.clear();
    }
};

void PathFinder::Sampling(Node* sample, Node*& parent, int accel, float delta_t) {
    sample->x = parent->x + delta_t*N;
    sample->y = parent->y + 0.5*(accel)*delta_t*N*delta_t*N + (parent->v)*delta_t*N;
    sample->v = parent->v + accel*delta_t*N;
    sample->parent = parent;
}

bool PathFinder::collision(Node* sample, Node*& parent, float delta_t) {
    float tmpDist;
    float lineGrad;
    float interceptY; 
    float tmpX, tmpY;

    lineGrad = (sample->y - parent->y) / (delta_t*N);
    interceptY = (parent->y - lineGrad * parent->x);

    for (auto o : objectList) {
        if (o.first > parent->x && o.second > parent->y && o.first <= sample->x) {
            tmpX = o.first;
            tmpY = lineGrad * tmpX + interceptY;
            tmpDist = abs(o.second - tmpY);
            if (tmpDist <= distance_object_to_line) {  
                delete sample;
                return false;
            }
        }        
    }
    return true;
}

float PathFinder::Filter(float acceleration, float velocity, float x_new_start) {
    float y_new_start = (0.5*acceleration)*x_new_start*x_new_start + velocity*x_new_start;
    float v_new_start = velocity + acceleration*x_new_start;
    float Target_A = findPath(x_new_start, y_new_start, v_new_start);

    if (Target_A < -10) {
        cout << "First : ///////////////////////////////////Fail///////////////////////////////////" << endl;
        Target_A =  findPath(0, 0, velocity);
        if(Target_A < -10) {
            cout << "Second : ///////////////////////////////////Fail///////////////////////////////////" << endl;
            return Target_A;
        }
        else {
            cout << "Second : Success" << endl;
            return Target_A;
        } 
    }
    cout << "First : Success" << endl;
    Target_A = acceleration;
    return Target_A;
}

float PathFinder::findPath(float start_x, float start_y, float start_v) {
    Node* head = new Node;
    head->x = start_x;
    head->y = start_y;
    head->v = start_v;
    nodeList.push_back(head);

    int iterator = 0;
    bool result = false;
    int cnt=0;
    while (pathfound == false)
    {
        for (int i = iterator ; i > -6 ; i--) {
            ptNode = new Node;
            Node* parent = nodeList.back();
            Sampling(ptNode, parent , i, delta_t);
            result = collision(ptNode, parent, delta_t);
            cnt++;
            if (result) {
                ptNode->x = ptNode->parent->x + delta_t;
                ptNode->y = 0.5*(i)*delta_t*delta_t + (ptNode->parent->v)*delta_t + ptNode->parent->y;
                ptNode->v = ptNode->parent->v + i*delta_t;
                ptNode->a = i;
                nodeList.push_back(ptNode);
                if (nodeList.back()->x >= width || nodeList.back()->v <= 0) pathfound = true;
                iterator = 0;
                break;
            }
            else if (i == -5) {
                for (int j = nodeList.size() - 1; j >= 0; j--) {
                    Node* node = nodeList.back();
                    if (node->a == -5) {
                        delete node;
                        nodeList.pop_back();
                    }
                    else {
                        iterator = (nodeList.back()->a) - 1;
                        delete node;
                        nodeList.pop_back();
                        break;
                    }
                }
            }
        }
        if (nodeList.empty()) return -100;
    }
    cout << "try : " << cnt <<endl;
    return nodeList.at(1)->a;
}

void PathFinder::AddQuadraticFunctionPoints(float a, float b, float c, float xEnd, float gap) {
    for (float x = 0; x <= xEnd; x += gap) {
        float y = (a/2) * x * x + b * x + c;
        objectList.push_back(make_pair(x, y));
    }
}

void PathFinder::AddPointsToLShape(float x, float y, float horizontalLength, float verticalHeight, float gap1, float gap2) {
    objectList.push_back(make_pair(x, y)); 
    objectList.push_back(make_pair(x + horizontalLength, y));
    objectList.push_back(make_pair(x, y + verticalHeight));

    for (float j = y + gap2; j < y + verticalHeight; j += gap2) {
        objectList.push_back(make_pair(x, j));
    }

    for (float i = x; i <= x + horizontalLength; i += gap1) {
        objectList.push_back(make_pair(i, y));
    }
}

void PathFinder::AddPointsSquare(float x, float y, float horizontalLength, float verticalHeight, float gap1, float gap2) {
    objectList.push_back(make_pair(x, y)); 
    objectList.push_back(make_pair(x + horizontalLength, y));
    objectList.push_back(make_pair(x, y + verticalHeight));
    objectList.push_back(make_pair(x + horizontalLength, y + verticalHeight));

    for (float i = x + gap1; i < x + horizontalLength; i += gap1) objectList.push_back(make_pair(i, y));
    for (float i = x + gap1; i < x + horizontalLength; i += gap1) objectList.push_back(make_pair(i, y + verticalHeight));
    for (float j = y + gap2; j < y + verticalHeight; j += gap2) objectList.push_back(make_pair(x, j));
    for (float j = y + gap2; j < y + verticalHeight; j += gap2) objectList.push_back(make_pair(x + horizontalLength, j));
}

void saveData(const vector<Node*>& nodeList, const vector<pair<float, float>>& objectList) {
    ofstream fobject("object.txt");
    ofstream fsample("sampling.txt");
    ofstream faccel("acceleration.txt");

    for (auto a : objectList)   fobject << a.first << "," << a.second << "\n";
    fobject.close();

    for (Node* node : nodeList) fsample << node->x << "," << node->y << endl;
    fsample.close();

    for (Node* node : nodeList) faccel << node->x << "," << node->a << endl;
    faccel.close();
}

int main() {
    for (int i = 0 ; i < 10 ; i++) {
        auto start = std::chrono::high_resolution_clock::now();
///Front Object Info
        float front_obj_a = 0;
        float front_obj_v = 0;
        float front_obj_s = 45;
///Currunt State
        float VehicleState_speed = 20;
        // float VehicleState_Acceleration = 3; // pid
///Acceleration Input Command
        float LongitudinalCommand = 3;
///Object
        PathFinder pathFinder;

        pathFinder.AddQuadraticFunctionPoints(front_obj_a, front_obj_v, front_obj_s, width, 0.1);
        // pathFinder.AddPointsToLShape(5.2, 14, 1, 3, 0.1 ,2*distance_object_to_line);
        // pathFinder.AddPointsToLShape(4.1, 60, 2, 10, 0.1 ,2*distance_object_to_line); 
        // pathFinder.AddPointsToLShape(2.5, 60, 0.5, 3, 0.1 ,2*distance_object_to_line);
        pathFinder.AddPointsToLShape(1.2, 42, 1, 3, 0.1 ,2*distance_object_to_line);
        // pathFinder.AddPointsToLShape(5.3, 20, 1.5, 10, 0.1 ,2*distance_object_to_line);
        // pathFinder.AddPointsToLShape(1, 13, 2, 3, 0.01 ,2*distance_object_to_line);
        // pathFinder.AddPointsToLShape(1.5, 10, 1.5, 10, 0.1 ,2*distance_object_to_line);

        float Target_A = pathFinder.Filter(LongitudinalCommand, VehicleState_speed, delta_t);

        cout << "My Cmd: " << Target_A << endl;

        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = finish - start;
        std::cout << "Elapsed time: " << elapsed.count() << " ms\n";
    
        saveData(pathFinder.nodeList, pathFinder.objectList);
    }
    return 0;
}