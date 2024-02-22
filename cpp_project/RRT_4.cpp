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
#define N 3

const float width = 5;

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
    bool noway;

    PathFinder() : pathfound(false), noway(false) {}

    void Sampling(Node* sample,Node*& parent, int accel, float delta_t);
    bool collision(Node* sample, Node*& parent, float delta_t);
    float Filter(float velocity, float acceleration, float delta_t);
    float findPath(float start_x, float start_y, float start_v);
    void InitNode();
    // void Front_Object(float a, float b, float c);
    // void Predicted_Object(float x, float y, float horizontalLength, float verticalHeight);
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
    sample->y = parent->y + 0.5*(accel)*pow(delta_t*N, 2) + (parent->v)*delta_t*N;
    sample->v = parent->v + accel*delta_t*N;
    sample->parent = parent;

    // cout << "  " << endl;
    // cout << "start  " << accel << endl;
    // cout << "sample x : " << sample->x << endl;
    // cout << "sample y : " << sample->y << endl;
}

bool PathFinder::collision(Node* sample, Node*& parent, float delta_t) {

    float tmpDist;
    float lineGrad;
    float interceptY; 
    float tmpX, tmpY;

    lineGrad = (sample->y - parent->y) / (delta_t*N);
    interceptY = (parent->y - lineGrad * parent->x);

    for (auto o : objectList) {
        if (o.first > parent->x && o.first <= sample->x) {
            tmpX = o.first;
            tmpY = lineGrad * tmpX + interceptY;
            tmpDist = abs(o.second - tmpY);
            if (tmpDist <= distance_object_to_line) {  
                // cout << "Object touch" << endl;
                // cout << "o.x " << o.first << endl;
                // cout << "o.y " << o.second <<endl;
                // cout << "lline " << lineGrad << endl;
                // cout << "tmpY " << tmpY <<endl;
                // cout << "tmpDist : " << tmpDist << endl;
                delete sample;
                return false;
            }
        }        
    }
    return true;
}

float PathFinder::Filter(float velocity, float acceleration, float delta_t) {
    float x_new_start = delta_t;
    float y_new_start = (0.5*acceleration)*pow(delta_t, 2) + velocity*delta_t;
    float v_new_start = velocity + acceleration*delta_t;

    float Target_A = findPath(x_new_start, y_new_start, v_new_start);

    if (Target_A < -10) {
        cout << "First : Fail" << endl;
        Target_A =  findPath(0, 0, velocity);
        if(Target_A < -10) {
            cout << "Second : Fail" << endl;
            return -10000;
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
    float delta_t = 0.5;
    bool result = false;
    while (pathfound == false)
    {
        for (int i = iterator ; i > -6 ; i--) {
            ptNode = new Node;
            Node*& parent = nodeList.back();
            Sampling(ptNode, parent , i, delta_t);
            result = collision(ptNode, parent, delta_t);
            
            if (result) {
                ptNode->x = ptNode->parent->x + delta_t;
                ptNode->y = 0.5*(i)*pow(delta_t, 2) + (ptNode->parent->v)*delta_t + ptNode->parent->y;
                ptNode->v = ptNode->parent->v + i*delta_t;
                ptNode->a = i;
                nodeList.push_back(ptNode);
                iterator = 0;
                break;

                // cout << "Curve !!!!!!!!!!!!!!!!!" << endl;
                // cout << "ptNode x :" << ptNode->x << endl;
                // cout << "ptNode y :" << ptNode->y << endl;
                // cout << "ptNode v :" << ptNode->v << endl;
                // cout << "ptNode a :" << ptNode->a << endl;
                // cout << "fin " << endl;
                // cout << " " << endl;
            }

            else if (i == -5 && !result) {
                for (int j = nodeList.size() - 1; j >= 0; j--) {
                    Node* node = nodeList[j];
                    // cout << node->a << endl;
                    if (node->a == -5) {
                        nodeList.pop_back();
                    }
                    else {
                        iterator = (nodeList.back()->a) - 1;
                        nodeList.pop_back();
                        break;
                    }
                }
            }
        }

        if (nodeList.empty()) {
            noway = true;
            return -1000;
        }
        
        // if (lastX <= 3) delta_t = 0.2;
        // else if ( 3 < lastX) delta_t = 0.5;
        if (ptNode->x >= width || ptNode->v <= 0)    break;
    }
    cout << "nodesize : " << nodeList.size() << endl;
    Node*& first_acceleration = nodeList.at(1);
    return first_acceleration->a;
}

void PathFinder::InitNode() {
    for (auto a : nodeList) {
        delete a;
    }
    nodeList.clear();
}

// void PathFinder::Front_Object(float a, float b, float c) {
//     float x_end;
    
//     if (a < 0) {
//         x_end = -b/a;
//         if (x_end >width) x_end = width;
//     }
//     else x_end = width;
    
//     float y_end = (a/2)*pow(x_end, 2) + b*x_end + c;
//     objectList.push_back(make_pair(x_end, y_end));
// }

// void PathFinder::Predicted_Object(float x, float y, float horizontalLength, float verticalHeight) {

// }

void PathFinder::AddQuadraticFunctionPoints(float a, float b, float c, float xEnd, float gap) {
    for (float x = 0; x <= xEnd; x += gap) {
        float y = (a/2) * pow(x, 2) + b * x + c;
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
        float front_obj_a = -4;
        float front_obj_v = 7;
        float front_obj_s = 3;
///Currunt State
        float VehicleState_speed = 17;
        float VehicleState_Acceleration = 3; // pid
///Acceleration Input Command
        float LongitudinalCommand = 0;
///Object
        PathFinder pathFinder;
    // pathFinder.AddQuadraticFunctionPoints(front_obj_a/2, front_obj_v, front_obj_s, width, 0.1);
    // pathFinder.Front_Object(front_obj_a/2, front_obj_v, front_obj_s);
        pathFinder.AddPointsSquare(2, 35, 2, 3, 0.01 ,2*distance_object_to_line);
    // pathFinder.AddPointsSquare(1.5, 20, 1.5, 10, 0.1 ,2*distance_object_to_line);
    // pathFinder.AddPointsToLShape(1, 13, 2, 3, 0.01 ,2*distance_object_to_line);
    // pathFinder.AddPointsToLShape(1.5, 10, 1.5, 10, 0.1 ,2*distance_object_to_line);

    
        float Target_A = pathFinder.Filter(VehicleState_speed, LongitudinalCommand, 0.5);

        cout << "My Cmd: " << Target_A << endl;


        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = finish - start;
        std::cout << "Elapsed time: " << elapsed.count() << " ms\n";
        // cout << "i : " << i << endl;
        
        // sleep(1);

    saveData(pathFinder.nodeList, pathFinder.objectList);
        // pathFinder.InitNode();

    }
    
    
    
    return 0;
}