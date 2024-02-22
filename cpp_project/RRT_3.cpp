#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <algorithm>
#include <fstream>
#include <cmath>
using namespace std;

#define distance_object_to_line 5
#define N 3

const float width = 10;
const float delta_t = 0.5;

struct Node {
    float x = 0;
    float y = 0;
    float v = 0;
    int a = 0;
    Node* parent = nullptr;
};
vector<Node*> nodeList;

vector<pair<float, float>> objectList;

void Sampling(Node* sample, int accel) {
    Node* parent = nodeList.back();

    sample->x = parent->x + delta_t*N;
    sample->y = parent->y + 0.5*(accel)*pow(delta_t*N, 2) + (parent->v)*delta_t*N;
    sample->v = parent->v + accel*delta_t*N;
    sample->parent = parent;

    // cout << "  " << endl;
    // cout << "start  " << accel << endl;
    // cout << "sample x : " << sample->x << endl;
    // cout << "sample y : " << sample->y << endl;
}

bool collision(Node* sample, int accel) {
    Node* parent = sample->parent;

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
                delete sample;
                return false;
            }
        }
    }
    
    return true;
}

void AddPointsToLShape(float x, float y, float horizontalLength, float verticalHeight, float gap1, float gap2) {
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

void Square(float x, float y, float horizontalLength, float verticalHeight, float gap1, float gap2) {
    objectList.push_back(make_pair(x, y)); 
    objectList.push_back(make_pair(x + horizontalLength, y));
    objectList.push_back(make_pair(x, y + verticalHeight));
    objectList.push_back(make_pair(x + horizontalLength, y + verticalHeight));

    for (float i = x + gap1; i < x + horizontalLength; i += gap1) objectList.push_back(make_pair(i, y));
    for (float i = x + gap1; i < x + horizontalLength; i += gap1) objectList.push_back(make_pair(i, y + verticalHeight));
    for (float j = y + gap2; j < y + verticalHeight; j += gap2) objectList.push_back(make_pair(x, j));
    for (float j = y + gap2; j < y + verticalHeight; j += gap2) objectList.push_back(make_pair(x + horizontalLength, j));
}

void AddQuadraticFunctionPoints(float a, float b, float c, float xEnd, float gap) {
    for (float x = 0; x <= xEnd; x += gap) {
        float y = (a/2) * pow(x, 2) + b * x + c;
        objectList.push_back(make_pair(x, y));
    }
}

void InitNode() {
    for (auto a : nodeList) {
        Node* remove = a;
        delete a;
    }
    nodeList = vector<Node*>();
}

int main() {   
    ofstream fobject("object.txt");
    ofstream fsample("sampling.txt");
    ofstream faccel("acceleration.txt");
    
    auto start = std::chrono::high_resolution_clock::now();

    bool result = false;
    bool pathfound = false;
    bool noway = false;
///Front Object Info
    float front_obj_a = -2;
    float front_obj_v = 20;
    float front_obj_s = 50;
///Currunt State
    float VehicleState_speed = 22;
    float VehicleState_Acceleration = 1;
///Acceleration Input Command
    float LongitudinalCommand = 2;
///Object
    // AddPointsToLShape(2.8, 60, 0.5, 11, 0.1 ,2*distance_object_to_line);
    // AddPointsToLShape(1.3, 38, 0.6, 5, 0.1, 2*distance_object_to_line);
    // Square(2.8, 10, 0.5, 11, 0.2 ,2*distance_object_to_line);
    // Square(1.3, 18, 0.6, 5, 0.2, 2*distance_object_to_line);
    AddQuadraticFunctionPoints(front_obj_a/2, front_obj_v, front_obj_s, width, 0.2);

    float start_x = 0;
    float start_y = 0;
    float start_v = VehicleState_speed;
    Node* ptNode = nullptr;
    Node* head = new Node;
    head->x = start_x;
    head->y = start_y;
    head->v = start_v;
    nodeList.push_back(head);

    int iterator = 0;
    int cnt = 0;

    while (pathfound == false && noway == false)
    {
        for (int i = iterator ; i > -6 ; i--) {
            ptNode = new Node;
            Sampling(ptNode, -i);
            result = collision(ptNode, i);
            cnt++;
            if (result) {
                ptNode->x = ptNode->parent->x + delta_t;
                ptNode->y = 0.5*(i)*pow(delta_t, 2) + (ptNode->parent->v)*delta_t + ptNode->parent->y;
                ptNode->v = ptNode->parent->v + i*delta_t;
                ptNode->a = i;
                nodeList.push_back(ptNode);
                iterator = 0;
                break;
            }

            else if (i == -5 && !result) {
                for (int j = nodeList.size() - 1; j >= 0; j--) {
                    Node* node = nodeList[j];
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
            continue;
        }

        Node* lastNode = nodeList.back();
        float lastX = lastNode->x;
        float lastV = lastNode->v;
        // if (lastX > 3 && lastNode->v) {

        // }

        if (lastX == 10 || lastV <= 0)    pathfound = true;
    }
    
    if (noway) {
        cout << "Way_no" << endl;
        cout << "try : " << cnt << endl;
    }    
    if (pathfound) {
        cout << "Way_ok" << endl;
        cout << "point : " << nodeList.size() << endl;
        cout << "try : " << cnt << endl;
    }
    ///clock
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = finish - start;
    std::cout << "Elapsed time: " << elapsed.count() << " ms\n";



    //save data
    if (fobject.fail()) cout << "Error" << endl;
    for (auto a : objectList)   fobject << a.first << "," << a.second << "\n";
    fobject.close();

    for (Node* node : nodeList) fsample << node->x << "," << node->y << endl;
    fsample.close();

    for (Node* node : nodeList) faccel << node->x << "," << node->a << endl;
    faccel.close();
    
    InitNode();
    return 0;
}