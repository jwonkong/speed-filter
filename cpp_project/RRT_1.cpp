#include <iostream>
#include <string>
#include <time.h>
#include <vector>
#include <algorithm>
#include <fstream>
#include <cmath>
using namespace std;

#define distance_object_to_line 2
#define N 3

const int width = 10;
const int height = 80;
const float delta_t = 0.5;
const float start_x = 0;
const float start_y = 0;
const float start_v = 19;

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
    Node* lastNode = nodeList.back();

    sample->x = lastNode->x + delta_t*N;
    sample->y = lastNode->y + 0.5*(-accel)*pow(delta_t*N, 2) + (lastNode->v)*delta_t*N;
    sample->parent = lastNode;

    // cout << "  " << endl;
    // cout << "start  " << accel << endl;
    // cout << "sample x : " << sample->x << endl;
    // cout << "sample y : " << sample->y << endl;
}

bool collision(Node* sample, int accel)
{
    Node* parent = sample->parent;
    //y = a(x-x_0)^2 + b(x-x_0) + c
    float a = (parent->a)/2;
    float b = (parent->v);
    float c = (parent->y);
    float tmpDist;
    float lineGrad;
    float interceptY; 
    float tmpX, tmpY;

    if ((sample->x - parent->x) != 0)   lineGrad = (sample->y - parent->y) / (sample->x - parent->x);
    else    lineGrad = 0;

    interceptY = (parent->y - lineGrad * parent->x);

    if (sample->y > height || sample->y < 0) {
        // cout << "Y OVER" << endl;
        delete sample;
        return false;
    }
    else {
        for (auto o : objectList) {
            if (o.first > parent->x) {
                for (float x1 = 0; x1 <= sample->x; x1+=delta_t) {
                    tmpX = x1;
                    tmpY = a*pow(x1, 2) + b*x1 + c;
                    tmpDist = sqrt(pow((o.first - tmpX), 2) + pow((o.second - tmpY), 2));
                    if (tmpDist < distance_object_to_line) {   
                        // cout << "Object touch" << endl;
                        // cout << "tmpDist : " << tmpDist << endl;
                        delete sample;
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

void AddPointsToLShape(float x, float y, float horizontalLength, float verticalHeight, float gap) {
    for (float i = x; i <= x + horizontalLength; i += gap) objectList.push_back(make_pair(i, y));
    for (float j = y; j <= y + verticalHeight; j += gap) objectList.push_back(make_pair(x, j));
}

void InitNode()
{
    for (auto a : nodeList)
    {
        Node* remove = a;
        delete a;
    }
    nodeList = vector<Node*>();
}

int main()
{   
    ofstream fobject("object.txt");
    ofstream fsample("sampling.txt");
    ofstream faccel("acceleration.txt");
    
    clock_t start, finish;
    double duration;
    start = clock();

    bool result = false;
    bool pathfound = false;
    bool noway = false;
    
    Node* ptNode = nullptr;
    Node* head = new Node;
    head->x = start_x;
    head->y = start_y;
    head->v = start_v;
    nodeList.push_back(head);

    AddPointsToLShape(2, 50, 1, 20, 0.1);

    int iterator = 0;
    int cnt = 0;
    while (pathfound == false && noway == false)
    {
        for (int i = iterator ; i < 6 ; i++) {
            ptNode = new Node;
            Sampling(ptNode, i);
            result = collision(ptNode, i);
            cnt++;
            if (result) {
                ptNode->x = ptNode->parent->x + delta_t;
                ptNode->y = 0.5*(-i)*pow(delta_t, 2) + (ptNode->parent->v)*delta_t + ptNode->parent->y;
                ptNode->v = ptNode->parent->v - i*delta_t;
                ptNode->a = -i;
                nodeList.push_back(ptNode);
                iterator = 0;

                // cout << "Curve !!!!!!!!!!!!!!!!!" << endl;
                // cout << "ptNode x :" << ptNode->x << endl;
                // cout << "ptNode y :" << ptNode->y << endl;
                // cout << "ptNode v :" << ptNode->v << endl;
                // cout << "ptNode a :" << ptNode->a << endl;
                // cout << "fin " << endl;
                // cout << " " << endl;
                break;
            }

            else if (i == 5 && !result) {
                for (int j = nodeList.size() - 1; j >= 0; j--) {
                    Node* node = nodeList[j];
                    if (node->a == -5) {
                        nodeList.pop_back();
                    }
                    else {
                        iterator = -(nodeList.back()->a) + 1;
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
        if (lastX == 10 || lastV <= 0)    pathfound = true;
    }
    
    if (noway == true) {
        cout << "noway" << endl;
    }
    ///clock
    finish = clock();
    duration = (double)(finish - start) / CLOCKS_PER_SEC;
    cout << duration << "초" << endl;

    cout << "found : " << nodeList.size() << endl;
    cout << "try : " << cnt << endl;

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