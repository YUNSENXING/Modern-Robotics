//     _     _    __   __    _______    _______    _______
//      \___/       \_/      |______    |______    |______
//     _/   \_       |       ______|    ______|    ______|
// #include<bits/stdc++.h>
#include<string>
#include<iostream>
#include<cstring>
#include<algorithm>
#include<vector>
#include<cstdio>
#include<cmath>
#include<queue>
#include<cstring>
#include<stack>
#include<unordered_map>
#include<map>
#include<deque>
#include<sstream>
#include<fstream>
#include<limits>
#include<unordered_set>
#include<random>


using namespace std;

typedef long long ll;
typedef pair<int,int> PII;
typedef pair<int,string> PIS;
typedef pair<ll,int> PLI;
const int N = 2e3+10;


constexpr float EPS = 0.009f;

struct Obstacles{
    float x, y, r;
}Ob[N];

struct Node{
    int ID;
    float x, y;
    float Hvalue;
}L[N];

struct ListEdge{
    int list1, list2;
    float cost;
     bool operator <(const ListEdge &L)const{
        if( cost != L.cost){
            return cost < L.cost;
        }
        return list1 < L.list1;
     }


}LE[N];


float Gen_Random_Number(float Min_Number, float Max_Number){
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(Min_Number,Max_Number);
    float random_number = dis(gen);
    return random_number;
}


bool check_valid(float x, float y, float cx, float cy, float cr){
    if(pow(x - cx,2) + pow(y - cy, 2) > pow(cr,2) + EPS){
        return true;
    }else{
        return false;
    }
}


string line_Obstacles;
int idx_Obstacles = 0;

bool check_collision(float x1, float y1, float x2, float y2, float cx, float cy, float cr){
    float dx = x2 - x1;
    float dy = y2 - y1;

    float fx = cx - x1;
    float fy = cy - y1;

    float number = fx * dx + fy * dy;
    float denom = dx * dx + dy * dy;

    float t = number / denom;

    if(t >= 0 && t <= 1){
        float cx_x1 = x1 + t * dx;
        float cy_y1 = y1 + t * dy;

        if(!check_valid(cx_x1, cy_y1, cx, cy, cr)){
            return true;
        }
    }

    return false;
    
}


void solve(){

    ifstream file_Obstacles("/Users/chichenghongye/Downloads/Control/EIT_Master/DD2410 Introduction to Robotics/Modern Robotics/Code/Class4_Unit2_Sampling-Based_Planning_Project/Code/obstacles.csv");

    // string line_Obstacles;
    // int idx_Obstacles = 0;
    

    while(getline(file_Obstacles,line_Obstacles)){
        if(line_Obstacles.empty() || line_Obstacles[0] == '#'){
            continue;
        }else{
            stringstream ss(line_Obstacles);
            string item_Obstacles;

            vector<string> fields_Obstacles;
            while (getline(ss, item_Obstacles, ',')) {
                fields_Obstacles.push_back(item_Obstacles);
            }

            Ob[idx_Obstacles] = {stof(fields_Obstacles[0]),stof(fields_Obstacles[1]),stof(fields_Obstacles[2])/2};
            
            idx_Obstacles ++;

        }
    }

    file_Obstacles.close();

    // 随机选点
    // add the start information

    Node Start_Node = {1, -0.5, -0.5, float(sqrt(pow(-0.5-0.5, 2) + pow(-0.5-0.5, 2))  )};
    L[1] = Start_Node;
    int idx_Node = 2;

    queue<Node> Open;
    Open.push(Start_Node);

    // cout << Current_Node.ID << " " << Current_Node.x << " " << Current_Node.y << endl;
    bool is_find_goal = false;

    while(!is_find_goal){
        bool is_success = true;

        auto Current_Node = Open.back();


        for(int i = 0 ; i < idx_Obstacles; i ++){
            if(check_collision(Current_Node.x, Current_Node.y, 0.5, 0.5, Ob[i].x, Ob[i].y, Ob[i].r)){
                is_success = false;
            }
        }

        if(is_success){
            is_find_goal = true;
            L[idx_Node] = {idx_Node, 0.5, 0.5, 0};
            Open.push({idx_Node, 0.5, 0.5});
            idx_Node ++;
        }else{
            bool is_valid = true;
            float randomX = Gen_Random_Number(-0.5, 0.5);
            float randomY = Gen_Random_Number(-0.5, 0.5);

            for(int i = 0 ; i < idx_Obstacles ; i ++){
                if(!check_valid(randomX, randomY, Ob[i].x, Ob[i].y, Ob[i].r) || check_collision(Current_Node.x, Current_Node.y, randomX, randomY, Ob[i].x, Ob[i].y, Ob[i].r)){
                    is_valid = false;
                }
            }



            if(is_valid){
                Open.push({idx_Node, randomX, randomY, float(sqrt(pow(randomX - 0.5, 2) + pow(randomY - 0.5, 2) ))});
                L[idx_Node] = {idx_Node, randomX, randomY, float(sqrt(pow(randomX - 0.5, 2) + pow(randomY - 0.5, 2) ) )};
                idx_Node ++;
            }
        }
    }

    // cout << "轨迹为： " << endl;

    vector<Node> Path;
    int idx_Path = 0;

    while(!Open.empty()){
        auto P = Open.front();
        // cout << P.ID << " " << P.x << " " << P.y << endl;
        Path.push_back({P.ID, P.x, P.y});
        Open.pop();
    }

    for(int i = 1 ; i < idx_Node  ; i ++){
        LE[i].list1 = L[i].ID;
        LE[i].list2 = L[i + 1].ID;
        LE[i].cost = sqrt( pow(L[i + 1].x - L[i].x, 2) +  pow(L[i + 1].y - L[i].y, 2) );   
        // cout << L[i].ID << " " << L[i].x << " " << L[i].y << endl;
    }


    // cout << idx_Node << endl;


    // cout << endl;

    // for(int i = 1 ; i < idx_Node - 1 ; i ++){
    //     cout << LE[i].list1 << " " << LE[i].list2 << " " << LE[i].cost << endl;
    // }cout << endl;



    std::ofstream node_file("nodes.csv"); 
    for(int i = 1 ; i < idx_Node ; i ++){
        node_file << L[i].ID << ',' << L[i].x << ',' << L[i].y << ',' << L[i].Hvalue << '\n';
    }

    std::ofstream edges_file("edges.csv"); 
    for(int i = 1 ; i < idx_Node - 1 ; i ++){
        edges_file << LE[i].list1 << ',' << LE[i].list2 << ',' << LE[i].cost << '\n';
    }


    std::ofstream path_file("path.csv"); 
    for(int i = 0 ; i < Path.size() ; i ++){
        path_file << Path[i].ID;
        if(i + 1 != Path.size()){
            path_file << ',';
        }
    }


    return ;
    

}

int main(){

    solve();

    return 0;
}