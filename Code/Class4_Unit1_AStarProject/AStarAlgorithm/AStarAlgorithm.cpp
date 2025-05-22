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


using namespace std;

typedef long long ll;
typedef pair<int,int> PII;
typedef pair<int,string> PIS;
typedef pair<ll,int> PLI;
const int N = 2e3+10;

float Past_Cost[N];
float Optimist_Ctg[N];
float Estimate_Total_Cost[N];
int Parent_Node[N];


struct AllNodeList{
    int id;
    float x, y, Hvalue;
    bool operator <(const AllNodeList &L)const{
        if(Hvalue != L.Hvalue){
            return Hvalue > L.Hvalue;
        }

        return id < L.id;
    }
}L[N];

struct ListEdge{
    int list1, list2;
    float cost;
}LE[N];

struct CompareEstimateTotalCost{
    bool operator()(int a , int b) const{
        return Estimate_Total_Cost[a] > Estimate_Total_Cost[b];
    }
};


void solve(){

    ifstream file_Nodes("/Users/chichenghongye/Downloads/Control/EIT_Master/DD2410 Introduction to Robotics/Modern Robotics/Code/Class4_Unit1_AStarProject/AStarAlgorithmFile/nodes.csv");

    string line_Nodes;
    int idx_Nodes = 0;
    

    while(getline(file_Nodes,line_Nodes)){
        if(line_Nodes.empty() || line_Nodes[0] == '#'){
            continue;
        }else{
            stringstream ss(line_Nodes);
            string item_Nodes;

            vector<string> fields_Nodes;
            while (getline(ss, item_Nodes, ',')) {
                fields_Nodes.push_back(item_Nodes);
            }

            L[idx_Nodes] = {stoi(fields_Nodes[0]), stof(fields_Nodes[1]), stof(fields_Nodes[2]), stof(fields_Nodes[3])};
            idx_Nodes ++;

        }
    }

    file_Nodes.close();

    // read Node.csv Done

    
    // begin Edges.csv
    ifstream file_Edges("/Users/chichenghongye/Downloads/Control/EIT_Master/DD2410 Introduction to Robotics/Modern Robotics/Code/Class4_Unit1_AStarProject/AStarAlgorithmFile/edges.csv");

    string line_Edges;


    int idx_Edge = 0;

    while(getline(file_Edges,line_Edges)){
        if(line_Edges.empty() || line_Edges[0] == '#'){
            continue;
        }else{
            stringstream ss(line_Edges);
            string item_Edges;

            vector<string> fields_Edges;
            while (getline(ss, item_Edges, ',')) {
                fields_Edges.push_back(item_Edges);
            }
            
            LE[idx_Edge] = {stoi(fields_Edges[0]), stoi(fields_Edges[1]), stof(fields_Edges[2])};
            idx_Edge ++;


        }
    }

    file_Edges.close();

    // cout << "This is the Nodes.csv data " << endl;
    // for(int i = 0 ; i < idx_Nodes ; i ++){
    //     cout << L[i].id << " " << L[i].x << " " << L[i].y << " " << L[i].Hvalue << endl;
    // }

    // cout << "This is the Edges.csv data " << endl;
    // for(int i = 0 ; i < idx_Edge ; i ++){
    //     cout << LE[i].list1 << " " << LE[i].list2 << " " << LE[i].cost << endl;
    // } 


    // sort(L, L + idx_Nodes);

    

    // suppose that start node is 0, goal node is N
    int start_Node = 0;
    int goal_Node = idx_Nodes - 1;

    // initial the Matrix
    fill_n(Past_Cost, N, numeric_limits<float>::infinity());
    Past_Cost[0] = 0;

    for(int i = 0 ; i < idx_Nodes ; i ++){
        Optimist_Ctg[i] = L[i].Hvalue;
    }

    fill_n(Estimate_Total_Cost, N, numeric_limits<float>::infinity());
    Estimate_Total_Cost[start_Node] = Optimist_Ctg[start_Node];

    memset(Parent_Node, -1, sizeof(Parent_Node));

    // Start A* Algorithm

    priority_queue<int,vector<int> , CompareEstimateTotalCost> Open_Node;
    unordered_set<int> Closed_Node;

    Open_Node.push(start_Node);

    // cout << Open_Node.front().Hvalue << endl;

    while(Open_Node.size() != 0){

        auto Open_Node_First = Open_Node.top();
        Open_Node.pop();

        if(Open_Node_First == goal_Node){
            // find the goal node;
            break;
        }

        if(Closed_Node.count(Open_Node_First)){
            continue;
        }


        // update
        for(int i = 0 ; i < idx_Edge; i ++){

            // cout << LE[i].list1 << " " << LE[i].list2 << " " << LE[i].cost << endl;


            int Update_index = -1;
            if(LE[i].list1 - 1 == Open_Node_First || LE[i].list2 - 1 == Open_Node_First){

                if(LE[i].list1 - 1 == Open_Node_First){
                    Update_index = LE[i].list2 - 1;
                }else if(LE[i].list2 - 1 == Open_Node_First){
                    Update_index = LE[i].list1 - 1;
                }
            }

            if(Update_index == -1){
                continue;
            }


            // cout << Update_index << endl;


            float new_Estimate_Total_Cost = Past_Cost[Open_Node_First] + LE[i].cost;

            if(new_Estimate_Total_Cost < Past_Cost[Update_index]){
                Past_Cost[Update_index] = new_Estimate_Total_Cost;
                Estimate_Total_Cost[Update_index] = Past_Cost[Update_index] + Optimist_Ctg[Update_index];
                Parent_Node[Update_index] = Open_Node_First;
                Open_Node.push(Update_index);
            }
            
        }

        Closed_Node.insert(Open_Node_First);

    }

    vector<int> path;
    for(int i = goal_Node; i != -1 ; i = Parent_Node[i]){
        path.push_back(i + 1);
    }

    reverse(path.begin(), path.end());


    std::ofstream path_file("path.csv"); 
    for(int i = 0 ; i < path.size() ; i ++){
        path_file << path[i];
        if(i + 1 != path.size()){
            path_file << ',';
        }
    }

    path_file << '\n';
    path_file.close();



    return ;
    

}

int main(){

    solve();

    return 0;
}