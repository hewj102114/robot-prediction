#include "robo_navigation/global_planner.hpp"


void Floyd::loadMatrix(const Mat& matrix){
    matrix.convertTo(weight_graph,CV_64FC1);
    node_count=matrix.rows;
    memcpy(arrDis,weight_graph.data,sizeof(arrDis));
}

void Floyd::initFloydGraph() {

    for (int i = 0; i < node_count; ++i) {
        for (int j = 0; j < node_count; ++j) {
            arrPath[i][j] = i;
        }
    }

    for (int k = 0; k < node_count; ++k)
        for (int i = 0; i < node_count; ++i)
            for (int j = 0; j < node_count; ++j) {
                if (arrDis[i][k] + arrDis[k][j] < arrDis[i][j]) {
                    arrDis[i][j] = arrDis[i][k] + arrDis[k][j];
                    arrPath[i][j] = arrPath[k][j];
                }
            }

   //cout<<"3-7:"<<arrDis[3][7]<<", 25-14: "<< arrDis[7][3]<<endl;
}

void Floyd::calcPath(int start, int end) {
    path.clear();
    if (start != end) {
        if (INF == arrDis[start][end]) {    //arrDis[start][end]>100
            cout << "INFINITE" << endl;
        } else {
            int k = end;
            do {
                k = arrPath[start][k];
                path.push_back(k);
            } while (k != start);

            reverse(path.begin(), path.end());
            path.push_back(end);
        }
    }
    else
        path.clear();
}

void Floyd::updateFloydGraph(int i,int j,int value){
    arrDis[i][j]=value;
    arrDis[j][i]=value;
    //initFloydGraph();
}
void Floyd::printPath(){
    for (int i=0;i<path.size() -1;i++){
        cout<<path[i]<<" - > ";
    }
    cout<<path.back()<<endl;
}
/*
int main(){
    FileStorage fs("../draw_map/matrix.xml", FileStorage::READ);
    Mat arrArcs, point_list;
    fs["Matrix"] >> arrArcs;
    fs["Point"] >> point_list;

    Floyd floyd(arrArcs);
    floyd.initFloydGraph();

    floyd.calcPath(1,12);
    floyd.printPath();

    floyd.updateFloydGraph(7,11,1000);
    floyd.calcPath(1,12);
    floyd.printPath();
}*/