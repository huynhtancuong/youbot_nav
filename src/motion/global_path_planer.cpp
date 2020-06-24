#include "global_path_planer.h"

GlobalPathPlaner::~GlobalPathPlaner() {
    _graph_map_ptr.reset();
}

GlobalPathPlaner::GlobalPathPlaner(GraphMapPtr graph_map_ptr):_bspline(graph_map_ptr->dim()) {
    _graph_map_ptr = graph_map_ptr;
    _initial_point = Eigen::VectorXd::Zero(graph_map_ptr->dim());
    _goal = 0;
    _start_point = 0;
    _way.reserve(0);
    _path.reserve(0);
    _keypoints_path.reserve(0);
}

bool GlobalPathPlaner::plan(int goal, int start_point , Eigen::VectorXd & initial_point) {
    _goal = goal;
    _start_point = start_point;
    _initial_point = initial_point;
    _gen_path();
}

bool GlobalPathPlaner::plan_by_way(std::vector<int> & way, Eigen::VectorXd &initial_point) {

    _initial_point = initial_point;
    _way = way;
    std::cout<<"start way point: "<<_way[0]<< std::endl;
    _start_point = _way.front();
    _goal = _way.back();
    _gen_path();

}

bool GlobalPathPlaner::_gen_path() {
    _keypoints_path.erase(_keypoints_path.begin(), _keypoints_path.end());
    _keypoints_path.emplace_back(_initial_point);
    for( int i = 0; i< _way.size(); i++){
        std::cout<<"graph map point coords: "<<_graph_map_ptr->point(_way[i]).coords<< std::endl;
        _keypoints_path.emplace_back(_graph_map_ptr->point(_way[i]).coords);
    }
    _bspline.generate_spline(_keypoints_path);
    std::cout<<"gen path length: "<< _bspline.len()<< std::endl;
    return true;
}

bool GlobalPathPlaner::get_point(double dist ,Eigen::VectorXd & out_point) {
    if(dist<0.0 || dist >_bspline.len()) return -1;
    _bspline.point(dist, out_point);
}

double GlobalPathPlaner::get_path_length() {
    return _bspline.len();
}

void GlobalPathPlaner::get_path(Path & out_path){
    out_path = _path;
}