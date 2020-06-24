#pragma once

#include <trj_planning/curves/besie_spline.h>
#include <trj_planning/trj_planer.h>
#include <path_planning/graph_map.h>

class GlobalPathPlaner;
typedef std::shared_ptr<GlobalPathPlaner> GlobalPathPlanerPtr;

class GlobalPathPlaner{
public:
    ~GlobalPathPlaner();
    GlobalPathPlaner(GraphMapPtr graph_map_ptr);
    bool plan(int goal, int start_point, Eigen::VectorXd & initial_point);
    bool plan_by_way(std::vector<int> & way, Eigen::VectorXd & initial_point);
    void get_path(Path & out_path);
    bool get_point(double dist ,Eigen::VectorXd & out_point);
    double get_path_length();
private:
    GraphMapPtr _graph_map_ptr;
    int _goal;
    int _start_point;
    Eigen::VectorXd _initial_point;
    Path _keypoints_path;
    BesieSpline _bspline;
    Path _path;
    std::vector<int> _way;
    bool _gen_path();
};
