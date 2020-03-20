#pragma once

#include <vector>
#include <Eigen/Dense>
#include "spdlog/spdlog.h"

struct PointTimeDist;

using Path = std::vector<Eigen::VectorXd>;
using PathTimeDist = std::vector<PointTimeDist>;

struct PointTimeDist{
public:
    double ts;
    double dist;
    Eigen::VectorXd point;

    PointTimeDist(){
        ts = 0;
        dist=0;
        point = Eigen::VectorXd::Zero(3);
    }

    PointTimeDist(double time, double distance, Eigen::VectorXd pointVec){
        ts = time;
        dist = distance;
        point  = pointVec;
    }
};

struct LineBesie{
  public:
    Eigen::VectorXd _p0 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd _p1 = Eigen::VectorXd::Zero(3);
    LineBesie(Eigen::VectorXd p0, Eigen::VectorXd p1){
      _p0=p0;
      _p1=p1;
    }
    Eigen::VectorXd getPoint(double time){
      return (1-time)*_p0+time*_p1;
    }

    double getDistance(){
      return sqrtf(powf((_p1[0]-_p0[0]),2)+powf((_p1[1]-_p0[1]),2));
    }
};


class TrjPlaner{
public:
  TrjPlaner(double timeDelay, double maxLinDistance, double maxAccel, double maxVelocity, double heigh);
  bool getPathWithTime(Path & path, double velocity, PathTimeDist & out);
  // From px to meters
  void transformPointsAndScale(const Path & path, Path & out);

private:
  double _timeDelay;
  double _maxAccel;
  double _maxVelocity;
};
