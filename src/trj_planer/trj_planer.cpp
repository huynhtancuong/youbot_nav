#include "trj_planer.h"
#include <iostream>
#include <property_manager/property_manager.h>


TrjPlaner::TrjPlaner(double timeDelay, double maxLinDistance, double maxAccel, double maxVelocity, double heigh){
  _maxAccel = maxAccel;
  _maxVelocity = maxVelocity;
  _timeDelay = timeDelay;
  spdlog::info("TrjPlaner::TrjPlaner");
}

bool TrjPlaner::getPathWithTime(Path & path, double velocity, PathTimeDist & out){
  if(velocity>_maxVelocity) return false;
  double globalTimePart;
  double globalDistPart;
  PointTimeDist pt;
  double currentDist;
  double timePart;
  double timeNorm;
  for( int i=0; i < path.size()-1; i++){
    LineBesie line(path[i], path[i+1]);
    currentDist=line.getDistance();
    timePart = currentDist/velocity;
    timeNorm = 1.0 / timePart;
    globalDistPart += currentDist;
    for(double t = 0; t <= timePart; t+= _timeDelay){
        PointTimeDist pt(globalTimePart+t, globalDistPart, line.getPoint(t*timeNorm));
      out.emplace_back(pt);
    }
    globalTimePart+=timePart;
  }
  return true;
}
