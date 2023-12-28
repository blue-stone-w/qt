#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

#include "boundingbox.h"
#include <myutility/cutility.h>
#include <myutility/rutility.h>

class ObjectTracker
{
public:
  ObjectTracker();

  ~ObjectTracker();
  void setCurTime(const double& time);
  void track(std::vector<ObjectBox>& obj, double timeIn);
  std::vector<ObjectBox>& getObjResult();

private:
  void init();
  void predict();
  void match();
  bool buildList(std::vector<ObjectBox>& obj);
  bool isIntersect(ObjectBox& pre, ObjectBox& cur);
  void upGradeList();
  void upGradePreFrame();
  void track();
  // void calVelocity(const double time_diff);

  std::vector<ObjectBox> p_temp_list;
  std::vector<ObjectBox> p_preFrame_list;
  std::vector<ObjectBox> result_list_;
  double pre_time_;
  double cur_time_;
  double time_diff_ = 0.1;
  constexpr static int SPEED_AVERAGE_FRAMES = 5;
};

#endif // OBJECTTRACKER_H
