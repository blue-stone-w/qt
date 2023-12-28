#include "objecttracker.h"

ObjectTracker::ObjectTracker()
{
  pre_time_ = 0;
  cur_time_ = 0;
}
ObjectTracker::~ObjectTracker()
{
}

void ObjectTracker::setCurTime(const double& time)
{
  cur_time_ = time;
}

bool ObjectTracker::buildList(std::vector<ObjectBox>& obj)
{
  p_temp_list = obj;
  // time_diff_ = cur_time_ - pre_time_;

  if (pre_time_ == 0 || result_list_.empty())
  {
    init();
    upGradePreFrame();

    return false;
  }
  return true;
}

void ObjectTracker::predict()
{
  if (p_preFrame_list.empty())
  {
    return;
  }

  int ii = p_preFrame_list.size();
  for (int i = 0; i < ii; ++i)
  {
    //
    p_preFrame_list[i].predict_x = p_preFrame_list[i].track_x + p_preFrame_list[i].vx * time_diff_;
    p_preFrame_list[i].predict_y = p_preFrame_list[i].track_y + p_preFrame_list[i].vy * time_diff_;
  }
}

void ObjectTracker::init()
{
  result_list_ = p_temp_list;
  int ii = result_list_.size();
  for (int i = 0; i < ii; ++i)
  {
    result_list_[i].vx = 0;
    result_list_[i].vy = 0;
    result_list_[i].age = 1;
    result_list_[i].label = 0;
    result_list_[i].lost = 0;
  }
}

bool ObjectTracker::isIntersect(ObjectBox& pre, ObjectBox& cur)
{
  bool one = std::abs(pre.predict_x - cur.track_x) <= 0.5 * (pre.length + cur.length);
  bool two = std::abs(pre.predict_y - cur.track_y) <= 0.5 * (pre.width + cur.width);
  if (one && two)
    return true;
  else
    return false;
}

// The current frame matches the previous frame,

void ObjectTracker::match()
{
  int tl = p_temp_list.size();
  int pl = p_preFrame_list.size();
  bool ismatch = false;
  // search corresponding object in this frame for every object in last frame
  for (int i = 0; i < pl; ++i) // last frame
  {
    Velocity t_v;
    ismatch = false;
    for (int j = 0; j < tl; ++j) // this frame
    {
      if (p_temp_list[j].label) // object in this frame have been matched
      {
        continue;
      }

      ismatch = isIntersect(p_preFrame_list[i], p_temp_list[j]);
      if (!ismatch)
      {
        continue;
      }

      // update object
      p_temp_list[j].label = true;
      p_temp_list[j].id = p_preFrame_list[i].id;


      // update last velocity
      t_v.vx = (p_temp_list[j].track_x - p_preFrame_list[i].track_x) / time_diff_;
      t_v.vy = (p_temp_list[j].track_y - p_preFrame_list[i].track_y) / time_diff_;
      if (std::abs(p_preFrame_list[i].vx) < 0.01 || std::abs(p_preFrame_list[i].vx) > 10)
      {
        p_preFrame_list[i].vx = 0;
      }
      if (std::abs(p_preFrame_list[i].vy) < 0.01 || std::abs(p_preFrame_list[i].vy) > 10)
      {
        p_preFrame_list[i].vy = 0;
      }
      p_preFrame_list[i].vx = t_v.vx;
      p_preFrame_list[i].vy = t_v.vy;
      p_preFrame_list[i].velocityQue.emplace_back(t_v);
      while (p_preFrame_list[i].velocityQue.size() > SPEED_AVERAGE_FRAMES)
      {
        p_preFrame_list[i].velocityQue.pop_front();
      }
      if (p_preFrame_list[i].age >= SPEED_AVERAGE_FRAMES)
      {
        t_v = std::accumulate(p_preFrame_list[i].velocityQue.end()-SPEED_AVERAGE_FRAMES, p_preFrame_list[i].velocityQue.end(), Velocity());
        t_v.vx /= SPEED_AVERAGE_FRAMES;
        t_v.vy /= SPEED_AVERAGE_FRAMES;
        p_preFrame_list[i].vx = t_v.vx;
        p_preFrame_list[i].vy = t_v.vy;
      }

      // update last property
      p_preFrame_list[i].lost = 0;
      p_preFrame_list[i].attribute = 0;
      p_preFrame_list[i].age++;
      p_preFrame_list[i].x = p_temp_list[j].x;
      p_preFrame_list[i].y = p_temp_list[j].y;
      p_preFrame_list[i].track_x = p_temp_list[j].track_x;
      p_preFrame_list[i].track_y = p_temp_list[j].track_y;


      break; // have found correspondence
    } // endfor: have traversed objects in last frame or have found correspondence

    if (ismatch)
    {
      continue;
    }

    // if not matched
    p_preFrame_list[i].lost++;
    p_preFrame_list[i].age++;
//    p_preFrame_list[i].track_x = p_preFrame_list[i].predict_x; // not matched, use prediction as reference position
//    p_preFrame_list[i].track_y = p_preFrame_list[i].predict_y;
    p_preFrame_list[i].attribute = 1;
    p_preFrame_list[i].x = p_preFrame_list[i].x + p_preFrame_list[i].vx * time_diff_;
    p_preFrame_list[i].y = p_preFrame_list[i].y + p_preFrame_list[i].vy * time_diff_;

  } // endfor: last frame
}

void ObjectTracker::upGradeList()
{
  result_list_.clear();

  //
  int tl = p_temp_list.size();
  for (int i = 0; i < tl; ++i) // this frame
  {
    if (p_temp_list[i].label)
    {
      continue;
    }
    ObjectBox& c_obj = p_temp_list[i];
    c_obj.id += p_preFrame_list.size();  //
    // c_obj.lost++;

    p_preFrame_list.emplace_back(c_obj);
  }

  //
  int pl = p_preFrame_list.size();
  for (int i = 0; i < pl; ++i)
  {
    int temp_lost = p_preFrame_list[i].lost;
    if (temp_lost > 2)
    {
      continue;
    }
    result_list_.emplace_back(p_preFrame_list[i]);
  }

}

void ObjectTracker::upGradePreFrame()
{
  p_preFrame_list.clear();
  p_preFrame_list = result_list_;
  pre_time_ = cur_time_;
}

void ObjectTracker::track(std::vector<ObjectBox>& obj, double timeIn)
{
  setCurTime(timeIn);

  if (!buildList(obj))
  {
    return ;
  }

  predict();
  match();
  upGradeList();
  upGradePreFrame();

  return ;
}

std::vector<ObjectBox>& ObjectTracker::getObjResult()
{
  return result_list_;
}
