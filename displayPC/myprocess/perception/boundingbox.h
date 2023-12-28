#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

//#include "myutility/cutility.h"
#include "eigen3/Eigen/Dense"
#include <queue>

struct Velocity
{
  float vx = 0;
  float vy = 0;
  Velocity operator+(Velocity& vrh)
  {
    Velocity vlh;
    vlh.vx = this->vx + vrh.vx;
    vlh.vy = this->vy + vrh.vy;
    return vlh;
  }
};
struct ObjectBox
{
  //  these three are for displaying box
  Eigen::Vector3f trans;
  Eigen::Quaternionf rotation;
  Eigen::Vector3f size; // width, height, depth,

  uint16_t id;  //障碍物ID
  double x;     //纵向距离,单位：米
  double y;     //横向距离，单位：米
  float vx;     //纵向速度，单位：米/秒
  float vy;     //横向速度，单位：米/秒

  double track_x; //用于追踪的点 track_x；
  double track_y; //           track_y;

  float predict_x; //预测点，lost时用。
  float predict_y;

  float ax;
  float ay;

  float heading;  //#朝向角，单位：度，正北方向为0度，顺时针增加。
  float length;
  float width;
  float height;

  uint8_t type;  //#障碍物类型 0-未知；1-动物；2-人；3-非机动车；4-机动车；5-石块；6-土堆；7-其他。
  uint16_t age;       // 生命周期，按照跟踪情况累加

  uint8_t probability;  // 障碍物可信度
  uint8_t attribute;  // 障碍物属性（0-实时检测，1-预测帧）
  uint8_t moveState;  // 运动状态 0-静止 1-运动

  int lost;
  bool label;

  std::deque<Velocity> velocityQue;
};

#endif // BOUNDINGBOX_H
