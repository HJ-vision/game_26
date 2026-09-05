#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
// 哨兵专有（原定义在 cboard.hpp，cboard 删除后挪到此处）
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};

struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;  //无人机专有
};

}  // namespace io

#endif  // IO__COMMAND_HPP