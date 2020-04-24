# 障碍物种类
int32 type
# 障碍物在本体坐标系下的位置
xyz_msg position
# 障碍物在本体坐标系下的运动信息
float64 radius
xyz_msg velocity
# 枚举类型，分为运动障碍物和静止障碍物两种
int32 mobile   = 0
int32 immobile = 1
float64 probability
