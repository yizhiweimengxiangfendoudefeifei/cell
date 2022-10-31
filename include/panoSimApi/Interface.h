#pragma once

#include <string>
#include <vector>

// return value for invalid queries (especially vehicle is not on the road), see Position::INVALID
const double INVALID_DOUBLE_VALUE = -1073741824.0;

// return value for invalid queries (especially vehicle is not on the road), see Position::INVALID
const int INVALID_INT_VALUE = -1073741824;


/// @brief 目标类型
enum class object_type : unsigned char
{
    vehicle = 0,/*< 车辆(Car/Van/Bus/OtherVehicle) */
    pedestrian,/*< 行人 */
    other,/*< 其他目标 */
    obstacle,/*< 障碍物 */

    unknown/*< 未知目标类型 */
};

/// @brief 目标子类型
enum class object_subtype : unsigned char
{
    Car = 0,
    Van,
    Bus,
    OtherVehicle,
    Pedestrian,
    NonMotorVehicle,
    Others,

    unknown/*< 未知目标子类型 */
};

/// @brief 降水类型
enum class PrecipitationType : unsigned char
{
    Rain = 0,/*< 雨 */
    Snow,/*< 雪 */

    unknown/*< 未知降水类型 */
};

/// @brief 车道线类型
enum class LaneLineType : unsigned char {
    None = 0,/*< 没有线 */
    SingleWhite,/*< 白色实线 */
    SingleYellow,/*< 黄色实线 */
    BrokenWhite,/*< 白色虚线 */
    BrokenYellow,/*< 黄色虚线 */
    DoubleWhite,/*< 白色双实线 */
    DoubleYellow,/*< 黄色双实线 */

    unknown/*< 未知车道线类型 */
};

/// @brief 路口行驶方向
enum class DirectionType : unsigned char {
    straight = 0,   /*< 直行 */
    left,           /*< 左转 */
    right,          /*< 右转 */
    u_turn,         /*< 调头 */
    unknown
};


/// @brief 停车位地锁状态
enum class ParkingSpotLockStatus : unsigned char {
    On = 0,
    Off
};

/// @brief 停车位限制属性
enum class ParkingSpotPermit : unsigned char {
    None = 0,
    Disabled,
    Woman,
    Family,
    EV
};

/// @brief 大气
typedef struct _EnvironmentAir {
    double Temperature;/*< 温度 */
    double Pressure;/*< 压力 */
    double Humidity;/*< 湿度 */
}EnvironmentAir;

/// @brief 降水
typedef struct _EnvironmentPrecipitation {
    PrecipitationType Type;/*< 降水类型(Rain/Snow) */
    double ParticleSize;/*< 降水颗粒大小 */
    double ParticleCapacity;/*< 降水颗粒分布 */
    double FallingAlpha;/*< 降水方向Alpha */
    double FallingBeta;/*< 降水方向Beta */
    double FallingSpeed;/*< 降水速度 */
}EnvironmentPrecipitation;

/// @brief 雾
typedef struct _EnvironmentFog {
    double Visibility;/*< 能见度(0-1000) */
}EnvironmentFog;

/// @brief 环境
typedef struct _Environment {
    EnvironmentAir air;/*< 大气 */
    EnvironmentPrecipitation precipitation;/*< 降水 */
    EnvironmentFog fog;/*< 雾 */
}Environment;

/// @brief 角点坐标
using VctVertex = std::vector<std::tuple<double, double, double>>;

/// @brief 返回目标尺寸
std::tuple<double, double, double> GetObjectSize(object_type type, int shape);

/// @brief 返回目标子类型
object_subtype GetObjectSubtype(object_type type, int shape);

/// @brief 返回目标角点坐标
VctVertex GetObjectVertex(object_type type, int shape, double X, double Y, double Z, double Yaw, double Pitch, double Roll);

/// @brief 返回主车角点坐标
VctVertex GetEgoVertex();

/// @brief 车道线点集
using VctPoint = std::vector<std::tuple<double, double>>;
/// @brief 四根车道线点集和类型
using LaneLines = std::tuple<VctPoint, LaneLineType, VctPoint, LaneLineType, VctPoint, LaneLineType, VctPoint, LaneLineType>;
/// @brief 返回四根车道线的点集和类型
LaneLines GetLaneLines();

/// @brief 返回天气数据结构
Environment GetWeatherInfo();

/// @brief 返回前方交通信号灯id和位置
using TrafficLightData = std::tuple<int, double, double, double, double, double, double>;
TrafficLightData GetTrafficLight();

/// @brief 返回范围内的交通标志牌
using TrafficSignData = std::tuple<int, double, double, double, double, double, double>;
std::vector<TrafficSignData> GetTrafficSign(double distance = 200);

/// @brief 返回范围内的障碍物, <shape,x,y,z,yaw,pitch,roll>
using ObstacleData = std::tuple<int, double, double, double, double, double, double>;
std::vector<ObstacleData> GetObstacle(double distance = 200);

/// @brief 返回范围内的障碍物(包含静态车辆), <id,type,shape,x,y,z,yaw,pitch,roll>
using ObstacleDataV2 = std::tuple<int, object_type, int, double, double, double, double, double, double>;
std::vector<ObstacleDataV2> GetObstacleV2(double distance = 200);

/// @brief 返回范围内的停车位的角点坐标(从车辆左后点开始顺时针)
using ParkingSpotData = std::tuple<double, double, double, double, double, double, double, double>;
std::vector<ParkingSpotData> GetParkingSpots(double distance);

/// @brief 返回范围内的停车位信息 (id,是否上锁,特殊限制None/Disabled/Woman/Family/EV) 和 角点坐标(从车辆左后点开始顺时针)
using ParkingSpotDataV2 = std::tuple<int, ParkingSpotLockStatus, ParkingSpotPermit, double, double, double, double, double, double, double, double>;
std::vector<ParkingSpotDataV2> GetParkingSpotsV2(double distance);

/// @brief 返回范围内的RSU, [(shape,x,y,z,yaw,pitch,roll,fov,range)]
using RsuData = std::tuple<int, double, double, double, double, double, double, double, double>;
std::vector<RsuData> GetRSU(double distance = 200);

/// @brief 是否在Junction内部
bool IsInsideJunction(double x, double y, int nJunctionIndex);

/// @brief 返回坐标系原点偏移
using UtmOriginData = std::tuple<std::int32_t, double, double, double>;
UtmOriginData GetUtmOrigin();

/// @brief 返回仿真时间起点偏移
int GetUtcOrigin();

/// @brief 返回驾驶任务路径
std::vector<std::tuple<int, DirectionType>> GetTaskRoute();

/// @brief 返回当前Edge
int GetCurrentEdge();

/// @brief 返回Edge所包含的Lane
std::vector<int> GetEdgeLanes(int nEdgeIndex);

/// @brief 返回Lane前方的可行驶方向
std::vector<DirectionType> GetLaneDirection(int nLaneIndex);

/// @brief 返回Lane宽度
double GetLaneWidth(int nLaneIndex);

/// @brief 返回车道中心线点集
std::vector<std::tuple<double, double>> GetLaneShape(int nLaneIndex);

/// @brief 返回Junction形状点集
std::vector<std::tuple<double, double>> GetJunctionShape(int nJunction);

/// @brief 停止当前仿真实验
void StopSimulation();
