#pragma once

#include <string>
#include <unordered_map>

#include <Interface.h>

//////////////////////////////////////////////////////////////////////////
/// @brief 返回目标尺寸
typedef std::tuple<double, double, double> (*fpGetObjectSize)(object_type type, int shape);

/// @brief 返回目标子类型
typedef object_subtype (*fpGetObjectSubtype)(object_type type, int shape);

/// @brief 返回目标角点坐标
typedef VctVertex (*fpGetObjectVertex)(object_type type, int shape, double X, double Y, double Z, double Yaw, double Pitch, double Roll);

/// @brief 返回主车角点坐标
typedef VctVertex (*fpGetEgoVertex)();

/// @brief 返回四根车道线的点集和类型
typedef LaneLines (*fpGetLaneLines)();

/// @brief 返回天气数据结构
typedef Environment (*fpGetWeatherInfo)();

/// @brief 返回前方交通信号灯id和位置
typedef TrafficLightData (*fpGetTrafficLight)();

/// @brief 返回范围内的交通标志牌
typedef std::vector<TrafficSignData> (*fpGetTrafficSign)(double distance);

/// @brief 返回范围内的障碍物
typedef std::vector<ObstacleData> (*fpGetObstacle)(double distance);

/// @brief 返回范围内的障碍物(包含静态车辆), <id,type,shape,x,y,z,yaw,pitch,roll>
typedef std::vector<ObstacleDataV2> (*fpGetObstacleV2)(double distance);

/// @brief 返回范围内的停车位的角点坐标 (从车辆左后点开始顺时针)
typedef std::vector<ParkingSpotData> (*fpGetParkingSpots)(double distance);

/// @brief 返回范围内的停车位信息 (id,是否上锁,特殊限制None/Disabled/Woman/Family/EV) 和 角点坐标(从车辆左后点开始顺时针)
typedef std::vector<ParkingSpotDataV2> (*fpGetParkingSpotsV2)(double distance);

/// @brief 返回范围内的RSU
typedef std::vector<RsuData> (*fpGetRSU)(double distance);

/// @brief 是否在Junction内部
typedef bool (*fpIsInsideJunction)(double x, double y, int nJunctionIndex);

/// @brief 返回坐标系原点偏移
typedef UtmOriginData (*fpGetUtmOrigin)();

/// @brief 返回仿真时间起点偏移
typedef int (*fpGetUtcOrigin)();

/// @brief 返回驾驶任务路径
typedef std::vector<std::tuple<int, DirectionType>> (*fpGetTaskRoute)(void);

/// @brief 返回当前Edge
typedef int (*fpGetCurrentEdge)();

/// @brief 返回Edge所包含的Lane
typedef std::vector<int> (*fpGetEdgeLanes)(int nEdgeIndex);

/// @brief 返回Lane前方的可行驶方向
typedef std::vector<DirectionType> (*fpGetLaneDirection)(int nLaneIndex);

/// @brief 返回Lane宽度
typedef double (*fpGetLaneWidth)(int nLaneIndex);

/// @brief 返回车道中心线点集
typedef std::vector<std::tuple<double, double>> (*fpGetLaneShape)(int nLaneIndex);

/// @brief 返回Junction形状点集
typedef std::vector<std::tuple<double, double>> (*fpGetJunctionShape)(int nJunction);


/// @brief 停止当前仿真实验
typedef void (*fpStopSimulation)();
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
extern fpGetObjectSize getObjectSize;
extern fpGetObjectSubtype getObjectSubtype;
extern fpGetObjectVertex getObjectVertex;
extern fpGetEgoVertex getEgoVertex;
extern fpGetLaneLines getLaneLines;
extern fpGetWeatherInfo getWeatherInfo;
extern fpGetTrafficLight getTrafficLight;
extern fpGetTrafficSign _getTrafficSign;
extern fpGetObstacle _getObstacle;
extern fpGetObstacleV2 _getObstacleV2;
extern fpGetParkingSpots getParkingSpots;
extern fpGetParkingSpotsV2 getParkingSpotsV2;
extern fpGetRSU _getRSU;
extern fpIsInsideJunction isInsideJunction;
extern fpGetUtmOrigin getUtmOrigin;
extern fpGetUtcOrigin getUtcOrigin;

extern fpGetTaskRoute getTaskRoute;
extern fpGetCurrentEdge getCurrentEdge;
extern fpGetEdgeLanes getEdgeLanes;
extern fpGetLaneDirection getLaneDirection;
extern fpGetLaneWidth getLaneWidth;
extern fpGetLaneShape getLaneShape;
extern fpGetJunctionShape getJunctionShape;

extern fpStopSimulation stopSimulation;
//////////////////////////////////////////////////////////////////////////

/*!
 *  @brief     index, runtime export functions to dll
 *  @remark    do not change order, append new item at the end
 */
enum class FunctionName : unsigned char {
    GetObjectSize = 0,
    GetObjectSubtype,
    GetObjectVertex,
    GetLaneLines,
    GetWeatherInfo,
    GetEgoVertex,
    GetTrafficLight,
    GetTrafficSign,
    GetObstacle,
    StopSimulation,
    IsInsideJunction,

    GetTaskRoute,
    GetCurrentEdge,
    GetEdgeLanes,
    GetLaneDirection,
    GetLaneWidth,
    GetLaneShape,
    GetJunctionShape,
    GetParkingSpots,
    GetRSU,
    GetUtmOrigin,
    GetUtcOrigin,
    GetObstacleV2,
    GetParkingSpotsV2
};

#ifdef _WIN32
    #ifndef PANOSIM_API
        #ifdef PANOSIM_API_EXPORTS
            #define PANOSIM_API __declspec(dllexport)
        #else // PANOSIM_API_EXPORTS
            #define PANOSIM_API __declspec(dllimport)
        #endif // PANOSIM_API_EXPORTS
    #endif // !PANOSIM_API
#else // _WIN32
    #define PANOSIM_API 
#endif // _WIN32

using MapFunction = std::unordered_map<FunctionName, void*>;
bool SetFunctions(const MapFunction& functions);
using VectorParam = std::vector<std::string>;
extern "C" bool PANOSIM_API Initialize(const VectorParam & params, const MapFunction & mapFunctions);
