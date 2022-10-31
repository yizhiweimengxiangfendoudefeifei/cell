#pragma once

#include <windows.h>
#include <Export.h>

bool Initialize(const VectorParam& params, const MapFunction& functions)
{
    return SetFunctions(functions);
}

std::vector<TrafficSignData> getTrafficSign(double distance = 200)
{
    return _getTrafficSign(distance);
}

std::vector<ObstacleData> getObstacle(double distance = 200)
{
    return _getObstacle(distance);
}

std::vector<ObstacleDataV2> getObstacleV2(double distance = 200)
{
    return _getObstacleV2(distance);
}

std::vector<RsuData> getRSU(double distance = 200)
{
    return _getRSU(distance);
}
