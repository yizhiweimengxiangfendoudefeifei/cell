#pragma once

#include <cstdint>

namespace PanoSimSensorBus {
    //////////////////////////////////////////////////////////////////////////
#pragma pack(push, 1)
    struct MonoCamera_G_Header {
        std::int32_t time;
    };
    struct MonoCamera_G_Item {
        std::int8_t r;
        std::int8_t g;
        std::int8_t b;
    };
    /*! \brief  单目相机传感器总线数据结构定义
     *  name: panosim.{bus_id}.MonoCamera_G.{number}
     *  format: time@i,{height*width}@[,r@b,g@b,b@b
     */
    struct MonoCamera_G {
        MonoCamera_G_Header header;
        MonoCamera_G_Item* items;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const MONO_DETECTOR_LANE_FORMAT = "Timestamp@i,4@[,Lane_ID@i,Lane_Distance@d,Lane_Car_Distance_Left@d,"
        "Lane_Car_Distance_Right@d,Lane_Curvature@d,Lane_Coefficient_C0@d,Lane_Coefficient_C1@d,Lane_Coefficient_C2@d,"
        "Lane_Coefficient_C3@d,Lane_Class@b";
    constexpr std::uint32_t MONO_DETECTOR_LANE_ITEM_MAX_COUNT = 4;
#pragma pack(push, 1)
    struct MonoDetector_Lane_Header {
        std::int32_t Timestamp;
        std::int32_t width;
    };
    struct MonoDetector_Lane_Item {
        std::int32_t Lane_ID;
        double Lane_Distance;
        double Lane_Car_Distance_Left;
        double Lane_Car_Distance_Right;
        double Lane_Curvature;
        double Lane_Coefficient_C0;
        double Lane_Coefficient_C1;
        double Lane_Coefficient_C2;
        double Lane_Coefficient_C3;
        std::int8_t Lane_Class;
    };
    /*! \brief  单目车道线传感器总线数据结构定义
     *  name: panosim.{bus_id}.MonoDetector_Lane.{number}
     *  format: Timestamp@i,4@[,Lane_ID@i,Lane_Distance@d,Lane_Car_Distance_Left@d,Lane_Car_Distance_Right@d,Lane_Curvature@d,
                Lane_Coefficient_C0@d,Lane_Coefficient_C1@d,Lane_Coefficient_C2@d,Lane_Coefficient_C3@d,Lane_Class@b
     */
    struct MonoDetector_Lane {
        MonoDetector_Lane_Header header;
        MonoDetector_Lane_Item items[MONO_DETECTOR_LANE_ITEM_MAX_COUNT];
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const MONO_DETECTOR_OBJECT_FORMAT = "Timestamp@i,64@[,OBJ_ID@i,OBJ_Class@b,OBJ_X@d,OBJ_Y@d,OBJ_Z@d,"
        "OBJ_Velocity@d,OBJ_Length@d,OBJ_Width@d,OBJ_Height@d";
    constexpr std::uint32_t MONO_DETECTOR_OBJECT_ITEM_MAX_COUNT = 64;
#pragma pack(push, 1)
    struct MonoDetector_Object_Header {
        std::int32_t Timestamp;
        std::int32_t width;
    };
    struct MonoDetector_Object_Item {
        std::int32_t OBJ_ID;
        std::int8_t OBJ_Class;
        double OBJ_X;
        double OBJ_Y;
        double OBJ_Z;
        double OBJ_Velocity;
        double OBJ_Length;
        double OBJ_Width;
        double OBJ_Height;
    };
    /*! \brief  单目目标真值传感器总线数据结构定义
     *  name: panosim.{bus_id}.MonoDetector_Object.{number}
     *  format: Timestamp@i,64@[,OBJ_ID@i,OBJ_Class@b,OBJ_X@d,OBJ_Y@d,OBJ_Z@d,OBJ_Velocity@d,OBJ_Length@d,OBJ_Width@d,OBJ_Height@d
     */
    struct MonoDetector_Object {
        MonoDetector_Object_Header header;
        MonoDetector_Object_Item items[MONO_DETECTOR_OBJECT_ITEM_MAX_COUNT];
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const MONO_DETECTOR_TRAFFICLIGHT_FORMAT = "Timestamp@i,4@[,TrafficLight_Direction@b,TrafficLight_State@b,"
        "TrafficLight_Timer@i";
    constexpr std::uint32_t MONO_DETECTOR_TRAFFICLIGHT_ITEM_MAX_COUNT = 4;
#pragma pack(push, 1)
    struct MonoDetector_Trafficlight_Header {
        std::int32_t Timestamp;
        std::int32_t width;
    };
    struct MonoDetector_Trafficlight_Item {
        std::int8_t TrafficLight_Direction;
        std::int8_t TrafficLight_State;
        std::int32_t TrafficLight_Timer;
    };
    /*! \brief  单目交通灯真值传感器总线数据结构定义
     *  name: panosim.{bus_id}.MonoDetector_Trafficlight.{number}
     *  format: Timestamp@i,4@[,TrafficLight_Direction@b,TrafficLight_State@b,TrafficLight_Timer@i
     */
    struct MonoDetector_Trafficlight {
        MonoDetector_Trafficlight_Header header;
        MonoDetector_Trafficlight_Item items[MONO_DETECTOR_TRAFFICLIGHT_ITEM_MAX_COUNT];
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
#pragma pack(push, 1)
    struct FisheyeCamera_G_Header {
        std::int32_t time;
        std::int32_t width;
    };
    struct FisheyeCamera_G_Item {
        std::int8_t r;
        std::int8_t g;
        std::int8_t b;
    };
    /*! \brief  鱼眼相机传感器总线数据结构定义
     *  name: panosim.{bus_id}.FisheyeCamera_G.{number}
     *  format: time@i,{height*width}@[,r@b,g@b,b@b
     */
    struct FisheyeCamera_G {
        FisheyeCamera_G_Header header;
        FisheyeCamera_G_Item* items;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const RADAR_OBJLIST_G_FORMAT = "time@i,64@[,OBJ_ID@i,OBJ_Class@i,OBJ_S_Azimuth@d,OBJ_S_Elevation@d,OBJ_S_Velocity@d,"
        "OBJ_S_Range@d,OBJ_RCS@d";
    constexpr std::uint32_t RADAR_OBJLIST_G_ITEM_MAX_COUNT = 64;
#pragma pack(push, 1)
    struct Radar_ObjList_G_Header {
        std::int32_t time;
        std::int32_t width;
    };
    struct Radar_ObjList_G_Item {
        std::int32_t OBJ_ID;
        std::int32_t OBJ_Class;
        double OBJ_S_Azimuth;
        double OBJ_S_Elevation;
        double OBJ_S_Velocity;
        double OBJ_S_Range;
        double OBJ_RCS;
    };
    /*! \brief  目标级毫米波雷达传感器总线数据结构定义
     *  name: panosim.{bus_id}.Radar_ObjList_G.{number}
     *  format: time@i,64@[,OBJ_ID@i,OBJ_Class@i,OBJ_S_Azimuth@d,OBJ_S_Elevation@d,OBJ_S_Velocity@d,OBJ_S_Range@d,OBJ_RCS@d
     */
    struct Radar_ObjList_G {
        Radar_ObjList_G_Header header;
        Radar_ObjList_G_Item items[RADAR_OBJLIST_G_ITEM_MAX_COUNT];
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const LIDAR_OBJLIST_G_FORMAT = "time@i,1024@[,OBJ_ID@i,OBJ_Class@b,OBJ_Shape@i,OBJ_S_X@d,OBJ_S_Y@d,OBJ_S_Z@d,OBJ_S_Dist@d,OBJ_S_Azimuth@d,OBJ_S_Elevation@d,OBJ_Ego_Vx@d,OBJ_Ego_Vy@d,OBJ_Ego_Heading@d,OBJ_Length@d,OBJ_Width@d,OBJ_Height@d";
    constexpr std::uint32_t LIDAR_OBJLIST_G_ITEM_MAX_COUNT = 1024;
#pragma pack(push, 1)
    struct Lidar_ObjList_G_Header {
        std::int32_t time;
        std::int32_t width;// 大于0的时候有障碍物
    };
    struct Lidar_ObjList_G_Item {
        std::int32_t OBJ_ID;
        std::int8_t OBJ_Class;
        std::int32_t shape;
        double OBJ_S_X;
        double OBJ_S_Y;
        double OBJ_S_Z;
        double OBJ_S_Dist;
        double OBJ_S_Azimuth;// 方位角
        double OBJ_S_Elevation;
        double OBJ_Ego_Vx;
        double OBJ_Ego_Vy;
        double OBJ_Ego_Heading;
        double OBJ_Length;
        double OBJ_Width;
        double OBJ_Height;
    };
    /*! \brief  目标级激光雷达传感器总线数据结构定义
     *  name: panosim.{bus_id}.Lidar_ObjList_G.{number}
     *  format: time@i,1024@[,OBJ_ID@i,OBJ_Class@b,OBJ_S_X@d,OBJ_S_Y@d,OBJ_S_Z@d,OBJ_S_Dist@d,OBJ_S_Azimuth@d,OBJ_S_Elevation@d,
                OBJ_Ego_Vx@d,OBJ_Ego_Vy@d,OBJ_Ego_Heading@d,OBJ_Length@d,OBJ_Width@d,OBJ_Height@d
     */
    struct Lidar_ObjList_G {
        Lidar_ObjList_G_Header header;
        Lidar_ObjList_G_Item items[LIDAR_OBJLIST_G_ITEM_MAX_COUNT];
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
#pragma pack(push, 1)
    struct Lidar_PointCloud_G_Header {
        std::int32_t Timestamp;
        std::int32_t width;
    };
    struct Lidar_PointCloud_G_Item {
        float x;
        float y;
        float z;
        float intensity;
    };
    /*! \brief  点云级激光雷达传感器总线数据结构定义
     *  name: panosim.{bus_id}.Lidar_PointCloud_G.{number}
     *  format: Timestamp@i,{beams*measurements}@[,x@f,y@f,z@f,intensity@f
     */
    struct Lidar_PointCloud_G {
        Lidar_PointCloud_G_Header header;
        Lidar_PointCloud_G_Item* items;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const ULTRA_SONAR_G_FORMAT = "time@i,64@[,ID@i,OBJ_S_Range@d,OBJ_Spl@d";
    constexpr std::uint32_t ULTRA_SONAR_G_ITEM_MAX_COUNT = 64;
#pragma pack(push, 1)
    struct UltraSonar_G_Header {
        std::int32_t time;
        std::int32_t width;
    };
    struct UltraSonar_G_Item {
        std::int32_t ID;
        double OBJ_S_Range;
        double OBJ_Spl;
    };
    /*! \brief  超声波雷达传感器总线数据结构定义
     *  name: panosim.{bus_id}.UltraSonar_G.{number}
     *  format: time@i,64@[,ID@i,OBJ_S_Range@d,OBJ_Spl@d
     */
    struct UltraSonar_G {
        UltraSonar_G_Header header;
        UltraSonar_G_Item items[ULTRA_SONAR_G_ITEM_MAX_COUNT];
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const PARKING_SPACE_DETECTOR_FORMAT = "time@i,64@[,x1@d,y1@d,x2@d,y2@d,x3@d,y3@d,x4@d,y4@d";
    constexpr std::uint32_t PARKING_SPACE_DETECTOR_ITEM_MAX_COUNT = 64;
#pragma pack(push, 1)
    struct ParkingSpaceDetectorHeader {
        std::int32_t time;
        std::int32_t width;
    };
    struct ParkingSpaceDetectorItem {
        double x1;
        double y1;
        double x2;
        double y2;
        double x3;
        double y3;
        double x4;
        double y4;
    };
    /*! \brief  空车位检测传感器总线数据结构定义
     *  name: panosim.{bus_id}.ParkingSpaceDetector.{number}
     *  format: time@i,64@[,x1@d,y1@d,x2@d,y2@d,x3@d,y3@d,x4@d,y4@d
     */
    struct ParkingSpaceDetector {
        ParkingSpaceDetectorHeader header;
        ParkingSpaceDetectorItem items[PARKING_SPACE_DETECTOR_ITEM_MAX_COUNT];
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const GNSS_FORMAT = "Timestamp@i,Longitude@d,Latitude@d,Altitude@d,Heading@d,Velocity@d";
    /*! \brief  全球导航卫星系统传感器总线数据结构定义
     *  name: panosim.{bus_id}.GNSS.{number}
     *  format: Timestamp@i,Longitude@d,Latitude@d,Altitude@d,Heading@d,Velocity@d
     */
#pragma pack(push, 1)
    struct GNSS {
        std::int32_t Timestamp;
        double Longitude;
        double Latitude;
        double Altitude;
        double Heading;
        double Velocity;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const IMU_FORMAT = "Timestamp@i,ACC_X@d,ACC_Y@d,ACC_Z@d,Gyro_X@d,Gyro_Y@d,Gyro_Z@d,Compass@d";
    /*! \brief  惯性测量单元传感器总线数据结构定义
     *  name: panosim.{bus_id}.IMU.{number}
     *  format: Timestamp@i,ACC_X@d,ACC_Y@d,ACC_Z@d,Gyro_X@d,Gyro_Y@d,Gyro_Z@d,Compass@d
     */
#pragma pack(push, 1)
    struct IMU {
        std::int32_t Timestamp;
        double ACC_X;
        double ACC_Y;
        double ACC_Z;
        double Gyro_X;
        double Gyro_Y;
        double Gyro_Z;
        double Compass;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
#pragma pack(push, 1)
    struct DepthCamera_G_Header {
        std::int32_t time;
        std::int32_t width;
    };
    struct DepthCamera_G_Item {
        std::int8_t r;
        std::int8_t g;
        std::int8_t b;
    };
    /*! \brief  深度相机传感器总线数据结构定义
     *  name: panosim.{bus_id}.DepthCamera_G.{number}
     *  format: time@i,{height*width}@[,r@b,g@b,b@b
     */
    struct DepthCamera_G {
        DepthCamera_G_Header header;
        DepthCamera_G_Item* items;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
#pragma pack(push, 1)
    struct SegmentCamera_G_Header {
        std::int32_t time;
        std::int32_t width;
    };
    struct SegmentCamera_G_Item {
        std::int8_t r;
        std::int8_t g;
        std::int8_t b;
    };
    /*! \brief  语义分割相机传感器总线数据结构定义
     *  name: panosim.{bus_id}.SegmentCamera_G.{number}
     *  format: time@i,{height*width}@[,r@b,g@b,b@b
     */
    struct SegmentCamera_G {
        SegmentCamera_G_Header header;
        SegmentCamera_G_Item* items;
    };
#pragma pack(pop)

} // namespace PanoSimSensorBus
