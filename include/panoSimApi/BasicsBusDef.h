#pragma once

#include <cstdint>

namespace PanoSimBasicsBus {
    //////////////////////////////////////////////////////////////////////////
    const char* const EGO_FORMAT = "time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d";
#pragma pack(push, 1)
    /*! \brief  主车动力学输出数据
     *  name: panosim.{bus_id}.ego
     *  format: time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d
     */
    struct Ego {
        std::int32_t time;
        double x;
        double y;
        double z;
        double yaw;
        double pitch;
        double roll;
        double speed;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const EGO_EXTRA_FORMAT = "time@i,VX@d,VY@d,VZ@d,AVx@d,AVy@d,AVz@d,Ax@d,Ay@d,Az@d,AAx@d,AAy@d,AAz@d";
#pragma pack(push, 1)
    /*! \brief  主车动力学更多输出数据
     *  name: panosim.{bus_id}.ego_extra
     *  format: time@i,VX@d,VY@d,VZ@d,AVx@d,AVy@d,AVz@d,Ax@d,Ay@d,Az@d,AAx@d,AAy@d,AAz@d
     */
    struct EgoExtra {
        std::int32_t time;
        double VX;
        double VY;
        double VZ;
        double AVx;
        double AVy;
        double AVz;
        double Ax;
        double Ay;
        double Az;
        double AAx;
        double AAy;
        double AAz;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const TRAFFIC_FORMAT = "time@i,100@[,id@i,type@b,shape@i,x@f,y@f,z@f,yaw@f,pitch@f,roll@f,speed@f";
    constexpr std::uint32_t TRAFFIC_ITEM_MAX_COUNT = 100;
#pragma pack(push, 1)
    struct TrafficHeader {
        std::int32_t time;
        std::int32_t width;
    };
    struct TrafficItem {
        std::int32_t id;
        std::uint8_t type;
        std::int32_t shape;
        float x;
        float y;
        float z;
        float yaw;
        float pitch;
        float roll;
        float speed;

    };
    /*! \brief  交通参与物输出数据 (必须使用双缓冲DoubleBusReader来读取)
     *  name: panosim.{bus_id}.traffic
     *  format: time@i,100@[,id@i,type@b,shape@i,x@f,y@f,z@f,yaw@f,pitch@f,roll@f,speed@f
     */
    struct Traffic {
        TrafficHeader header;
        TrafficItem items[TRAFFIC_ITEM_MAX_COUNT];
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const EGO_TRAFFIC_FORMAT = "time@i,lane@i,station@d,lateral@d,internal@b,nextJunction@i";
#pragma pack(push, 1)
    /*! \brief  主车道路坐标系输出输出
     *  name: panosim.{bus_id}.ego_traffic
     *  format: time@i,lane@i,station@d,lateral@d,internal@b,nextJunction@i
     */
    struct EgoTraffic {
        std::int32_t time;
        std::int32_t lane;
        double station;
        double lateral;
        std::int8_t internal;
        std::int32_t nextJunction;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const TRAFFIC_LIGHT_FORMAT = "time@i,64@[,id@i,direction@b,state@b,timer@i";
    constexpr std::uint32_t TRAFFIC_LIGHT_ITEM_MAX_COUNT = 64;
#pragma pack(push, 1)
    struct TrafficLightHeader {
        std::int32_t time;
        std::int32_t width;
    };
    struct TrafficLightItem {
        std::int32_t id;
        std::int8_t direction;
        std::int8_t state;
        std::int32_t timer;
    };
    /*! \brief  交通信号灯输出数据
     *  name: panosim.{bus_id}.traffic_light
     *  format: time@i,64@[,id@i,direction@b,state@b,timer@i
     */
    struct TrafficLight {
        TrafficLightHeader header;
        TrafficLightItem items[TRAFFIC_LIGHT_ITEM_MAX_COUNT];
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const WARNING_FORMAT = "time@i,type@b,64@[,text@b";
    constexpr std::uint32_t WARNING_ITEM_MAX_COUNT = 64;
#pragma pack(push, 1)
    struct WarningHeader {
        std::int32_t time;
        std::int8_t type;
        std::int32_t width;
    };
    struct WarningItem {
        std::int8_t text;
    };
    /*! \brief  告警数据，其中type取值为0-Info、1-Warning、2-Error，text为warning文本
     *  name: panosim.{bus_id}.warning
     *  format: time@i,type@b,64@[,text@b
     */
    struct Warning {
        WarningHeader header;
        WarningItem items[WARNING_ITEM_MAX_COUNT];
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const GLOBAL_FORMAT = "time@i,variable@d";
#pragma pack(push, 1)
    /*! \brief  全局变量 (共10个)
     *  name: panosim.{bus_id}.global.[0-9]
     *  format: time@i,variable@d
     */
    struct Global {
        std::int32_t time;
        double variable;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const JUDGE_FORMAT = "time@i,judge@d";
#pragma pack(push, 1)
    /*! \brief  评估器 (共10个) 其中实时信号表示检测中间变量，最后一帧数据在实验结束时生成，表示最终评估结果
     *  name: panosim.{bus_id}.judge.[0-9]
     *  format: time@i,judge@d
     */
    struct Judge {
        std::int32_t time;
        double judge;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const EGO_CONTROL_FORMAT = "time@i,valid@b,throttle@d,brake@d,steer@d,mode@i,gear@i";
#pragma pack(push, 1)
    /*! \brief  外部驾驶员控制信号
     *  name: panosim.{bus_id}.ego_control
     *  format: time@i,valid@b,throttle@d,brake@d,steer@d,mode@i,gear@i
     */
    struct EgoControl {
        std::int32_t time;
        std::int8_t valid;
        double throttle;
        double brake;
        double steer;
        std::int32_t mode;
        std::int32_t gear;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const EGO_CONTROL_THROTTLE_FORMAT = "time@i,valid@b,throttle@d";
#pragma pack(push, 1)
    /*! \brief  油门独立控制信号
     *  name: panosim.{bus_id}.ego_control.throttle
     *  format: time@i,valid@b,throttle@d
     */
    struct EgoControlThrottle {
        std::int32_t time;
        std::int8_t valid;
        double throttle;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const EGO_CONTROL_BRAKE_FORMAT = "time@i,valid@b,brake@d";
    /*! \brief  刹车独立控制信号
     *  name: panosim.{bus_id}.ego_control.brake
     *  format: time@i,valid@b,brake@d
     */
#pragma pack(push, 1)
    struct EgoControlBrake {
        std::int32_t time;
        std::int8_t valid;
        double brake;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const EGO_CONTROL_STEER_FORMAT = "time@i,valid@b,steer@d";
    /*! \brief  方向盘独立控制信号
     *  name: panosim.{bus_id}.ego_control.steer
     *  format: time@i,valid@b,steer@d
     */
#pragma pack(push, 1)
    struct EgoControlSteer {
        std::int32_t time;
        std::int8_t valid;
        double steer;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const EGO_CONTROL_MODE_FORMAT = "time@i,valid@b,mode@i";
#pragma pack(push, 1)
    /*! \brief  挡位模式独立控制信号
     *  name: panosim.{bus_id}.ego_control.mode
     *  format: time@i,valid@b,mode@i
     */
    struct EgoControlMode {
        std::int32_t time;
        std::int8_t valid;
        std::int32_t mode;
    };
#pragma pack(pop)


    //////////////////////////////////////////////////////////////////////////
    const char* const EGO_CONTROL_GEAR_FORMAT = "time@i,valid@b,gear@i";
#pragma pack(push, 1)
    /*! \brief  手动挡位独立控制信号
     *  name: panosim.{bus_id}.ego_control.gear
     *  format: time@i,valid@b,gear@i
     */
    struct EgoControlGear {
        std::int32_t time;
        std::int8_t valid;
        std::int32_t gear;
    };
#pragma pack(pop)

} // namespace PanoSimBasicsBus
