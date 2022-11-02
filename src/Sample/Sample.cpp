#include <PanoSimApi.h>
#include <string>
#include <string_view>
#include <vector>
#include <iostream>
#include <fstream>

#include "reference_line.h"
#include "control.h"
/*主函数入口，panosim每10ms调用一次这个程序
*/
using std::string;
using std::string_view;
using std::vector;
using std::cout;
using std::endl;
using std::move;

using PanoSimSensorBus::LIDAR_OBJLIST_G_FORMAT;
using PanoSimSensorBus::Lidar_ObjList_G;
using PanoSimBasicsBus::EGO_CONTROL_FORMAT;
using PanoSimBasicsBus::EgoControl;
using PanoSimBasicsBus::EGO_FORMAT;
using PanoSimBasicsBus::Ego;

struct GlobalData {
    BusAccessor* lidar;
    BusAccessor* ego_control, *ego;
};

void PrintParameters(UserData* userData);
void SplitString(const string_view strSrc, const string_view strSeparator, vector<string>& vctSplitResult);
Eigen::MatrixXd vector_eigen(const std::vector<std::pair<double, double>>& input);

void ModelStart(UserData* userData) {
    PrintParameters(userData);

    auto pGlobal = new GlobalData;
    pGlobal->lidar = new BusAccessor(userData->busId, "Lidar_ObjList_G.0", LIDAR_OBJLIST_G_FORMAT);
    pGlobal->ego_control = new BusAccessor(userData->busId, "ego_control", EGO_CONTROL_FORMAT);
    pGlobal->ego = new BusAccessor(userData->busId, "ego", EGO_FORMAT);
    userData->state = pGlobal;
}

void ModelOutput(UserData* userData) {
    if (userData != nullptr) {
        auto pGlobal = static_cast<GlobalData*>(userData->state);// 类型转换
        if (pGlobal != nullptr) {
            referenceLine referenceline;// 生成一个参考线对象
            // 传感器
            Lidar_ObjList_G* pLidar = nullptr;
            if (pGlobal->lidar != nullptr) {
                pLidar = static_cast<Lidar_ObjList_G*>(pGlobal->lidar->GetHeader());
                referenceline.shape(pLidar);
                referenceline.findIndex(pLidar);
                referenceline.calcCenterPoint();// referenceline这个对象包含所有中心点的坐标
                // 插值函数中输入是eigen数据类型，我们的输入是vector<pair>,因此需要类型转换
                //Eigen::MatrixXd input = vector_eigen(referenceline.get_center_point_xy());
                /*std::cout << "input size: " << input.size() << std::endl;
                std::vector<std::pair<double, double>> output;
                referenceline.average_interpolation(input, output, 0.2, 0.6);*/
                //cout << "output.size: " << output.size() << endl;
                //referenceline.set_center_point_xy_final(output);// 得到最终的参考线
                //referenceline.get_center_point_xy_final() = output;// 得到最终的参考线
                /*cout << "OBJ_Class: " << pLidar->items->OBJ_Class << endl;
                cout << "OBJ_S_X: " << pLidar->items->OBJ_S_X << endl;
                cout << "width: " << pLidar->header.width << endl;*/

            }
            // 自车控制
            EgoControl* pEgoCtrl = nullptr;
            if (pGlobal->ego_control != nullptr) {
                pEgoCtrl = static_cast<EgoControl*>(pGlobal->ego_control->GetHeader());
            }
            // 自车状态类，在这里写控制
            std::vector<std::pair<double, double>> targetPath = referenceline.get_center_point_xy();// 参考路径
            //cout << "targetPath.size: " << targetPath.size() << endl;
            Ego* pEgo = nullptr;
            if (pGlobal->ego != nullptr) {
                pEgo = static_cast<Ego*>(pGlobal->ego->GetHeader());
                double steer = control::calculateSteering(targetPath, pEgo);
                cout << "steer: " << steer << endl;
                if (pEgo->speed * 3.6 > 10) {
                    pEgoCtrl->time = userData->time;
                    pEgoCtrl->valid = 1;
                    pEgoCtrl->throttle = 0;
                    pEgoCtrl->brake = 1;
                    pEgoCtrl->steer = steer;
                    pEgoCtrl->mode = 1;
                    pEgoCtrl->gear = 1;
                }
                else {
                    pEgoCtrl->time = userData->time;
                    pEgoCtrl->valid = 1;
                    pEgoCtrl->throttle = 1;
                    pEgoCtrl->brake = 0;
                    pEgoCtrl->steer = steer;
                    pEgoCtrl->mode = 1;
                    pEgoCtrl->gear = 1;
                }
            }
            if (pLidar != nullptr && pEgoCtrl != nullptr)
            {
                /*if (pLidar->header.width > 0) {
                    cout << "==============================" << endl;
                    pEgoCtrl->time = userData->time;
                    pEgoCtrl->valid = 1;
                    pEgoCtrl->throttle = 1;
                    pEgoCtrl->brake = 0;
                    pEgoCtrl->steer = 0;
                    pEgoCtrl->mode = 1;
                    pEgoCtrl->gear = 1;
                }*/
            }
        }
    }
}

void ModelTerminate(UserData* userData)
{
    if (userData->state != nullptr) {
        auto pGlobal = static_cast<GlobalData*>(userData->state);
        if (pGlobal != nullptr) {
            if (pGlobal->lidar != nullptr) {
                delete pGlobal->lidar;
                pGlobal->lidar = nullptr;
            }
            if (pGlobal->ego_control != nullptr) {
                delete pGlobal->ego_control;
                pGlobal->ego_control = nullptr;
            }
            delete pGlobal;
            userData->state = nullptr;
        }
    }
}

void SplitString(const string_view strSrc, const string_view strSeparator, vector<string>& vctSplitResult)
{
    vctSplitResult.clear();
    string::size_type nBegin = 0;
    string::size_type nEnd = strSrc.find(strSeparator);
    while (string::npos != nEnd) {
        vctSplitResult.emplace_back(move(strSrc.substr(nBegin, nEnd - nBegin)));
        nBegin = nEnd + strSeparator.size();
        nEnd = strSrc.find(strSeparator, nBegin);
    }
    if (nBegin != strSrc.length()) {
        vctSplitResult.emplace_back(move(strSrc.substr(nBegin)));
    }
}

void PrintParameters(UserData* userData)
{
    for (const auto& pairItem : userData->parameters) {
        cout << pairItem.first << ":" << pairItem.second << endl;
    }

    cout << userData->busId << endl;
    cout << userData->name << endl;

    const char* key_parameter = "Parameters";
    auto findParam = userData->parameters.find(key_parameter);
    if (findParam != userData->parameters.end()) {
        vector<string> vctParameter;
        constexpr string_view parameter_separator = ",";
        SplitString(findParam->second, parameter_separator, vctParameter);
        for (const auto& strParameter : vctParameter) {
            cout << strParameter << endl;
        }
    }
}

// 将vecotr数据类型转换为eigen
Eigen::MatrixXd vector_eigen(const std::vector<std::pair<double, double>> &input) {
    Eigen::MatrixXd out = Eigen::MatrixXd::Zero(input.size(), 3);
    for (int i = 0; i < input.size(); ++i) {
        out(i, 0) = input[i].first;// x
        out(i, 1) = input[i].second;// y
        out(i, 2) = 0;// z=0
    }
    return out;
}