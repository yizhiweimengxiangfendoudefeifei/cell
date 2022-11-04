#include <PanoSimApi.h>
#include <string>
#include <string_view>
#include <vector>
#include <iostream>
#include <fstream>

#include "control.h"

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
    bool turn=false;
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
        auto pGlobal = static_cast<GlobalData*>(userData->state);
        if (pGlobal != nullptr) {
            referenceLine referenceline;// referenceline class init
            
            Lidar_ObjList_G* pLidar = nullptr;
            Ego* pEgo = nullptr;
            if (pGlobal->lidar != nullptr && pGlobal->ego != nullptr) {
                pLidar = static_cast<Lidar_ObjList_G*>(pGlobal->lidar->GetHeader());
                pEgo = static_cast<Ego*>(pGlobal->ego->GetHeader());
                // reference_line calc
                referenceline.shape(pLidar);
                referenceline.calcCenterPoint();
                referenceline.sortIndex(pLidar, pEgo);
                referenceline.centerPoint();

                // interpolation algorithm
                if (referenceline.get_center_point_xy_sort().size() > 0) {
                    Eigen::MatrixXd input = vector_eigen(referenceline.get_center_point_xy_sort());
                    std::vector<std::pair<double, double>> output;
                    referenceline.average_interpolation(input, output, 0.5, 1.0);
                    referenceline.set_center_point_xy_final(output);
                    referenceline.center_point_xy = referenceline.get_center_point_xy_final();
                    referenceline.match_point_index_set.clear();
                    referenceline.sortIndex(pLidar, pEgo);
                }
            }
            // control class
            EgoControl* pEgoCtrl = nullptr;
            if (pGlobal->ego_control != nullptr) {
                pEgoCtrl = static_cast<EgoControl*>(pGlobal->ego_control->GetHeader());
                
                std::vector<std::pair<double, double>> targetPath = referenceline.get_center_point_xy_sort();// �ο�·��
                double steer = control::calculateSteering(targetPath, pEgo);
                cout << "steer: " << steer << endl;
                double thr = control::calculateThrottleBreak(targetPath, pEgo);
                
                pEgoCtrl->time = userData->time;
                pEgoCtrl->valid = 1;
                if (thr > 0) {
                    pEgoCtrl->throttle = thr;
                    pEgoCtrl->brake = 0;
                }
                else {
                    pEgoCtrl->throttle = 0;
                    pEgoCtrl->brake = -thr;
                }
                
                pEgoCtrl->steer = steer;
                pEgoCtrl->mode = 1;
                pEgoCtrl->gear = 1;
                
                
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

// vector eigen type transform
Eigen::MatrixXd vector_eigen(const std::vector<std::pair<double, double>> &input) {
    Eigen::MatrixXd out = Eigen::MatrixXd::Zero(input.size(), 3);
    for (int i = 0; i < input.size(); ++i) {
        out(i, 0) = input[i].first;// x
        out(i, 1) = input[i].second;// y
        out(i, 2) = 0;// z=0
    }
    return out;
}



//size_t mactPoint(std::vector<std::pair<double, double>> &path) {
//    std::vector<double> pts;
//    for (size_t i = 0; i < path.size(); ++i) {
//        pts.push_back(pow((double)path[i].first, 2) + pow((double)path[i].second, 2));
//    }
//    size_t index = std::min_element(pts.begin(), pts.end()) - pts.begin();
//    return index;
//}