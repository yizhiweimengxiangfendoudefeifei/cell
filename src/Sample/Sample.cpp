#include <PanoSimApi.h>
#include <string>
#include <string_view>
#include <vector>
#include <iostream>
#include <fstream>

#include "control.h"
#include "lqr_control.h"
#include "pure_puresuit_control.h"

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
using PanoSimBasicsBus::EGO_EXTRA_FORMAT;
using PanoSimBasicsBus::EgoExtra;


struct GlobalData {
    BusAccessor* lidar;
    BusAccessor* ego_control, *ego, *ego_extra;
    int times = 0;
    bool flg = false;
    std::shared_ptr<control> control_base;
    
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
    pGlobal->ego_extra = new BusAccessor(userData->busId, "ego_extral", EGO_EXTRA_FORMAT);
    userData->state = pGlobal;

    // control mode 0:lqr  1:pure_pursuit
    int control_mode = 0;
    switch (control_mode)
    {
    case 0:
        cout << "lqr init!!!";
        pGlobal->control_base = std::make_shared<lqrControl>(0.1, 0.06, 0.0);
        break;
    case 1:
        cout << "pure_puresuit init!!!";
        pGlobal->control_base = std::make_shared<purePursuit>(0.1, 0.06, 0.0);
        break;
    default:
        break;
    }
}

void ModelOutput(UserData* userData) {
    

    if (userData != nullptr) {
        auto pGlobal = static_cast<GlobalData*>(userData->state);
        if (pGlobal != nullptr) {
            referenceLine referenceline;// referenceline class init
            
            Lidar_ObjList_G* pLidar = nullptr;
            Ego* pEgo = nullptr;
            EgoExtra* pEgoExtra = nullptr;
            if (pGlobal->lidar != nullptr && pGlobal->ego != nullptr) {
                pLidar = static_cast<Lidar_ObjList_G*>(pGlobal->lidar->GetHeader());
                pEgo = static_cast<Ego*>(pGlobal->ego->GetHeader());
                pEgoExtra = static_cast<EgoExtra*>(pGlobal->ego->GetHeader());
                // reference_line calc
                referenceline.shape(pLidar);
                referenceline.calcCenterPoint();
                referenceline.sortIndex();
                referenceline.centerPoint();

                // interpolation algorithm
                if (referenceline.get_center_point_xy_sort().size() > 0) {
                    Eigen::MatrixXd input = vector_eigen(referenceline.get_center_point_xy_sort());
                    std::vector<std::pair<double, double>> output;
                    referenceline.average_interpolation(input, output, 0.2, 0.6);
                    referenceline.set_center_point_xy_final(output);
                    //std::cout << "+++++++++++++++++++++" << std::endl;
                    /*for (auto line : output) {
                        cout << line.first << "   " << line.second << endl;
                    }*/
                }
                // calc kappa theta
                referenceline.calc_k_theta();// struct RefPoint
            }
            // control class
            EgoControl* pEgoCtrl = nullptr;
            if (pGlobal->ego_control != nullptr) {
                pEgoCtrl = static_cast<EgoControl*>(pGlobal->ego_control->GetHeader());

                double steer = 0;
                std::vector<RefPoint> targetPathPoint = referenceline.point;
                

                steer = pGlobal->control_base->calculateCmd(targetPathPoint, pLidar, pEgo);
                int forwardIndex = pGlobal->control_base->calc_forwardIndex(targetPathPoint, pEgo);
                cout << "sample steer: " << steer << endl;
                double thr = pGlobal->control_base->calculateThrottleBreak(targetPathPoint, pEgo, forwardIndex);
                auto yellodist = referenceline.calculate_yellowdist(referenceline.get_yellow_point_xy_final());

                pEgoCtrl->time = userData->time;
                pEgoCtrl->valid = 1;
                 if (pGlobal->times <4 ) {
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
                     if (yellodist.second < 1 && yellodist.first > 0 && !pGlobal -> flg){
                         pGlobal -> flg = true;
                     }
                     else if (yellodist.second < 1 && yellodist.first < 0 && pGlobal->flg) {
                         pGlobal->flg = false;
                         pGlobal->times++;
                     }
                 }
                 else {
                     pEgoCtrl->throttle = 0;
                     pEgoCtrl->brake = 1;
                 }
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