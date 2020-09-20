#ifndef DATALEVLELPROCESS_H
#define DATALEVLELPROCESS_H
#include <string>
#include <math.h>
#include <iostream>
#include <map>
#include <stdlib.h>
#include <rfans_driver/point_types.h>
using namespace std;
class dataLevelProcess{
public:
    dataLevelProcess();
    ~dataLevelProcess(){}
    bool read_ini_file(std::string file_name);
    int setTemperature(float temperature);
    bool set_intensity_params();

   //多级数据的处理（默认）
    bool data_level_convert(RFANS_XYZ_S* mtPoint);

    //----------------------0级算法（过滤）-------------------------
    int filterTimeWindow(RFANS_XYZ_S* mtPoint);         // 时间窗口过滤
    int filterRangeIntensityWide(RFANS_XYZ_S* mtPoint); // 距离灰度脉宽过滤
    int filterAngle(RFANS_XYZ_S* mtPoint);              // 角度过滤
    int filterMultiEchoWide(RFANS_XYZ_S* mtPoint);      // 修正多回波对脉宽的影响



   //----------------------1级算法（标定）-------------------------
    //修正距离
    int reviseRangeIntensity(RFANS_XYZ_S* mtPoint);  //灰度改正表修正
    int revisePlusMultiCoef(RFANS_XYZ_S* mtPoint);  //加乘系数修正
    int reviseRangeConst(RFANS_XYZ_S* mtPoint);  //加常数修正
    int reviseMidFarCalib(RFANS_XYZ_S* mtPoint);
    int reviseRangeTemprature(RFANS_XYZ_S* mtPoint);  //修正温度对距离的影响

    //修正脉宽
    int reviseWideRange(RFANS_XYZ_S *mtPoint);   //修正距离对脉宽的影响
    int reviseWideLaser(RFANS_XYZ_S *mtPoint);    //修正激光器间的差异

    //转换为12bit灰度
    int trans12bitIntensity(RFANS_XYZ_S *mtPoint);

    //零位角修正
    int CalbZeroAngle(RFANS_XYZ_S *mtPoint);//RFans
    int reviseCFansAngle_128(RFANS_XYZ_S *mtPoint);//CFans128
    int reviseCFansAngle_32(RFANS_XYZ_S *mtPoint);//CFans32

    //安置角度标定
    int reviseCFansAnlignAnle_128(RFANS_XYZ_S *mtPoint);//CFans128
    int reviseCFansAnlignAnle_32(RFANS_XYZ_S *mtPoint);//CFans32

    //----------------------2级算法（转8bit灰度）-------------------------
   //12bit灰度转换为8bit灰度
    unsigned short trans8bitIntensity(RFANS_XYZ_S *mtPoint);

private:
    std::map<int, float> revise_map_;
    float temperature_;
    bool is_load_;
    int intensity_duration_[6];
    float params_[10];
};




#endif // DATALEVLELPROCESS_H
