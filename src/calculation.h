#ifndef CALCULATION_H
#define CALCULATION_H
#include "ssFrameLib.h"
#include <vector>
using namespace std;
/*
* this header file contain the device id and calculate the point coordinate
* function.
*/
const unsigned char RFANS_PRODUCT_MODEL_V6G_X16_0X32 = 0X32;
const unsigned char RFANS_PRODUCT_MODEL_V6G_X32_0X33 = 0X33;

const unsigned char RFANS_PRODUCT_MODEL_V6_X32_0X40 = 0X40;
const unsigned char RFANS_PRODUCT_MODEL_V6_X16A_0X41 = 0X41;
const unsigned char RFANS_PRODUCT_MODEL_V6_X16B_0X42 = 0X42;
const unsigned char RFANS_PRODUCT_MODEL_V6_X16Even_0X43 = 0X43;
const unsigned char RFANS_PRODUCT_MODEL_V6_X16Odd_0X44 = 0X44;

const unsigned char RFANS_PRODUCT_MODEL_V6P_X32_0X45 = 0X45;
const unsigned char RFANS_PRODUCT_MODEL_V6P_X16A_0X46 = 0X46;
const unsigned char RFANS_PRODUCT_MODEL_V6P_X16B_0X47 = 0X47;
const unsigned char RFANS_PRODUCT_MODEL_V6P_X16Even_0X48 = 0X48;
const unsigned char RFANS_PRODUCT_MODEL_V6P_X16Odd_0X49 = 0X49;

const unsigned char RFANS_PRODUCT_MODEL_V6A_X32_0X4A = 0X4A;
const unsigned char RFANS_PRODUCT_MODEL_V6A_X16A_0X4B = 0X4B;
const unsigned char RFANS_PRODUCT_MODEL_V6A_X16B_0X4C = 0X4C;
const unsigned char RFANS_PRODUCT_MODEL_V6A_X16Even_0X4D = 0X4D;
const unsigned char RFANS_PRODUCT_MODEL_V6A_X16Odd_0X4E = 0X4E;

const unsigned char RFANS_PRODUCT_MODEL_V6A_X16M_0X4F = 0X4F;
const unsigned char RFANS_PRODUCT_MODEL_V6B_X32_0X50=0X50 ;

const unsigned char RFANS_PRODUCT_MODEL_CFANS_X32_0X80=0X80 ;//????
const unsigned char RFANS_PRODUCT_MODEL_V6A_E1_0X55 = 0X55;
const unsigned char RFANS_PRODUCT_MODEL_V6A_E2_0X56 = 0X56;

const unsigned char RFANS_PRODUCT_MODEL_V6BC_16G_0X57 = 0X57;
const unsigned char RFANS_PRODUCT_MODEL_V6BC_16M_0X58 = 0X58;
const unsigned char RFANS_PRODUCT_MODEL_V6C_Z_X32_0X59 = 0X59;
const unsigned char RFANS_PRODUCT_MODEL_V6K_32M_0X5A = 0X5A;
const unsigned char RFANS_PRODUCT_MODEL_V6K_16M_0X5B = 0X5B;
const unsigned char RFANS_PRODUCT_MODEL_V6K_16G_0x5C = 0x5C;
const unsigned char CFNAS_32_0x81 = 0x81;
const int RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[] = {
    5, 7, 9, 11, 13, 15, 16, 18, 17, 19, 20, 21, 23, 25, 27, 29

};

const int RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[] = {
    6,8,10,12,14,16,18,17,19,20,22,21,24,26,28,30
};

const int RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[] = {
    1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31
};

const int RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[] = {
    0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30
};

const double HANGLE_V6B_X32_0x40[] = {
    6.01, -4.068, 3.377, -6.713,
    6.01, -4.068, 3.377, -6.713,
    6.01, -4.068, 3.377, -6.713,
    6.01, -4.068, 3.377, -6.713,
    6.01, -4.068, 3.377, -6.713,
    6.01, -4.068, 3.377, -6.713,
    6.01, -4.068, 3.377, -6.713,
    6.01, -4.068, 3.377, -6.713,
};

const double HANGLE_V6BC_16G_0x57[]={
    6.01, 3.377, 6.01, 3.377,
    6.01, 3.377, 6.01, 3.377,
    6.01, 3.377, 6.01, 3.377,
    6.01, 3.377, 6.01, 3.377,
};

const double HANGLE_V6BC_16M_0x58[]={
    2.1889,-2.8544,2.1889,-2.8544,
    2.1889,-2.8544,2.1889,-2.8544,
    2.1889,-2.8544,2.1889,-2.8544,
    2.1889,-2.8544,2.1889,-2.8544,
};


const double HANGLE_V5_X16[] = {
    -2.5, 2.5,
    -2.5, 2.5,
    -2.5, 2.5,
    -2.5, 2.5,
    -2.5, 2.5,
    -2.5, 2.5,
    -2.5, 2.5,
    -2.5, 2.5 };

const double VANGLE_V5_X16[] = {
    -15.0, -13.0,
    -11.0, -9.0,
    -7.0, -5.0,
    -4.0, -3.0,
    -2.0, -1.0,
    0,  1.0,
    3.0,  5.0,
    7.0,  9.0,  };

const double HANGLE_V6_X32_0x40[] = {
    6.35, -3.85, 3.85, -6.35,
    6.35, -3.85, 3.85, -6.35,
    6.35, -3.85, 3.85, -6.35,
    6.35, -3.85, 3.85, -6.35,
    6.35, -3.85, 3.85, -6.35,
    6.35, -3.85, 3.85, -6.35,
    6.35, -3.85, 3.85, -6.35,
    6.35, -3.85, 3.85, -6.35,

};

const double HANGLE_V6A_E1_0x55[] = {
    -4.28, -6.713, -4.28, -6.713,
    -4.28, -6.713, -4.28, -6.713,
    -4.28, -6.713, -4.28, -6.713,
    -4.28, -6.713, -4.28, -6.713,
};

const double VANGLE_V6_X32_0X40[] = {
    -20.5, -19.5, -18.5, -17.5,
    -16.5, -15.5, -14.5, -13.5,
    -12.5, -11.5, -10.5, -9.5,
    -8.5,   -7.5,  -6.5, -5.5,
    -4.5,   -3.5,  -2.5, -1.5,
    -0.5,    0.5,   1.5,  2.5,
    3.5,     4.5,   5.5,  6.5,
    7.5,     8.5,   9.5, 10.5,
};
const double  VAngle_V6C_Z_32[32] = {
    -15.5, -14.5, -13.5, -12.5
    - 11.5, -10.5, -9.5, -8.5,
    -7.5, -6.5, -5.5, -4.5,
    -3.5, -2.5, -1.5, -0.5,
    +0.5, +1.5, +2.5, +3.5,
    +4.5, +5.5, +6.5, +7.5,
    +8.5, +9.5, +10.5, +11.5,
    +12.5, +13.5, +14.5, +15.5

};
const double VAngle_V6B_16M[16] = { -15, -13, -11, -9, -7, -5, -4, -3, -2, -1, 0, 1, 3, 5, 7, 9 };
const double VAngle_V6B_16G[16]={-15,-13,-11,-9,-7,-5,-3,-1,1,3,5,7,9,11,13,15};

const double VANGLE_V6A_X32[] = {
    -20, -19, -18, -17,
    -16, -15, -14, -13,
    -12, -11, -10,  -9,
    -8,  -7,  -6,  -5,
    -4,  -3,  -2,  -1,
    0,   1,   2,   3,
    4,   5,   6,   7,
    8,   9,  10,  11,
};
static double VAngle_16E1[16] = {
    -19.5, -17.5, -15.5, -13.5,
    -11.5, -9.5, -7.5, -5.5,
    -3.5, -1.5, 0.5, 2.5,
    4.5, 6.5, 8.5, 10.5
};
static double VAngle_16E2[16] = {
    -19, -17, -15, -13,
    -11, -9, -7, -5,
    -3,    -1, 1, 3,
    5, 7, 9, 11
};

/*const double HANGLE_V6G_X32_0X33[] = {
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
};*/

const double HANGLE_V6G_X32_0X33[] = {
    5.21, -3.48, 3.04, -5.62,
    5.21, -3.48, 3.04, -5.62,
    5.21, -3.48, 3.04, -5.62,
    5.21, -3.48, 3.04, -5.62,
    5.21, -3.48, 3.04, -5.62,
    5.21, -3.48, 3.04, -5.62,
    5.21, -3.48, 3.04, -5.62,
    5.21, -3.48, 3.04, -5.62,
    5.21, -3.48, 3.04, -5.62,
};

const double VANGLE_V6G_X32_0X33[] = {
    -25,    -22,   -19, -16,
    -13,    -11,    -9,  -7,
    -5.5,   -4.5,  -3.5, -2.9,
    -2.45,   -2.1, -1.75, -1.4,
    -1.05,   -0.7, -0.35,    0,
    0.35,    0.7,  1.05,  1.4,
    2.5,    3.5,   4.5,    6,
    8,     10,    12,   15,
};


//const double HANGLE_V6K_32M[] = {
//    6.35, -3.7, 3.3, -5.8,
//    6.35, -3.7, 3.3, -5.8,
//    6.35, -3.7, 3.3, -5.8,
//    6.35, -3.7, 3.3, -5.8,
//    6.35, -3.7, 3.3, -5.8,
//    6.35, -3.7, 3.3, -5.8,
//    6.35, -3.7, 3.3, -5.8,
//    6.35, -3.7, 3.3, -5.8
//};

//const double VANGLE_V6K_32M[32] = {
//    -16.9, -15, -13.68, -12,
//    -10.5, -9.5, -8.5, -7.5,
//    -6.5, -6, -5.5, -5,
//    -4.5, -4, -3.5, -3,
//    -2.5, -2, -1.5, -1,
//    -0.5, 0, 0.5, 1,
//    2, 3, 4, 5,
//    6.37, 8, 9.26, 11
//};

//const double HANGLE_V6K_16M[] = {
//    1.325, -1.325, 1.325, -1.325,
//    1.325, -1.325, 1.325, -1.325,
//    1.325, -1.325, 1.325, -1.325,
//    1.325, -1.325, 1.325, -1.325
//};

//const double VAngle_V6K_16M[16] = {
//    -15, -12, -9.5, -7.5,
//    -6, -5, -4, -3,
//    -2, -1, 0, 1,
//    3, 5, 8, 11
//};
//V6 0x40-0x44 v6A 0x4A-0x4E
const double DELTA_T_V6_X32_0x40[] ={
    0,      3.125,     1.5625,    4.6875,
    6.25, 9.375,   7.8125, 10.9375,
    12.5,  15.625,    14.0625,  17.1875,
    18.75, 21.875,  20.3125, 23.4375,
    25,     28.125,    26.5625,    29.6875,
    31.25, 34.375,   32.8125,  35.9375,
    37.5, 40.625,   39.0625,  42.1875,
    43.75,46.875,  45.3125, 48.4375,
};

//V6G_0x33 V6B/C 0x50 0x5A
const double DELTA_T_V6G_V6BC_X32_0x33_0x50[] ={
    0,      6.25,     12.5,    18.75,
    1.5625, 7.8125,   14.0625, 20.3125,
    3.125,  9.375,    15.625,  21.875,
    4.6875, 10.9375,  17.1875, 23.4375,
    25,     31.25,    37.5,    43.75,
    26.562, 32.812,   39.062,  45.312,
    28.125, 34.375,   40.625,  46.875,
    29.6875,35.9375,  42.1875, 48.4375,
};

//V6A-16M, V6A-16E1 0x4F 0x55 0x56
const double DELTA_T_V6A_X16M_0X4F_0x55_0x56[] ={
    0,     3.125,
    6.25,  9.375,
    12.5,  15.625,
    18.75, 21.875,
    25,    28.125,
    31.25, 34.375,
    37.5,  40.625,
    43.75, 46.875,
};

//V6B/C-16G, V6B/C-16M,0x57,0x58
const double DELTA_T_V6BC_16GM_0x57_0x58[] ={
    0,     12.5,
    3.125, 15.625,
    6.25,  18.75,
    9.375, 21.875,
    25,    37.5,
    28.125,40.625,
    31.25, 43.75,
    34.375,46.875
};

//CFans
const double DELTA_T_CFANS_0x80[] = {
    4.6875, 10.9375, 17.1875,23.4375,
    3.125,  9.375,   15.625, 21.875,
    1.5625, 7.8125,  14.0625,20.3125,
      0,     6.25,    12.5,   18.75,
    29.6875, 35.9375, 42.1875, 48.4375,
    28.125,  34.375,  40.625,  46.875,
    26.5625, 32.8125, 39.0625, 45.3125,
       25,    31.25,    37.5,   43.75
};

//V6K-16G
const double DELTA_T_RFANS_V6K_16G_0x5C[]={
    0,   13.32,
    3.33,16.65,
    6.66, 19.98,
    9.99, 23.31,
    26.64, 39.96,
    29.97, 43.29,
    33.3, 46.62,
    36.63, 49.95
};



// V6K-32M
const double HANGLE_V6K_32M_0X5A[32]={
    3.7, -6.35, 6.35, -3.7,
    3.7, -6.35, 6.35, -3.7,
    3.7, -6.35, 6.35, -3.7,
    3.7, -6.35, 6.35, -3.7,
    3.7, -6.35, 6.35, -3.7,
    3.7, -6.35, 6.35, -3.7,
    3.7, -6.35, 6.35, -3.7,
    3.7, -6.35, 6.35, -3.7
};

const double VANGLE_V6K_32M_0x5A[32]={
    -16.9, -15, -13.68, -12,
    -10.5, -9.5, -8.5, -7.5,
    -6.5, -6, -5.5, -5,
    -4.5, -4, -3.5, -3,
    -2.5, -2, -1.5, -1,
    -0.5, 0, 0.5, 1,
    2, 3, 4, 5,
    6.37, 8, 9.26, 11
};


const double DELTA_T_V6K_32M_0x5A[32]={
    0,   6.25,    12.5,   18.75,
    1.5625, 7.8125, 14.0625,20.3125,
    3.125,  9.375,  15.625,  21.875,
    4.6875, 10.9375,  17.1875,  23.4375,
    25, 31.25, 37.5, 43.75,
    26.5625, 32.8125, 39.0625, 45.3125,
    28.125, 34.375, 40.625, 46.875,
    29.6875, 35.9375, 42.1875, 48.4375

};




//V6K-16M

const double HANGLE_V6K_16M_0x5B[] = {
    1.325, -1.325, 1.325, -1.325,
    1.325, -1.325, 1.325, -1.325,
    1.325, -1.325, 1.325, -1.325,
    1.325, -1.325, 1.325, -1.325
};

const double VANGLE_V6K_16M_0x5B[]={
    -12, -15, -7.5, -9.5,
    -5, -6,-3, -4,
    -1, -2,1, 0,
    5, 3,11, 8
};

const double DELTA_T_V6K_16M_0x5B[]={
    13.32,0,16.65,3.33,
    19.98,6.66,23.31,9.99,
    39.96,26.64,43.29,29.97,
    46.62,33.3,49.95,36.63
};

extern float m_anglePara[30];
extern double m_mirrorVector[4][3];
extern double m_lidarAngle[2][32];
extern std::vector<float> s_reviseangles;
extern float cfans_lidarAngle[2][8];
extern float mirrorVector[4][3];
extern float m_anglePara_32[30];
extern std::vector<float> s_reviseangles_32;
int calcXyz(unsigned char flag,float &mtRange, float &mtAngle, RFANS_XYZ_S &outXyz);
int calcCFansCoor(float range ,float angle ,int index,int laserID, RFANS_XYZ_S &outXyz);
int calcCFansXYZ_32(float range, float angle,int index, int laserID, RFANS_XYZ_S &outXyz);
int initCFansPara(std::string reviseAngle);
int initCFans_32(std::string revisePara);
class Calculation{
public:
    explicit Calculation();
    ~Calculation();
private:
};
#endif // CALCULATION_H
