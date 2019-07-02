/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#ifndef _FRAME_LIB_H_
#define _FRAME_LIB_H_
#include <iostream>
#include <string>
#include "rfans_driver/RfansPacket.h"

using namespace std;

const static int REG_DEVICE_CTRL = (0x40);
const static int REG_DATA_TRANS = (0x70);
const static int REG_DATA_LEVEL = 0x08;

const static int CMD_SCAN_SPEED_5HZ = 0;
const static int CMD_SCAN_SPEED_10HZ = 0x50;
const static int CMD_SCAN_SPEED_20HZ = 0xF0;
const static int CMD_SCAN_ENABLE = 0x1;
const static int CMD_LASER_ENABLE = 0x2;
const static int CMD_CALC_DATA = 0x2;
const static int CMD_DEBUG_DATA = 0x5;
const static int CMD_RCV_CLOSE = 0x0;
const static int CMD_LEVEL0_ECHO = 0x00000000 ;
const static int CMD_LEVLE0_DUAL_ECHO = 0x00000002;
const static int CMD_LEVEL1_ECHO = 0x01000200;
const static int CMD_LEVEL1_DUAL_ECHO = 0x01000202;
const static int CMD_LEVEL2_ECHO = 0x0201BF00;
const static int CMD_LEVEL2_DUAL_ECHO = 0x0201BF02;
const static int CMD_LEVEL3_ECHO = 0x03000100;
const static int CMD_LEVEL3_DUAL_ECHO = 0x03000102;



const static int ANGLE_SPEED_5HZ = 5;
const static int ANGLE_SPEED_10HZ = 10;
const static int ANGLE_SPEED_20HZ = 20;

static unsigned short DEVICE_PORT_NUMBER = 2014;
static unsigned short PC_PORT_NUMBER = 2014;
static std::string DEVICE_IP_STRING = "192.168.0.3";

static unsigned short UDP_FRAME_MIN =(18);

#define DEB_FRAME_WRITE (0xA5)  //write head sync
#define DEB_FRAME_READ  (0x5a)  //read head sync
#define DEB_FRAME_ERROR  (0xE7) //err  haad sync

#define FRAME_MSG_LENGTH (1024) 

const int UDPREG_MAX_COUNT = 256;
const int ROMREG_MAX_COUNT = 0x7FF;

//const unsigned char RFANS_PRODUCT_MODEL_V6G_X16_0X32 = 0X32;
//const unsigned char RFANS_PRODUCT_MODEL_V6G_X32_0X33 = 0X33;

//const unsigned char RFANS_PRODUCT_MODEL_V6_X32_0X40 = 0X40;
//const unsigned char RFANS_PRODUCT_MODEL_V6_X16A_0X41 = 0X41;
//const unsigned char RFANS_PRODUCT_MODEL_V6_X16B_0X42 = 0X42;
//const unsigned char RFANS_PRODUCT_MODEL_V6_X16Even_0X43 = 0X43;
//const unsigned char RFANS_PRODUCT_MODEL_V6_X16Odd_0X44 = 0X44;

//const unsigned char RFANS_PRODUCT_MODEL_V6P_X32_0X45 = 0X45;
//const unsigned char RFANS_PRODUCT_MODEL_V6P_X16A_0X46 = 0X46;
//const unsigned char RFANS_PRODUCT_MODEL_V6P_X16B_0X47 = 0X47;
//const unsigned char RFANS_PRODUCT_MODEL_V6P_X16Even_0X48 = 0X48;
//const unsigned char RFANS_PRODUCT_MODEL_V6P_X16Odd_0X49 = 0X49;

//const unsigned char RFANS_PRODUCT_MODEL_V6A_X32_0X4A = 0X4A;
//const unsigned char RFANS_PRODUCT_MODEL_V6A_X16A_0X4B = 0X4B;
//const unsigned char RFANS_PRODUCT_MODEL_V6A_X16B_0X4C = 0X4C;
//const unsigned char RFANS_PRODUCT_MODEL_V6A_X16Even_0X4D = 0X4D;
//const unsigned char RFANS_PRODUCT_MODEL_V6A_X16Odd_0X4E = 0X4E;

//const unsigned char RFANS_PRODUCT_MODEL_V6A_X16M_0X4F = 0X4F;
//const unsigned char RFANS_PRODUCT_MODEL_V6B_X32_0X50=0X50 ;

//const unsigned char RFANS_PRODUCT_MODEL_CFANS_X32_0X3780=0X3780 ;
//const unsigned char RFANS_PRODUCT_MODEL_V6A_E1_0X55 = 0X55;
//const unsigned char RFANS_PRODUCT_MODEL_V6A_E2_0X56 = 0X56;

//const unsigned char RFANS_PRODUCT_MODEL_V6BC_16G_0X57 = 0X57;
//const unsigned char RFANS_PRODUCT_MODEL_V6BC_16M_0X58 = 0X58;
//const unsigned char RFANS_PRODUCT_MODEL_V6C_Z_X32_0X59 = 0X59;

//const int RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[] = {
//  5, 7, 9, 11, 13, 15, 16, 18, 17, 19, 20, 21, 23, 25, 27, 29

//};

//const int RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[] = {
//  6,8,10,12,14,16,18,17,19,20,22,21,24,26,28,30
//};

//const int RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[] = {
//  1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31
//};

//const int RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[] = {
//  0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30
//};

//////物理编号角度顺序
//const double HANGLE_V6B_X32_0x40[] = {
//    6.01, -4.068, 3.377, -6.713,
//    6.01, -4.068, 3.377, -6.713,
//    6.01, -4.068, 3.377, -6.713,
//    6.01, -4.068, 3.377, -6.713,
//    6.01, -4.068, 3.377, -6.713,
//    6.01, -4.068, 3.377, -6.713,
//    6.01, -4.068, 3.377, -6.713,
//    6.01, -4.068, 3.377, -6.713,
//};

//const double HANGLE_V6BC_16G_0x57[]={
//	6.01, 3.377, 6.01, 3.377,
//	6.01, 3.377, 6.01, 3.377,
//	6.01, 3.377, 6.01, 3.377,
//	6.01, 3.377, 6.01, 3.377,
//};

//const double HANGLE_V6BC_16M_0x58[]={
//     2.1889,-2.8544,2.1889,-2.8544,
//     2.1889,-2.8544,2.1889,-2.8544,
//     2.1889,-2.8544,2.1889,-2.8544,
//     2.1889,-2.8544,2.1889,-2.8544,
//};


//const double HANGLE_V5_X16[] = {
//  -2.5, 2.5,
//  -2.5, 2.5,
//  -2.5, 2.5,
//  -2.5, 2.5,
//  -2.5, 2.5,
//  -2.5, 2.5,
//  -2.5, 2.5,
//  -2.5, 2.5 };

//const double VANGLE_V5_X16[] = {
//  -15.0, -13.0,
//  -11.0, -9.0,
//  -7.0, -5.0,
//  -4.0, -3.0,
//  -2.0, -1.0,
//     0,  1.0,
//   3.0,  5.0,
//   7.0,  9.0,  };

////物理编号角度顺序
//const double HANGLE_V6_X32_0x40[] = {
//  6.35, -3.85, 3.85, -6.35,
//  6.35, -3.85, 3.85, -6.35,
//  6.35, -3.85, 3.85, -6.35,
//  6.35, -3.85, 3.85, -6.35,
//  6.35, -3.85, 3.85, -6.35,
//  6.35, -3.85, 3.85, -6.35,
//  6.35, -3.85, 3.85, -6.35,
//  6.35, -3.85, 3.85, -6.35,

//};

//const double HANGLE_V6A_E1_0x55[] = {
//    -4.068, -6.713, -4.068, -6.713,
//    -4.068, -6.713, -4.068, -6.713,
//    -4.068, -6.713, -4.068, -6.713,
//    -4.068, -6.713, -4.068, -6.713,
//};

//const double VANGLE_V6_X32_0X40[] = {
//  -20.5, -19.5, -18.5, -17.5,
//  -16.5, -15.5, -14.5, -13.5,
//  -12.5, -11.5, -10.5, -9.5,
//  -8.5,   -7.5,  -6.5, -5.5,
//  -4.5,   -3.5,  -2.5, -1.5,
//  -0.5,    0.5,   1.5,  2.5,
//  3.5,     4.5,   5.5,  6.5,
//  7.5,     8.5,   9.5, 10.5,
//};
//const double  VAngle_V6C_Z_32[32] = {
//    -15.5, -14.5, -13.5, -12.5
//    - 11.5, -10.5, -9.5, -8.5,
//    -7.5, -6.5, -5.5, -4.5,
//    -3.5, -2.5, -1.5, -0.5,
//    +0.5, +1.5, +2.5, +3.5,
//    +4.5, +5.5, +6.5, +7.5,
//    +8.5, +9.5, +10.5, +11.5,
//    +12.5, +13.5, +14.5, +15.5

//};
//const double VAngle_V6B_16M[16] = { -15, -13, -11, -9, -7, -5, -4, -3, -2, -1, 0, 1, 3, 5, 7, 9 };
//const double VAngle_V6B_16G[16]={-15,-13,-11,-9,-7,-5,-3,-1,1,3,5,7,9,11,13,15};

//const double VANGLE_V6A_X32[] = {
//  -20, -19, -18, -17,
//  -16, -15, -14, -13,
//  -12, -11, -10,  -9,
//   -8,  -7,  -6,  -5,
//   -4,  -3,  -2,  -1,
//    0,   1,   2,   3,
//    4,   5,   6,   7,
//    8,   9,  10,  11,
//};
//static double VAngle_16E1[16] = {
//    -19.5, -17.5, -15.5, -13.5,
//    -11.5, -9.5, -7.5, -5.5,
//    -3.5, -1.5, 0.5, 2.5,
//    4.5, 6.5, 8.5, 10.5
//};
//static double VAngle_16E2[16] = {
//    -19, -17, -15, -13,
//    -11, -9, -7, -5,
//    -3,    -1, 1, 3,
//    5, 7, 9, 11
//};

///*const double HANGLE_V6G_X32_0X33[] = {
//  5.52, -3.48, 3.48, -5.52,
//  5.52, -3.48, 3.48, -5.52,
//  5.52, -3.48, 3.48, -5.52,
//  5.52, -3.48, 3.48, -5.52,
//  5.52, -3.48, 3.48, -5.52,
//  5.52, -3.48, 3.48, -5.52,
//  5.52, -3.48, 3.48, -5.52,
//  5.52, -3.48, 3.48, -5.52,
//  5.52, -3.48, 3.48, -5.52,
//};*/

//const double HANGLE_V6G_X32_0X33[] = {
//    5.21, -3.48, 3.04, -5.62,
//    5.21, -3.48, 3.04, -5.62,
//    5.21, -3.48, 3.04, -5.62,
//    5.21, -3.48, 3.04, -5.62,
//    5.21, -3.48, 3.04, -5.62,
//    5.21, -3.48, 3.04, -5.62,
//    5.21, -3.48, 3.04, -5.62,
//    5.21, -3.48, 3.04, -5.62,
//    5.21, -3.48, 3.04, -5.62,
//};

//const double VANGLE_V6G_X32_0X33[] = {
//  -25,    -22,   -19, -16,
//  -13,    -11,    -9,  -7,
// -5.5,   -4.5,  -3.5, -2.9,
//-2.45,   -2.1, -1.75, -1.4,
//-1.05,   -0.7, -0.35,    0,
// 0.35,    0.7,  1.05,  1.4,
//  2.5,    3.5,   4.5,    6,
//    8,     10,    12,   15,
// };

////V6 0x40-0x44 v6A 0x4A-0x4E
//const double DELTA_T_V6_X32_0x40[] ={
//    0,      3.125,     1.5625,    4.6875,
//    6.25, 9.375,   7.8125, 10.9375,
//    12.5,  15.625,    14.0625,  17.1875,
//    18.75, 21.875,  20.3125, 23.4375,
//    25,     28.125,    26.5625,    29.6875,
//    31.25, 34.375,   32.8125,  35.9375,
//    37.5, 40.625,   39.0625,  42.1875,
//    43.75,46.875,  45.3125, 48.4375,
//};

////V6G_0x33 V6B/C 0x50
//const double DELTA_T_V6G_V6BC_X32_0x33_0x50[] ={
//    0,      6.25,     12.5,    18.75,
//    1.5625, 7.8125,   14.0625, 20.3125,
//    3.125,  9.375,    15.625,  21.875,
//    4.6875, 10.9375,  17.1875, 23.4375,
//    25,     31.25,    37.5,    43.75,
//    26.562, 32.812,   39.062,  45.312,
//    28.125, 34.375,   40.625,  46.875,
//    29.6875,35.9375,  42.1875, 48.4375,
//};

////V6A-16M, V6A-16E1 0x4F 0x55 0x56
//const double DELTA_T_V6A_X16M_0X4F_0x55_0x56[] ={
//    0,     3.125,
//    6.25,  9.375,
//    12.5,  15.625,
//    18.75, 21.875,
//    25,    28.125,
//    31.25, 34.375,
//    37.5,  40.625,
//    43.75, 46.875,
//};

////V6B/C-16G, V6B/C-16M,0x57,0x58
//const double DELTA_T_V6BC_16GM_0x57_0x58[] ={
//    0,     12.5,
//    3.125, 15.625,
//    6.25,  18.75,
//    9.375, 21.875,
//    25,    37.5,
//    28.125,40.625,
//    31.25, 43.75,
//    34.375,46.875
//};

typedef enum{
  eCmdWrite,  //write command
  eCmdRead,   //read command
  eCmdQuery,  //Query command
} SCD_FRAME_TYPE_E ;

typedef struct _frams_buffer{
  char msgStream[FRAME_MSG_LENGTH];
  int writeIdx;
  int readIdx;
  int length;
} FRAMS_BUFFER_S;

#pragma pack(1)
typedef struct {                 //! rfans udp command package
  unsigned char msgHead;         //!< head sync
  unsigned char msgCheckSum;     //!< check sum
  unsigned short regAddress;     //!< register add
  unsigned int regData;          //!< register data
} DEB_FRAME_S;

typedef enum {
  eDevCmdIdle = 0,
  eDevCmdWork,
  eDevCmdSimu,
  eDevCmdBreak,
  eDevCmdReset,
  eDevCmdAsk,
} DEB_CMD_E;

typedef enum {
  eFormatCalcData = 0x2,
  eFormatDebugData = 0x5,
}DEB_DFORMAT_E;


static int DEVICE_MOTOR_HZ = 5;
typedef struct {
  DEB_CMD_E cmdstat;
  DEB_DFORMAT_E dataFormat;
  int scnSpeed;
  int dataLevel;
  int lsrFreq;
  float rangeMin, rangeMax;
}DEB_PROGRM_S;

static const size_t upd_packet_size = 0x8000;  //32KB
const unsigned char ID_RFANSBLOCKV2_SYNC = 0x96;
const unsigned char ID_RFANSBLOCKV32_0_15_SYNC = 0x97;
const unsigned char ID_RFANSBLOCKV32_16_31_SYNC = 0x98;

const unsigned char ID_RFANSBLOCKV6G_0_15_SYNC = 0x99;
const unsigned char ID_RFANSBLOCKV6G_16_31_SYNC = 0x9A;

const unsigned char ID_RFANSBLOCKV32_GM_0_15_SYNC = 0x87;
const unsigned char ID_RFANSBLOCKV32_GM_16_31_SYNC = 0x88;

const unsigned char ID_RFANSBLOCK_GM_SYNC = 0xee;


const int UDP_PACKET_SIZE_V5A = 1380;
const int UDP_PACKET_SIZE_V6G = 1206;
const int UDP_PACKET_SIZE_DATA_LEVEL_ORI=1406;

typedef struct {
  unsigned short angle  : 16 ;          //scan angle, 0.01°
  unsigned short rangeOne  : 16 ;       //echo 1,  cm
  unsigned short rangeTwo  : 16 ;       //echo 2,  cm
  unsigned char  intentOne : 8 ;        //0~255
  unsigned char  intentTwo : 8 ;        //0~255
}SCDRFANS_POINT_S;

const int RFANS_LASER_COUNT = 16 ;
typedef struct {                                        //138 byte
  unsigned char           dataID : 8;                   //Sync Number
  unsigned char           chksum : 8;                   //chksum  Bytes[3,138]
  unsigned int            t0stampH  : 32;               //T0 Bloceek
  unsigned int            t0stampL  : 32;               //T0 tag
  SCDRFANS_POINT_S        laserData[RFANS_LASER_COUNT] ;//
}SCDRFANS_BLOCK_S;

typedef struct {
  float x,y,z ;
  float intent;
  int laserid;
  float timeflag;
  float hangle;
  unsigned char mirrorid;
  //float angle, range ;
}RFANS_XYZ_S;

static const int DECODE_BUFFER_SIZE = 0x40000; // 256KB

static const int UDP_PACKET_SIZE = 2*1024; // 256KB
static const int UDP_PACKET_BUFFER_SIZE = 16*1024; // 256KB
typedef struct  {
  int packetsize ;
  unsigned char buffer[UDP_PACKET_SIZE];
}UDP_PACKET_S;

typedef struct {
  int wrHead, rdTail, bufSize;
  UDP_PACKET_S buffer[UDP_PACKET_BUFFER_SIZE];
} UDP_PACKET_BUFFER_S;


typedef struct {
  int wrHead, rdTail, bufSize;
  unsigned char buffer[DECODE_BUFFER_SIZE];
} UDP_DECBUFFER_S;

typedef struct
{
  unsigned short range;
  unsigned char intensity;
}RFans_Laser32Block_S;

const unsigned short RFANS_UDPFRAMV6G_FLAT = 0xFFEE;
const unsigned short LASER32BLOCK_COUNT = 32;
typedef struct
{
  unsigned short flag; //oxFFEE
  unsigned short azimuthAngle;
  RFans_Laser32Block_S laserBlock[32];
}RFans_DataBlock_S;
const unsigned short UDP32FRAMEV6G_COUNT = 12;
const unsigned short RFANS_GM_16_FLAG = 0x3732;
//const unsigned short RFANS_GM_32_FLAG = 0x373C;
const unsigned short RFANS_V6_GM_33_FLAG = 0x3733;


typedef struct
{
  SCDRFANS_BLOCK_S blockdata[10];
}RFans_UDPFRAMEV5_S;

typedef struct
{
    //42 bype header + 12*100byte group + 4byte GPS Timestamp + 2byte Reserved
  RFans_DataBlock_S dataBlock[UDP32FRAMEV6G_COUNT];//each 100byte & total 12 group
  unsigned int gpsTimestamp;
  unsigned char gmReservedA;
  unsigned char gmReservedB;
}RFans_UDP32FRAMEV6G_S;

//data level enum set

typedef enum{
    LEVEL0_ECHO=0,
    LEVEL0_DUAL_ECHO,
    LEVEL1_ECHO,
    LEVEL1_DUAL_ECHO,
    LEVEL2_ECHO,
    LEVEL2_DUAL_ECHO,
    LEVEL3_ECHO,
    LEVEL3_DUAL_ECHO,

}MULTI_LEVEL_DATA;

// add different level data definition

// data level enum type.
typedef enum {
    DATA_LEVEL_ORI = 0,     //0级：原始数据
    DATA_LEVEL_CALIB,       //1级：标定数据
    DATA_LEVEL_USER,        //2级：用户级别数据，12bit灰度
    DATA_LEVEL_USER_SIMPLE, //3级：用户级别精简数据
} DATA_LEVEL_E;

// algorithm flag
typedef enum {
    ALGORITHM_ID_PREPROCESS = 0x00, //预处理模块
    ALGORITHM_ID_DISTANCE = 0x10,   //距离标定
    ALGORITHM_ID_INTENSITY = 0x10,  //灰度标定
    ALGORITHM_ID_ANGLE = 0x20,      //角度标定
    ALGORITHM_ID_STRETCHing = 0x30, //灰度拉伸
} ALGORITHM_ID_E;


// mirror id

typedef enum {
    MIRROR_ID_CFANS_00 = 0x0,   //CFANS ...
    MIRROR_ID_CFANS_01 = 0x1,
    MIRROR_ID_CFANS_02 = 0x2,
    MIRROR_ID_RFANS = 0x3,      //RFANS镜面标识
} MIRROR_ID_E;


// data packet id
typedef enum {
    PACK_ID_DUALECHO_MIX    = 0x00,   //双回波混合输出
    PACK_ID_DUALECHO        = 0x01,   //双回波输出
    PACK_ID_STRONG_ECHO     = 0x02,   //最强回波输出
    PACK_ID_FIRST_ECHO      = 0x03,   //第一回波输出

//    PACK_ID_DUAL_ECHO_MIX       = 0x01,
//    PACK_ID_TRIPLE_ECHO         = 0x0A,
//    PACK_ID_DUAL_ECHO           = 0x0C,
//    PACK_ID_SIGNLE_STRONG_ECHO  = 0x0D,
//    PACK_ID_SIGNLE_FIRST_ECHO   = 0x0E,
} PACK_ID_E;

static const int GROUP_NUM_ORI = 6;
static const int GROUP_NUM_CALIB = 6;
static const int GROUP_NUM_USER = 10;
static const int GROUP_NUM_USER_SIMPLE = 12;
static const int POINT_NUM_ORI = 32;
static const int POINT_NUM_CALIB = 32;
static const int POINT_NUM_USER = 16;
static const int POINT_NUM_USER_SIMPLE = 32;

// 0级数据封装：
typedef struct {
    unsigned short range;
    unsigned short rising_edge;
    uint8_t intensity_pulse[3]; //{Intensity[23:12]，Pulse Widt[11:0]}
} POINT_ORI_S;

typedef struct {
    // flag 定义
    //	2 bit 数据分级
    //	10bit 算法模块编号
    //	2 bit 镜面标识
    //	2 bit 数据打包格式编号
    unsigned short flag;
    unsigned short azimuth_angle;
    POINT_ORI_S points[POINT_NUM_ORI];
} GROUP_ORI_S;

typedef struct {
    GROUP_ORI_S groups[GROUP_NUM_ORI];
    uint8_t reserved[32];
    uint32_t gps_timestamp;
    unsigned char gmReservedA;
    unsigned char gmReservedB;
} PACKET_ORI_S;//1406 byte

//1级数据封装同0级数据
//2级数据封装：12bit intensity
typedef struct {
    uint16_t range1;
    uint16_t range2;
    uint8_t intents[3]; //{intensity1[23:12]，intensity2[11:0]}
} POINT_USER_S;           // contains 2 point

typedef struct {
    uint16_t flag;
    uint16_t azimuth_angle;
    POINT_USER_S points[POINT_NUM_USER];//16 points
} GROUP_USER_S;

typedef struct {
    GROUP_USER_S groups[GROUP_NUM_USER];
    uint8_t reserved[40];
    uint32_t gps_timestamp;
    unsigned char gmReservedA;
    unsigned char gmReservedB;
} PACKET_USER_S;//1206byte

//3级数据封装：
typedef struct {
    uint16_t range;
    uint8_t intensity;
} POINT_USER_SIMPLE_S;

typedef struct {
    uint16_t flag;
    uint16_t azimuth_angle;
    POINT_USER_SIMPLE_S points[POINT_NUM_USER_SIMPLE];
} GROUP_USER_SIMPLE_S;

typedef struct {
    GROUP_USER_SIMPLE_S groups[GROUP_NUM_USER_SIMPLE];
    uint32_t gps_timestamp;
    //uint16_t factory;
    unsigned char gmReservedA;
    unsigned char gmReservedB;
} PACKET_USER_SIMPLE_S;//1206 byte
#pragma pack()

#ifdef __cplusplus
extern "C"
{
#endif
int swapchar( unsigned char * _data, int size_ ) ;
int checkSum(unsigned char * _dataBuf, int count_ ) ;

DEB_FRAME_S packDEBV3Frame(SCD_FRAME_TYPE_E flag, int regAddress_, int regData_);

void writeFrameBuffer(FRAMS_BUFFER_S *mtFrameMsgBuf, char * _mt_frame, int mt_size);

void readDEBFrameBuffer(FRAMS_BUFFER_S *mtFrameMsgBuf, DEB_FRAME_S *mtRegMap);

int processFrameV5( RFans_UDPFRAMEV5_S *mtFrame,std::vector<SCDRFANS_BLOCK_S> &outBlocks);
int processFrameV6G(RFans_UDP32FRAMEV6G_S *mtFrame, std::vector<SCDRFANS_BLOCK_S> &outBlocks);
//int  searchUDPPacket(UDP_DECBUFFER_S *mtUdpBuffer, BLOCK_VECTOR_S *outBlocks);

int searchBlock(unsigned char *data, int size,int &flag,
								SCDRFANS_BLOCK_S *outBlock) ;

#ifdef __cplusplus
}
#endif


#endif
