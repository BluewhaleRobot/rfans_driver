#include "calculation.h"
#include <math.h>
#include <cmath>
float m_anglePara[30]={0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,45,-15,45,-15,0,0};
float m_anglePara_32[30]={0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,45,-15,45,-15,0,0};
double m_mirrorVector[4][3];
double m_lidarAngle[2][32];
float cfans_lidarAngle[2][8];
float mirrorVector[4][3];
std::vector<float> s_reviseangles;
std::vector<float> s_reviseangles_32;
Calculation::Calculation()
{

}

Calculation::~Calculation()
{

}
inline unsigned int strToList(std::vector<float> &list, std::string str)
{
    unsigned int count = 0;
    string::size_type prev = 0;
    string::size_type pos = 0;
    string tmpValue ;
    list.clear();
    while((pos = str.find_first_of(',', pos))!= string::npos)
    {
        tmpValue =  str.substr(prev, pos - prev);
        list.push_back( atof(tmpValue.c_str()));
        pos++;
        prev = pos;
        count++;
    }
    return count;
}
int calcXyz(unsigned char flag, float &mtRange, float &mtAngle, RFANS_XYZ_S &outXyz)
{
    int rtn = 1;
    double tmptheta=0, ot = 0 ;

    switch (flag)
    {
    case RFANS_PRODUCT_MODEL_V6G_X32_0X33:
        mtAngle+= HANGLE_V6G_X32_0X33[outXyz.laserid];
        tmptheta = VANGLE_V6G_X32_0X33[outXyz.laserid] * M_PI / 180.0;
        break;
    case RFANS_PRODUCT_MODEL_V6P_X32_0X45:
    case RFANS_PRODUCT_MODEL_V6_X32_0X40:
        mtAngle+= HANGLE_V6_X32_0x40[outXyz.laserid];
        tmptheta = VANGLE_V6_X32_0X40[outXyz.laserid] * M_PI / 180.0;
        break;
    case RFANS_PRODUCT_MODEL_V6P_X16A_0X46:
    case RFANS_PRODUCT_MODEL_V6_X16A_0X41:
        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;

        mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[outXyz.laserid]];
        tmptheta = VANGLE_V6_X32_0X40[RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
        break;

    case RFANS_PRODUCT_MODEL_V6P_X16B_0X47:
    case RFANS_PRODUCT_MODEL_V6_X16B_0X42:

        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;

        mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[outXyz.laserid]];
        tmptheta = VANGLE_V6_X32_0X40[RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
        break;
    case RFANS_PRODUCT_MODEL_V6P_X16Even_0X48:
    case RFANS_PRODUCT_MODEL_V6_X16Even_0X43:

        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
        mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[outXyz.laserid]];
        tmptheta = VANGLE_V6_X32_0X40[RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
        break;

    case RFANS_PRODUCT_MODEL_V6P_X16Odd_0X49:
    case RFANS_PRODUCT_MODEL_V6G_X16_0X32:
    case RFANS_PRODUCT_MODEL_V6_X16Odd_0X44:
        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
        tmptheta = VANGLE_V6_X32_0X40[RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
        mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[outXyz.laserid]];
        break;

    case RFANS_PRODUCT_MODEL_V6A_X32_0X4A:
        mtAngle+= HANGLE_V6_X32_0x40[outXyz.laserid];
        tmptheta = VANGLE_V6A_X32[outXyz.laserid] * M_PI / 180.0;
        break;
    case RFANS_PRODUCT_MODEL_V6A_X16A_0X4B:
        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;

        mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[outXyz.laserid]];
        tmptheta = VANGLE_V6A_X32[RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
        break;
    case RFANS_PRODUCT_MODEL_V6A_X16B_0X4C:
        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;

        mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[outXyz.laserid]];
        tmptheta = VANGLE_V6A_X32[RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
        break;
    case RFANS_PRODUCT_MODEL_V6A_X16Even_0X4D:

        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
        mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[outXyz.laserid]];
        tmptheta = VANGLE_V6A_X32[RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
        break;
    case RFANS_PRODUCT_MODEL_V6A_X16Odd_0X4E:
        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
        tmptheta = VANGLE_V6A_X32[RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
        mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[outXyz.laserid]];
        break;
    case RFANS_PRODUCT_MODEL_V6A_E1_0X55:
        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
        tmptheta = VAngle_16E1[outXyz.laserid] * M_PI / 180.0;
        mtAngle+= HANGLE_V6A_E1_0x55[outXyz.laserid];
        break;
    case RFANS_PRODUCT_MODEL_V6A_E2_0X56:
        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
        tmptheta = VAngle_16E2[outXyz.laserid] * M_PI / 180.0;
        mtAngle+= HANGLE_V6A_E1_0x55[outXyz.laserid];
        break;
    case ID_RFANSBLOCKV32_16_31_SYNC:
        outXyz.laserid += RFANS_LASER_COUNT;
        tmptheta = VANGLE_V6_X32_0X40[outXyz.laserid] * M_PI / 180.0;
        break;
    case ID_RFANSBLOCKV6G_16_31_SYNC:
        outXyz.laserid += RFANS_LASER_COUNT;
        tmptheta = VANGLE_V6G_X32_0X33[outXyz.laserid] * M_PI / 180.0;
        break;
    case ID_RFANSBLOCKV32_0_15_SYNC:
        tmptheta = VANGLE_V6_X32_0X40[outXyz.laserid] * M_PI / 180.0;
        break;
    case ID_RFANSBLOCKV6G_0_15_SYNC:
        tmptheta = VANGLE_V6G_X32_0X33[outXyz.laserid] * M_PI / 180.0;
        break;
    case ID_RFANSBLOCKV2_SYNC:
        tmptheta = VANGLE_V5_X16[outXyz.laserid] * M_PI / 180.0;
        break;
    case RFANS_PRODUCT_MODEL_V6A_X16M_0X4F:
        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
        mtAngle+= HANGLE_V5_X16[outXyz.laserid];
        tmptheta = VANGLE_V5_X16[outXyz.laserid] * M_PI / 180.0;
        break;
    case RFANS_PRODUCT_MODEL_V6B_X32_0X50:
        mtAngle += HANGLE_V6B_X32_0x40[outXyz.laserid];
        tmptheta = VANGLE_V6A_X32[outXyz.laserid] * M_PI / 180.0;
        break;
    case RFANS_PRODUCT_MODEL_V6BC_16G_0X57:
        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
        mtAngle += HANGLE_V6BC_16G_0x57[outXyz.laserid];
        tmptheta = VAngle_V6B_16G[outXyz.laserid] * M_PI / 180.0;
        break;
    case RFANS_PRODUCT_MODEL_V6BC_16M_0X58:
        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
        mtAngle += HANGLE_V6BC_16M_0x58[outXyz.laserid];
        tmptheta = VAngle_V6B_16M[outXyz.laserid] * M_PI / 180.0;
        break;
    case   RFANS_PRODUCT_MODEL_V6C_Z_X32_0X59:
        mtAngle += HANGLE_V6B_X32_0x40[outXyz.laserid];
        tmptheta = VAngle_V6C_Z_32[outXyz.laserid] * M_PI / 180.0;
        break;
    case  RFANS_PRODUCT_MODEL_V6K_32M_0X5A:
        mtAngle += HANGLE_V6K_32M_0X5A[outXyz.laserid];
        tmptheta = VANGLE_V6K_32M_0x5A[outXyz.laserid]* M_PI / 180.0;
        break;

    case RFANS_PRODUCT_MODEL_V6K_16M_0X5B:
        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
        mtAngle += HANGLE_V6K_16M_0x5B[outXyz.laserid];
        tmptheta = VANGLE_V6K_16M_0x5B[outXyz.laserid]* M_PI / 180.0;
        break;

    case RFANS_PRODUCT_MODEL_V6K_16G_0x5C:
        outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
                    outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
        mtAngle += HANGLE_V6BC_16G_0x57[outXyz.laserid];
        tmptheta = VAngle_V6B_16G[outXyz.laserid]* M_PI / 180.0;
        break;

    }

    if(mtAngle>360) mtAngle -= 360;
    if(mtAngle<0) mtAngle +=360 ;

    ot = mtAngle*M_PI / 180.0;

    outXyz.x = mtRange*cos(tmptheta) *cos(-ot );
    outXyz.y = mtRange*cos(tmptheta) *sin(-ot );
    outXyz.z = mtRange*sin(tmptheta) ;
    outXyz.hangle = mtAngle;
    return rtn ;
}

int initCFans_32(std::string revisePara)
{
    float tmpAngle[2][8] ={{-8.4, -6, -3.6, -1.2, 1.2, 3.6, 6, 8.4},{0}};
    for(int i = 0; i<2; i++)
    {
        memcpy(cfans_lidarAngle[i],tmpAngle[i],sizeof(tmpAngle[i]));
    }

    float Mirror_AngleY1 = -0.9*M_PI / 180.0;
    float Mirror_AngleX2 = 0*M_PI / 180.0;
    float Mirror_AngleY3 = -0.3*M_PI / 180.0;
    float Mirror_AngleX4 = -0.6*M_PI / 180.0;

    float tmpPlaneNormal[4][3]={{-cos(Mirror_AngleY1), 0, sin(Mirror_AngleY1)},
                                { 0, -cos(Mirror_AngleX2), sin(Mirror_AngleX2)},
                                {cos(Mirror_AngleY3), 0, sin(Mirror_AngleY3)},
                                { 0, cos(Mirror_AngleX4), sin(Mirror_AngleX4)}
                             };
    // rotation matrix.
    for(int i=0; i<4; i++)
    {
        memcpy(mirrorVector[i],tmpPlaneNormal[i],sizeof(tmpPlaneNormal[i]));
    }
    strToList(s_reviseangles_32,revisePara);
    for(int i =0; i<s_reviseangles_32.size();i++)
    {
       m_anglePara_32[i] = s_reviseangles_32[i];
    }

    return 1;

}
int initCFansPara(std::string reviseAngle) {
    //readAngleParaFile() ;
    // 32个激光器的角度
    double tmpAngle[2][32]= { {-13.6, -13.35,-11.82,-11.57,-10.04 ,-9.79,-8.26 ,-8.01,-6.48,-6.23,-4.7,-4.45,-2.92,-2.67,-1.14,-0.89,0.64,0.89,2.42,2.67,4.2,4.45,5.98,6.23,7.76,8.01,9.54,9.79,11.32,11.57,13.1,13.35},
                              { -14.95, 15.95, -17.05, 14.05, -14.95, 15.95, -17.05, 14.05, -14.95, 15.95, -17.05, 14.05, -14.95, 15.95, -17.05, 14.05, -14.95, 15.95, -17.05, 14.05, -14.95, 15.95, -17.05, 14.05, -14.95, 15.95, -17.05, 14.05, -14.95, 15.95, -17.05, 14.05}   };
    for (int i=0; i<2; i++)  {
        memcpy(m_lidarAngle[i], tmpAngle[i], sizeof(tmpAngle[i]));
    }
    //旋转矩阵
     double tmpPlaneNormal[4][3]= { { -1.0000, 0, 0.0066 }, { 0, -1.0000, 0 }, { 1.0000, 0, -0.0066 }, { 0 , 0.9999 ,-0.0133 }} ;
    for (int i=0; i<4; i++)    {
        memcpy(m_mirrorVector[i], tmpPlaneNormal[i], sizeof(tmpPlaneNormal[i]));
    }

    strToList(s_reviseangles, reviseAngle);
    for (int i = 0; i < s_reviseangles.size(); ++i) {
        m_anglePara[i] = s_reviseangles[i];
    }
    //    char angle[256] = { 0 };
    //    char cmd[512] = {0};
    //    sprintf(angle, "%f,%f,%f,%f,%f,%f,%f,%f", m_anglePara[0], m_anglePara[1],m_anglePara[2],
    //            m_anglePara[3],m_anglePara[4],m_anglePara[5],m_anglePara[6],m_anglePara[7]);
    //    sprintf(cmd, "echo \"%s\" > aaa.txt", angle);
    //    system(cmd);
    return 1;
}

int calcCFansXYZ_32(float range, float angle, int index, int laserID, RFANS_XYZ_S &outXyz)
{
    float tmpMirrorVector[3];
    memcpy(tmpMirrorVector,mirrorVector[index],sizeof(mirrorVector[index]));

    angle+= m_anglePara_32[24];
    if(angle <0)
    {
        angle+=360.0;
    }
    else if(angle>360.0)
    {
        angle-=360.0;
    }
    float angleH =cfans_lidarAngle[1][laserID]*M_PI/180.0;
    float angleV = cfans_lidarAngle[0][laserID]*M_PI/180.0;
    float rayDirection[3] = {fabs(sin(angleH)*cos(angleV)), -cos(angleH)*cos(angleV), sin(angleV)};
    float stb = sin(angle*M_PI/180.0);
    float ctb = cos(angle*M_PI/180.0);

    float tmptt = fabs(( (ctb*tmpMirrorVector[0] + stb*tmpMirrorVector[1])*rayDirection[0]+
                  (-stb*tmpMirrorVector[0] + ctb*tmpMirrorVector[1])*rayDirection[1]+
                  tmpMirrorVector[2]*rayDirection[2]) );

    float tmpXyz[3];
    tmpXyz[0] = (rayDirection[0] + 2*(ctb*tmpMirrorVector[0] + stb*tmpMirrorVector[1])*tmptt)* range;
    tmpXyz[1] = (rayDirection[1] + 2*(-stb*tmpMirrorVector[0]+ctb*tmpMirrorVector[1])*tmptt)*range;
    tmpXyz[2] = (rayDirection[2] + 2*tmpMirrorVector[2]*tmptt)*range;

    float angleCorrX, angleCorrY, angleCorrZ;
    int planeBaseIndex;
    planeBaseIndex = index*3;
    tmpXyz[1] += m_anglePara_32[28];
    angleCorrX = m_anglePara_32[planeBaseIndex+0];
    angleCorrY = m_anglePara_32[planeBaseIndex+1];
    angleCorrZ = m_anglePara_32[planeBaseIndex+2];
    outXyz.x = tmpXyz[1] * (cos(angleCorrX)*sin(angleCorrZ) - cos(angleCorrZ)*sin(angleCorrX)*sin(angleCorrY))
               - tmpXyz[2] * (sin(angleCorrX)*sin(angleCorrZ) + cos(angleCorrX)*cos(angleCorrZ)*sin(angleCorrY))
               + tmpXyz[0] * cos(angleCorrY)*cos(angleCorrZ);
    outXyz.y = tmpXyz[1] * (cos(angleCorrX)*cos(angleCorrZ) + sin(angleCorrX)*sin(angleCorrY)*sin(angleCorrZ))
            - tmpXyz[2] * (cos(angleCorrZ)*sin(angleCorrX) - cos(angleCorrX)*sin(angleCorrY)*sin(angleCorrZ))
            - tmpXyz[0] * cos(angleCorrY)*sin(angleCorrZ);
    outXyz.z = tmpXyz[0] * sin(angleCorrY)
            + tmpXyz[2] * cos(angleCorrX)*cos(angleCorrY)
            + tmpXyz[1] * cos(angleCorrY)*sin(angleCorrX);
}
int calcCFansCoor(float range, float angle, int index, int laserID, RFANS_XYZ_S &outXyz)
{

    int rtn = 1;
    // float tmpXYZ[3];
    double tmpMirrorVector[3];
    memcpy(tmpMirrorVector, m_mirrorVector[index], sizeof(m_mirrorVector[index]));

    /* if (laserID % 2 == 0) {
       //angle += 45 ;
       angle += m_anglePara[24];
   }else{
       // angle -= 15 ;
       angle += m_anglePara[25];
   }*/

    switch (laserID % 4) {
    case 0:
        angle  += m_anglePara[24];
        break;
    case 1:
        angle  += m_anglePara[25];
        break;
    case 2:
        angle  += m_anglePara[26];
        break;
    case 3:
        angle  += m_anglePara[27];
        break;
    }


    double tmpDrection[3];
    double angleV = m_lidarAngle[0][laserID] * M_PI / 180.0;
    double angleH = m_lidarAngle[1][laserID] * M_PI / 180.0;
    double direction[2][3]={ {abs(sin(angleH)*cos(angleV)), cos(angleH)*cos(angleV), sin(angleV) } ,{abs(sin(angleH)*cos(angleV)), -cos(angleH)*cos(angleV), sin(angleV) }};
    if (laserID % 2 == 0)
        memcpy(tmpDrection,direction[1],sizeof(direction[1])) ;
    else
        memcpy(tmpDrection,direction[0],sizeof(direction[0])) ;

    double stb = sin(angle*M_PI / 180.0);
    double ctb = cos(angle*M_PI / 180.0);
    double tmptt = fabs(( (ctb*tmpMirrorVector[0] + stb*tmpMirrorVector[1])*tmpDrection[0] +(-stb*tmpMirrorVector[0] + ctb*tmpMirrorVector[1])*tmpDrection[1] +tmpMirrorVector[2]*tmpDrection[2]) );
    double tmpXyz0[3];
    tmpXyz0[0] = (tmpDrection[0] + 2 * (ctb*tmpMirrorVector[0] + stb*tmpMirrorVector[1])*tmptt)* range;
    tmpXyz0[1] = (tmpDrection[1] + 2 * (-stb*tmpMirrorVector[0] + ctb*tmpMirrorVector[1])*tmptt)* range;
    tmpXyz0[2] = (tmpDrection[2] + 2 * tmpMirrorVector[2] * tmptt)* range;



    double angleCorrX, angleCorrY, angleCorrZ;
    int planBaseIndex;
    if ((laserID % 2) == 0){
        planBaseIndex = index* 3;
        tmpXyz0[1] += m_anglePara[28];
    }else{
        planBaseIndex = index * 3 + 12;
        tmpXyz0[1] += m_anglePara[29];
    }


    angleCorrX = m_anglePara[ planBaseIndex + 0];
    angleCorrY = m_anglePara[ planBaseIndex + 1];
    angleCorrZ = m_anglePara[ planBaseIndex + 2];

    outXyz.x = tmpXyz0[1] * (cos(angleCorrX)*sin(angleCorrZ) - cos(angleCorrZ)*sin(angleCorrX)*sin(angleCorrY)) - tmpXyz0[2] * (sin(angleCorrX)*sin(angleCorrZ) + cos(angleCorrX)*cos(angleCorrZ)*sin(angleCorrY)) + tmpXyz0[0] * cos(angleCorrY)*cos(angleCorrZ);
    outXyz.y= tmpXyz0[1] * (cos(angleCorrX)*cos(angleCorrZ) + sin(angleCorrX)*sin(angleCorrY)*sin(angleCorrZ)) - tmpXyz0[2] * (cos(angleCorrZ)*sin(angleCorrX) - cos(angleCorrX)*sin(angleCorrY)*sin(angleCorrZ)) - tmpXyz0[0] * cos(angleCorrY)*sin(angleCorrZ);
    outXyz.z = tmpXyz0[0] * sin(angleCorrY) + tmpXyz0[2] * cos(angleCorrX)*cos(angleCorrY) + tmpXyz0[1] * cos(angleCorrY)*sin(angleCorrX);

    return rtn;
}
