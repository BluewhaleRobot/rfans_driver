#ifndef _POINT_TYPES_H_
#define _POINT_TYPES_H_
#pragma pack(1)
typedef struct {
  float x,y,z ;
  float intent;
  int laserid;
  float timeflag;
  float hangle;
  float pulseWidth;
  float range;
  unsigned char mirrorid;
}RFANS_XYZ_S;
#pragma pack()
#endif

