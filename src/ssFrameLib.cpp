/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#include "ssFrameLib.h"
#include "string.h"
#include <ros/ros.h>
#include "ioapi.h"

int swapchar( unsigned char * _data, int size_ ) {
  int i = 0 , j = 0;
  char tmp = 0 ;
  for ( i=0, j= size_ - 1; i < size_ / 2 ; i++ , j--){
    tmp = _data[i] ;
    _data[i] = _data[j];
    _data[j] = tmp ;
  }
  return 0;
}

int checkSum(unsigned char * _dataBuf, int count_ ) {
  int rtn = 0 ;
  for( int i = 0 ; i < count_ ; i++ ) {
    rtn += _dataBuf[ i ] ;
  }
  rtn = rtn & 0xFF ;
  return rtn ;
}

DEB_FRAME_S packDEBV3Frame(SCD_FRAME_TYPE_E flag, int regAddress, int regData)
{
  DEB_FRAME_S tmpFrame_;
  memset(&tmpFrame_, 0, sizeof(DEB_FRAME_S));

  switch (flag) {
  case eCmdWrite:
    tmpFrame_.msgHead = DEB_FRAME_WRITE;
    break;
  case eCmdRead:
    tmpFrame_.msgHead = DEB_FRAME_READ;
    break;
  case eCmdQuery:
    tmpFrame_.msgHead = DEB_FRAME_READ;
    break;
  default:
    tmpFrame_.msgHead = DEB_FRAME_WRITE;
    break;
  }
  unsigned short tmpSData = 0;
  tmpSData = regAddress;
  swapchar((unsigned char *)&tmpSData, sizeof(tmpSData) ) ;

  tmpFrame_.regAddress = tmpSData;// htons((unsigned short)regAddress_);


  swapchar((unsigned char *)&regData, sizeof(regData));
  tmpFrame_.regData = regData;// htonl(regData_);
  //check sum
  unsigned char *tmpBase = (unsigned char*)&tmpFrame_;
  tmpFrame_.msgCheckSum = checkSum(tmpBase + 2, sizeof(DEB_FRAME_S)-2);
  return tmpFrame_;
}

void bufferToStruck(FRAMS_BUFFER_S *mtFrameMsgBuf, void * mtframe, int mtFrameSize) {
  unsigned char* _tmp_buffer = NULL;
  _tmp_buffer = (unsigned char*)mtframe;
  int tmpIdx = mtFrameMsgBuf->readIdx;
  for (int i = 0; i < mtFrameSize; i++) {
    _tmp_buffer[i] = mtFrameMsgBuf->msgStream[tmpIdx];
    tmpIdx = (tmpIdx + 1) % FRAME_MSG_LENGTH;
  }
  return;
}

void writeFrameBuffer(FRAMS_BUFFER_S *mtFrameMsgBuf, char * _mt_frame, int mt_size) {
  for (int i = 0; i < mt_size; i++) {
    mtFrameMsgBuf->msgStream[mtFrameMsgBuf->writeIdx] = _mt_frame[i];
    mtFrameMsgBuf->writeIdx = (mtFrameMsgBuf->writeIdx + 1) % FRAME_MSG_LENGTH;
    mtFrameMsgBuf->length++;
  }
  return;
}

void readDEBFrameBuffer(FRAMS_BUFFER_S *mtFrameMsgBuf, DEB_FRAME_S *mtRegMap) {
  DEB_FRAME_S tmp_frame;
  int tmp_proLength = sizeof(DEB_FRAME_S);
  while (mtFrameMsgBuf->length >= tmp_proLength) {
    bufferToStruck(mtFrameMsgBuf, (void*)&tmp_frame, tmp_proLength);
    switch (tmp_frame.msgHead) {
    case DEB_FRAME_READ:
      mtRegMap[tmp_frame.regAddress%ROMREG_MAX_COUNT] = tmp_frame;
      mtFrameMsgBuf->readIdx = (mtFrameMsgBuf->readIdx + tmp_proLength) % FRAME_MSG_LENGTH;
      mtFrameMsgBuf->length -= tmp_proLength;
      break;
    default:
      mtFrameMsgBuf->readIdx = (mtFrameMsgBuf->readIdx + 1) % FRAME_MSG_LENGTH;
      mtFrameMsgBuf->length--;
      break;
    }
  }
  return;
}

bool checkGMFormat(RFans_UDP32FRAMEV6G_S * tmpGM)
{
  bool rtn = true;

 if (tmpGM->dataBlock[0].flag == RFANS_UDPFRAMV6G_FLAT
     && tmpGM->dataBlock[11].flag == RFANS_UDPFRAMV6G_FLAT) rtn = true;
 else rtn = false;

  return rtn;
}


/** @brief get the block from data
               *
               * @param data , buffer
               * @param size , size of buffer
							 * @param outflag , 0: no 1:  yes
               * @param outBlock , ouput SCDRFANS_BLOCK_S outBlock.
               * exist : outBlock.dataID == key
               * nothing: outBlock.dataID == 0
               * @returns
               * exist :  The index
               * nothing: The residual size
               */
//int  searchUDPPacket(UDP_DECBUFFER_S *mtUdpBuffer, BLOCK_VECTOR_S *outBlocks) {

//  unsigned char tmpChk = 0 ;
// SCDRFANS_BLOCK_S *tmpBlock;
//  while (mtUdpBuffer->bufSize > UDP_PACKET_SIZE_V6G) {

//    switch( mtUdpBuffer->buffer[mtUdpBuffer->rdTail] ) {
//    //decode BLock Format
//    case ID_RFANSBLOCKV32_16_31_SYNC:
//    case ID_RFANSBLOCKV32_0_15_SYNC:
//    case ID_RFANSBLOCKV2_SYNC:
//    case ID_RFANSBLOCKV6G_0_15_SYNC:
//    case ID_RFANSBLOCKV6G_16_31_SYNC:
//      tmpBlock = (SCDRFANS_BLOCK_S *)(mtUdpBuffer->buffer + mtUdpBuffer->rdTail);

//      tmpChk = checkSum(((unsigned char*)tmpBlock) + 2, sizeof(SCDRFANS_BLOCK_S)-2);
//      if (tmpChk != tmpBlock->chksum) {  //check sum
//        mtUdpBuffer->rdTail++;
//        mtUdpBuffer->bufSize--;
//      } else {
//        mtUdpBuffer->rdTail += sizeof(SCDRFANS_BLOCK_S);
//        mtUdpBuffer->bufSize -= sizeof(SCDRFANS_BLOCK_S);
//        outBlocks->blocks[outBlocks->count++] = *tmpBlock;
//      }

//      break;
//    //decode GM Format
//    case ID_RFANSBLOCK_GM_SYNC:
//      if ( mtUdpBuffer->bufSize >= sizeof(RFans_UDP32FRAMEV6G_S))
//      {
//        RFans_UDP32FRAMEV6G_S *tmpGM = (RFans_UDP32FRAMEV6G_S *)(mtUdpBuffer->buffer + mtUdpBuffer->rdTail);
//        if (checkGMFormat(tmpGM))
//        {
//          processFrameV6G(tmpGM, outBlocks);
//          mtUdpBuffer->rdTail += sizeof(RFans_UDP32FRAMEV6G_S);
//          mtUdpBuffer->bufSize -= sizeof(RFans_UDP32FRAMEV6G_S);
//        } else
//        {
//          mtUdpBuffer->rdTail++;
//          mtUdpBuffer->bufSize--;
//        }
//      }
//      else return 0;
//      break;
//    default:
//      mtUdpBuffer->rdTail++;
//      mtUdpBuffer->bufSize--;
//      break;
//    }

//  }//end while
//  return 0;
//}


int searchBlock(unsigned char *data, int size,int &outflag, SCDRFANS_BLOCK_S *outBlock) {
  int tmpSize = size;
  int tmpReadIndex = 0;
  int tmpOffset = 1 ;
  unsigned char tmpChk = 0 ;
  outBlock->dataID = 0;
  outflag = 0 ;

  while (tmpSize > sizeof(SCDRFANS_BLOCK_S) ) {
    switch( data[tmpReadIndex] ) {
    case ID_RFANSBLOCKV32_16_31_SYNC:
    case ID_RFANSBLOCKV32_0_15_SYNC:
    case ID_RFANSBLOCKV2_SYNC:
    case ID_RFANSBLOCKV6G_0_15_SYNC:
    case ID_RFANSBLOCKV6G_16_31_SYNC:
      memcpy(outBlock, data + tmpReadIndex, sizeof(SCDRFANS_BLOCK_S));
      tmpChk = checkSum(((unsigned char*)outBlock) + 2, sizeof(SCDRFANS_BLOCK_S)-2);

      if (tmpChk != outBlock->chksum) {  //check sum
        tmpReadIndex += tmpOffset ;
        tmpSize--;
      } else {
        outflag = 1;
        return tmpReadIndex; // ROS_INFO_STREAM( "searchBlock");
      }
      break;
    default:
      tmpReadIndex += tmpOffset ;
      tmpSize--;
      break;
    }

  }//end while
  return tmpSize;
}
