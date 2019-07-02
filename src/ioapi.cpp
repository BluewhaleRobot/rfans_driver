/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include "ioapi.h"
#include "bufferDecode.h"

namespace rfans_driver
{

static const int POLL_TIMEOUT = 1000; // one second (in msec)
static const size_t packet_size = sizeof(rfans_driver::RfansPacket().data);
static const size_t packet_count = packet_size/ sizeof(SCDRFANS_BLOCK_S );
extern size_t packet_size_pcap;

////////////////////////////////////////////////////////////////////////
// base class implementation
////////////////////////////////////////////////////////////////////////
IOAPI::IOAPI()
{
  m_bufferPro = new  SSBufferDec();
}

IOAPI::~IOAPI() {
  if(m_bufferPro) {
    delete m_bufferPro;
    m_bufferPro = NULL;
  }
}

/** @brief Write the Regidit. */
int IOAPI::HW_WRREG(int flag, int regAddress, unsigned int regData) {
  int rtn = 0;
  DEB_FRAME_S wdFrame_;
  long tmp_len = sizeof(DEB_FRAME_S);
  memset(&wdFrame_, 0, sizeof(DEB_FRAME_S));
  wdFrame_ = packDEBV3Frame(eCmdWrite, regAddress, regData);
  write((unsigned char *)&wdFrame_, tmp_len);
  return rtn;
}

int IOAPI::revPacket(rfans_driver::RfansPacket &pkt)
{
  int rtn = 0 ;

  int readCount = 32;

//  if(m_bufferPro->freeSize()<= UDP_PACKET_SIZE_V5A) return rtn ;
  if(m_bufferPro->freeSize()<=UDP_PACKET_SIZE_DATA_LEVEL_ORI) return rtn;

  while( readCount-- > 0 &&  m_bufferPro->freeSize() > upd_packet_size ) {
    rtn = read(m_bufferPro->getWriteIndex(), upd_packet_size) ;
    if(rtn >0 ) {
      m_bufferPro->moveWriteIndex(rtn) ;
      if(readCount == 31) pkt.stamp = ros::Time::now();
    } else if(rtn < 0 ) break;
  }

  return rtn ;
}

/** @brief Get packet. */
int IOAPI::getPacket(rfans_driver::RfansPacket &pkt) {
  int rtn = 0;
  rtn = m_bufferPro->readPacket(pkt) ;
  return rtn;
}

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////
/** @brief constructor
         *
         *  @param private_nh ROS handle for calling node.
         *  @param port UDP port number
         */
IOSocketAPI::IOSocketAPI(std::string ipstr, uint16_t devport, int16_t pcport)
{
  m_sockfd = -1;
//  ROS_INFO_STREAM("Opening UDP socket: " <<
//                  " pc port "            << pcport<<
//                  " device port  "       << devport<<
//                  " device ip  "         << ipstr );
  m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  if (m_sockfd == -1) {
    perror(" create socket error");
    return;
  }

  sockaddr_in my_addr;
  memset(&my_addr, 0, sizeof(my_addr));
  my_addr.sin_family = AF_INET;
  my_addr.sin_port = htons(pcport);
  my_addr.sin_addr.s_addr = INADDR_ANY;

  memset(&m_devaddr, 0, sizeof(m_devaddr));
  m_devaddr.sin_family = AF_INET;
  m_devaddr.sin_port = htons(devport);
  m_devaddr.sin_addr.s_addr = inet_addr(ipstr.c_str() );

  if (bind(m_sockfd, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
    perror("bind message");
    return;
  }

  if (fcntl(m_sockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    perror("non-block message");
    return;
  }

  //ROS_INFO_STREAM("rfans socket fd is " << m_sockfd);
  reset();
}

/** @brief destructor */
IOSocketAPI::~IOSocketAPI(void)
{
  (void)close(m_sockfd);
}

/** @brief Write */
int IOSocketAPI::write(unsigned char *data, int size)
{
  int rtn = 0 ;
  unsigned char tmpCmd[UDP_FRAME_MIN] ;
  unsigned char *tmpBuf = data ;
  socklen_t sender_address_len = sizeof(m_devaddr);
  if(m_sockfd <= 0) return rtn ;

  if (size < UDP_FRAME_MIN && size > 0) {
    memcpy(tmpCmd, data, size);
    tmpBuf = tmpCmd;
    size = UDP_FRAME_MIN;
  }

  rtn = sendto(m_sockfd, data, size, 0, (sockaddr*)&m_devaddr, sender_address_len ) ;
  if (rtn < 0) {
    //perror("IOSocketAPI:write") ;
  }
  return rtn ;
}

int IOSocketAPI::reset()
{
  m_bufferPro->reset();
  return 0;
}

/** @brief read from RFans UDP Data  */
int IOSocketAPI::read(unsigned char *data, int size)
{
  struct pollfd fds[1];
  fds[0].fd = m_sockfd;
  fds[0].events = POLLIN;
  int nbytes = 0 ;
  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  int retval = 0 ;
  retval = poll(fds, 1, POLL_TIMEOUT);
  if(fds[0].revents & POLLIN) {
    nbytes = recvfrom(m_sockfd, data, size, 0,
                      (sockaddr*)&sender_address, &sender_address_len);
    if(m_devaddr.sin_addr.s_addr != sender_address.sin_addr.s_addr) {
      nbytes = 0 ;
    }
  }
  if (retval == 0) {
    ROS_WARN("IOSocketAPI::read  poll() timeout");
    nbytes = 0;
  }
  return nbytes ;
}
////////////////////////////////////////////////////////////////////////
// SSFileAPI class implementation
////////////////////////////////////////////////////////////////////////
SSFileAPI::SSFileAPI(const char *fileName) {
  s_rawFile = NULL;
    reset();

  if(fileName) {


    s_rawFile = fopen(fileName,"rb+") ;
    if(s_rawFile)
          ROS_INFO("open file %s",fileName);
  }
  m_blocks = (SCDRFANS_BLOCK_S *)malloc(packet_count*sizeof(SCDRFANS_BLOCK_S) );
}

SSFileAPI::~SSFileAPI() {
  if(s_rawFile)
    fclose(s_rawFile);
  if(m_blocks)
    free(m_blocks);
}

int SSFileAPI::create_file(int flag)
{
  time_t rawtime;
  struct tm * timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  char tmpFileName[FILENAME_MAX] = { 0 };


  if(flag){
    sprintf(tmpFileName, "%s/data-%d%d%d-%d%d%d.imp", s_dataPath,
            timeinfo->tm_year, timeinfo->tm_mon, timeinfo->tm_mday,
            timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    s_rawFile = fopen(tmpFileName, "wb+");
  } else {
    sprintf(tmpFileName, "%s/data-%d%d%d-%d%d%d.xyz", s_dataPath,
            timeinfo->tm_year, timeinfo->tm_mon, timeinfo->tm_mday,
            timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    s_rawFile = fopen(tmpFileName, "w+");
    if(s_rawFile)
      fprintf(s_rawFile,"x y z intent laserID timeFlag\n");
  }

  return 0;
}

int SSFileAPI::outputFile(std::vector<SCDRFANS_BLOCK_S> &outBlocks, std::vector<RFANS_XYZ_S> &outXyzBlocks, int flag) {
  int rtn = 0 ;
  if(!s_rawFile) {
    create_file();
  }
  int tmpXyzIndex = 0 ;

  double tmpTime = 0 ;
  float timeOffset = 31.25;
  float timeScale = 0.0000001;
  float tmpRange = 0 ;
  float tmpAngle = 0 ;
  float UINTCONVERT = 0.01 ;

  if (s_rawFile) {
    for(int i=0;i <outBlocks.size();i++){
      for(int j=0 ; j < RFANS_LASER_COUNT && tmpXyzIndex < outXyzBlocks.size() ;j++) {
        tmpXyzIndex = i*RFANS_LASER_COUNT+j;
        tmpTime = outBlocks[i].t0stampH + outBlocks[i].t0stampL * timeOffset *timeScale * j; //calc UTC time

        tmpRange = outBlocks[i].laserData[j].rangeOne*UINTCONVERT;
        tmpAngle = outBlocks[i].laserData[j].angle*UINTCONVERT;
        switch(flag) {
//        case 0:// for test
//          tmpTime = outBlocks[i].t0stampL;

//          fprintf(s_rawFile,"%f %f %d %d %lf\n",
//                  tmpRange , tmpAngle,
//                  outBlocks[i].laserData[j].intentOne, j , tmpTime) ;
//          break;
//        case 2:// for test
//          fprintf(s_rawFile,"%f %f %f %f %f %d %d %lf\n",
//                  outXyzBlocks[tmpXyzIndex].x,   outXyzBlocks[tmpXyzIndex].y,  outXyzBlocks[tmpXyzIndex].z,
//                  tmpRange, tmpAngle,
//                  outBlocks[i].laserData[j].intentOne, j , tmpTime) ;
//          break;
        default:
          tmpTime = outBlocks[i].t0stampL;

          fprintf(s_rawFile,"%f %f %f %d %d %lf\n",
                  outXyzBlocks[tmpXyzIndex].x,   outXyzBlocks[tmpXyzIndex].y,  outXyzBlocks[tmpXyzIndex].z,
                  outBlocks[i].laserData[j].intentOne, j , tmpTime) ;
          break;
        }

      } //end laser number
    } //end packet number
    if ( ftell(s_rawFile) >= DATA_FILE_SIZE) {
      create_file();
    }
  }
  return rtn ;
}

int SSFileAPI::outputFile(sensor_msgs::PointCloud2 &pointCloud, int flag)
{
  int rtn =0;

  if(!s_rawFile) {
    create_file(1);
  }

  RFANS_XYZ_S *tmpXyzs = (RFANS_XYZ_S *)(&pointCloud.data[0]) ;
  if (s_rawFile) {
    write( (unsigned char *)tmpXyzs, pointCloud.data.size() ) ;
  }

  return rtn ;
}

int SSFileAPI::write(unsigned char *data, int size)  {
  int rtn = 0 ;
  if(!s_rawFile) {
    create_file(1);
  }

  if (s_rawFile) {
    rtn = fwrite(data, 1, size, s_rawFile);
    if ( ftell(s_rawFile) >= DATA_FILE_SIZE) {
      create_file(1);
    }
  }
  return rtn ;
}

int SSFileAPI::read(unsigned char *data, int size) {
  int rtn = 0 ;
  if(!s_rawFile) return rtn ;

  if( !feof(s_rawFile) ) {
    rtn = fread(data,1,size,s_rawFile);
  } else {
    fseek(s_rawFile, 0 ,SEEK_SET );
  }
  usleep(12000) ;
  //usleep(1000) ;
  return rtn;
}

int SSFileAPI::reset() {
  if(s_rawFile) {
    fclose(s_rawFile);
    s_rawFile = 0 ;
  }
  return 0;
}

int SSFileAPI::printf(char *msgStr, int size)
{
  int rtn = 0 ;
  if(!s_rawFile) {
    create_file();
  }
  char tmpMsg[FILENAME_MAX] ;

  if(size >FILENAME_MAX || size <=0) return 0 ;
  memcpy(tmpMsg, msgStr, size);

  if (s_rawFile) {
    fprintf(s_rawFile,"%s\n",tmpMsg);
    if ( ftell(s_rawFile) >= DATA_FILE_SIZE) {
      create_file();
    }
  }
  return 0;
}

////end read

    InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port,
                          double packet_rate,std::string filename,
                         std::string device_ip,bool read_once,
                         bool read_fast, double repeat_delay):
       private_nh_(private_nh),
       port_(port),
       packet_rate_(packet_rate),
       filename_(filename),
       devip_str_(device_ip)
     {
       pcap_ = NULL;
       empty_ = true;

       // get parameters using private node handle
       private_nh.param("read_once", read_once_, false);
       private_nh.param("read_fast", read_fast_, false);
       private_nh.param("repeat_delay", repeat_delay_, 0.0);

       if (read_once_)
         ROS_INFO("Read input file only once.");
       if (read_fast_)
         ROS_INFO("Read input file as quickly as possible.");
       if (repeat_delay_ > 0.0)
         ROS_INFO("Delay %.3f seconds before repeating input file.",
                  repeat_delay_);

       // Open the PCAP dump file
       ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
       if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_) ) == NULL)
         {
           ROS_FATAL("Error opening Rfans socket dump file, please set a correct path.");
           return;
         }

       std::stringstream filter;
       if( devip_str_ != "" )              // using specific IP?
         {
           filter << "src host " << devip_str_ << " && ";
         }
       filter << "udp dst port " << port;
       pcap_compile(pcap_, &pcap_packet_filter_,
                    filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
     }

     /** destructor */
     InputPCAP::~InputPCAP(void)
     {
       pcap_close(pcap_);
     }

     /** @brief Get one R-Fans packet. */
     int InputPCAP::getPacket(rfans_driver::Packet *pkt)
     {
       struct pcap_pkthdr *header;
       const u_char *pkt_data;

       while (true)
         {
           int res;
           if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
             {
               // Skip packets not for the correct port and from the
               // selected IP address.
               if (0 == pcap_offline_filter(&pcap_packet_filter_,
                                             header, pkt_data))
                 continue;

               // Keep the reader from blowing through the file.
               if (read_fast_ == false)
                 packet_rate_.sleep();
               packet_size_pcap = header->caplen - 42;
               pkt->data.resize(packet_size_pcap);
               memcpy(&pkt->data[0], pkt_data+42, packet_size_pcap);
               pkt->stamp = ros::Time::now(); // time_offset not considered here, as no synchronization required
               empty_ = false;
               return 0;                   // success
             }

           if (empty_)                 // no data in file?
             {
               ROS_WARN("Error %d reading R-Fans packet: %s",
                        res, pcap_geterr(pcap_));
               return -1;
             }

           if (read_once_)
             {
               ROS_INFO("end of file reached -- done reading.");
               return -1;
             }

           if (repeat_delay_ > 0.0)
             {
               ROS_INFO("end of file reached -- delaying %.3f seconds.",
                        repeat_delay_);
               usleep(rint(repeat_delay_ * 1000000.0));
             }

           ROS_DEBUG("replaying R-Fans dump file");

           // I can't figure out how to rewind the file, because it
           // starts with some kind of header.  So, close the file
           // and reopen it with pcap.
           pcap_close(pcap_);
           pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
           empty_ = true;              // maybe the file disappeared?
         } // loop back and try again
     }



}//end namespace
