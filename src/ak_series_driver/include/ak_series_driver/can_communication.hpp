#ifndef CUBEMARS_AK_SERIES_DRIVER__CAN_COMMUNICATION_HPP_
#define CUBEMARS_AK_SERIES_DRIVER__CAN_COMMUNICATION_HPP_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>

#define CAN_EFF_FLAG 0x80000000U
#define CAN_RTR_FLAG 0x40000000U
#define CAN_ERR_FLAG 0x20000000U

using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

class CanCommunication
{
    public:
        int ret;
        int s;
        int nbytes;
        struct sockaddr_can addr;
        struct ifreq ifr;
        struct canfd_frame frame;
        
        int start(const int channel, const int bitrate)
        {
            memset(&frame, 0, sizeof(struct canfd_frame));

            std::stringstream cmd;
            cmd << "echo Guitar14 | sudo -S ifconfig can" << channel << " down && sudo -S ip link set can" << channel << " up type can bitrate " << bitrate << " dbitrate 800000 fd on";
           // system(cmd.str().c_str());
            cmd << "echo Guitar14 | sudo -S ip link set can" << channel << " up type can bitrate " << bitrate << " dbitrate 800000 fd on";
            //system(cmd.str().c_str());
            cmd << "sudo ifconfig can" << channel << " up\n";
            //system(cmd.str().c_str());

            s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
            if (s < 0) {
                perror("Create socket PF_CAN failed!");
                return 1;
            }
            
            /*Specify can device*/
            
            char temp[10];
            sprintf(temp, "can%d", channel);
            strcpy(ifr.ifr_name, temp);
            ret = ioctl(s, SIOCGIFINDEX, &ifr);
            if (ret < 0) {
                perror("ioctl interface index failed!");
                return 1;
            }
            
            /*Bind the socket to can0*/
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;
            setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
            int fd_msg = 1;
            int fr_frames = setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &fd_msg, sizeof(fd_msg));
           
            ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
            if (ret < 0) {
                perror("bind failed!");
                return 1;
            }
            if (fr_frames < 0)
            {
                perror("Setting CAN_RAW_FD_FRAMES Failed");
                return 1;
            } 
         
            return 0;
           
        }

        void send_cmd(uint32_t can_id, uint8_t can_dlc, uint8_t *data)
        {
            
            frame.can_id = can_id | CAN_EFF_FLAG;
            frame.len = can_dlc;
            std::cout << "Can ID ";
            printf("%X", std::to_string(frame.can_id).c_str());
               std::cout << std::endl;
      
            for (unsigned int i = 0; i < sizeof(data); i++)
            {
                frame.data[i] = data[i];
                printf("%X", std::to_string(data[i]).c_str());
                std::cout << std::endl;
            }
            std::cout << std::endl;
            
            /*Send message out */
            nbytes = write(s, &frame, sizeof(struct canfd_frame)); 
            
            printf("%s", std::to_string(nbytes).c_str());
            std::cout << std::endl;
            
            if(nbytes == -1) {
                
                printf("Send frame incompletely!\r\n");
                
            }
            sleep_for(nanoseconds(1000));
        }

        void can_read()
        {
            nbytes = read(s, &frame, sizeof(frame));
            if (nbytes < 0) {
                perror("can raw socket read");
                //return 1;
            }

            /* paranoid check ... */
            if ((long unsigned) nbytes < sizeof(struct canfd_frame)) {
                fprintf(stderr, "read: incomplete CAN frame\n");
                //return 1;
            }
            
        }


    private:
        uint8_t response[6];
        unsigned char find_can(const int port)
        {
            char buf[128]={0};
            sprintf(buf,"/sys/class/net/can%d/can_bittiming/bitrate",port);
            return ((access(buf,0)==0));
        }


};


#endif // CUBEMARS_AK_SERIES_DRIVER__CAN_COMMUNICATION_HPP_