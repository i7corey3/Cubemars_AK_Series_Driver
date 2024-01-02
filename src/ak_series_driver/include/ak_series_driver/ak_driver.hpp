#ifndef CUBEMARS_AK_SERIES_DRIVER__AK_DRIVER_HPP_
#define CUBEMARS_AK_SERIES_DRIVER__AK_DRIVER_HPP_

#include "ak_series_driver/can_communication.hpp"
#include "ak_series_driver/mit_params.hpp"
#include <cmath>
#include <vector>


class AKDriver
{

    public:
        
        void setup(int channel, int bitrate)
        {
            can_.start(channel, bitrate);
                        
        }

        void start_motor(int addr)
        {
            uint8_t msg[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0Xfc};
            can_.send_cmd(addr, sizeof(msg), msg);
        }

        void set_motor_param(int addr, float position, float velocity, float torque, float Kp, float Kd)
        {

            pack_cmd(addr, position, velocity, Kp, Kd, torque);
        }

        void stop_motor(int addr)
        {
            uint8_t msg[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0Xfd};
            can_.send_cmd(addr, sizeof(msg), msg);
        }

        void reset_position(int addr)
        {
            set_motor_param(addr, 0.0, 0.0, 0.0, 0.0, 0.0);
            uint8_t msg[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0Xfe};
            can_.send_cmd(addr, sizeof(msg), msg);
        }
        

        void get_data(std::vector<float> &data)
        {
            can_.can_read();
            unpack_reply();
            data[0] = position_;
            data[1] = speed_;
            data[2] = torque_;
            data[3] = motor_temp_;
            data[4] = error_code_;
        }



    private:
        CanCommunication can_;
        AK60_6 param_;
        float position_;
        float speed_;
        float torque_;
        float motor_temp_;
        uint8_t error_code_;

        int map_range(float x, float in_min, float in_max, float out_min, float out_max)
        {
            if (x > in_max)
                return out_max;
            else if (x < in_min)
                return out_min;
            else
                return (int) (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
            
        }

        int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
        {
            /// Converts a float to an unsigned int, given range and number of bits ///
            float span = x_max - x_min;
            
            if(x < x_min) x = x_min;
            else if(x > x_max) x = x_max;
            return (int) ((x-x_min)*((float) ((1<<bits)/span)));
        }

        float uint_to_float(int x_int, float x_min, float x_max, int bits)
        {
            /// converts unsigned int to float, given range and number of bits ///
            float span = x_max - x_min;
            float offset = x_min;
            return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;     
        }

        void unpack_reply()
        {
            // for (int i=0; i<sizeof(can_.frame.data); i++)
            // {
            //     printf("data[%d] = %d\r\n", i, can_.frame.data[i]);
            // }
            /// unpack ints from can buffer ///
            int id = can_.frame.data[0]; //驱动 ID 号
            int p_int = (can_.frame.data[1]<<8)|can_.frame.data[2]; //Motor position data
            int v_int = (can_.frame.data[3]<<4)|(can_.frame.data[4]>>4); // Motor speed data
            int i_int = ((can_.frame.data[4]&0xF)<<8)|can_.frame.data[5]; // Motor torque data
            
            /// convert ints to floats ///
            float p = uint_to_float(p_int, param_.P_min, param_.P_max, 16);
            float v = uint_to_float(v_int, param_.V_min, param_.V_max, 12);
            float t = uint_to_float(i_int, param_.T_min, param_.T_max, 12);
            
            
            if(id == 1 || id == 0)
            {
                position_ = p; //Read the corresponding data according to the ID code
                speed_ = v;
                torque_ = t;
                motor_temp_ = (float) can_.frame.data[6];
                error_code_ = can_.frame.data[7];
                //std::cout << position << " " << speed << " " << torque << " " << std::endl;
            }
        }

        void pack_cmd(int addr, float p_des, float v_des, float kp, float kd, float t_ff)
        {
            //std::cout << p_des << v_des << kp << kd << t_ff << std::endl;
            /// limit data to be within bounds ///
            float p_des_, v_des_, kp_, kd_, t_ff_;
            
            p_des_ = fminf(fmaxf(param_.P_min, p_des), param_.P_max);
            v_des_ = fminf(fmaxf(param_.V_min, v_des), param_.V_max);
            kp_ = fminf(fmaxf(param_.Kp_min, kp), param_.Kp_max);
            kd_ = fminf(fmaxf(param_.Kd_min, kd), param_.Kd_max);
            t_ff_ = fminf(fmaxf(param_.T_min, t_ff), param_.T_max);
            /// convert floats to unsigned ints ///
            int p_int = float_to_uint(p_des_, param_.P_min, param_.P_max, 16);
            int v_int = float_to_uint(v_des_, param_.V_min, param_.V_max, 12);
            int kp_int = float_to_uint(kp_, param_.Kp_min, param_.Kp_max, 12);
            int kd_int = float_to_uint(kd_, param_.Kd_min, param_.Kd_max, 12);
            int t_int = float_to_uint(t_ff_, param_.T_min, param_.T_max, 12);

            //std::cout << p_int << "\n" << v_int << "\n" << kp_int << "\n" << kd_int << "\n" << t_int << std::endl;
            /// pack ints into the can buffer ///
            uint8_t msg[8];
            msg[0] = p_int>>8; // Position 8 higher
            msg[1] = p_int&0xFF; // Position 8 lower
            msg[2] = v_int>>4; // Speed 8 higher
            msg[3] = ((v_int&0xF)<<4)|(kp_int>>8); //Speed 4 bit lower KP 4bit higher
            msg[4] = kp_int&0xFF; // KP 8 bit lower
            msg[5] = kd_int>>4; // Kd 8 bit higher
            msg[6] = ((kd_int&0xF)<<4)|(t_int>>8); //KP 4 bit lower torque 4 bit higher https://www.cubemars.com/ 51 / 52
            msg[7] = t_int&0xff; // torque 4 bit lower
            
            can_.send_cmd(addr, sizeof(msg), msg);

            }

        


};


#endif // CUBEMARS_AK_SERIES_DRIVER__AK_DRIVER_HPP_