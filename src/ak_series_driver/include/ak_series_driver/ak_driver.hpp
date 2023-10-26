#ifndef CUBEMARS_AK_SERIES_DRIVER__AK_DRIVER_HPP_
#define CUBEMARS_AK_SERIES_DRIVER__AK_DRIVER_HPP_

#include "ak_series_driver/can_communication.hpp"
#include "ak_series_driver/mit_params.hpp"
#include <cmath>
#include <vector>

#define maxRawPosition 65535
#define maxRawVelocity 4095
#define maxRawTorque 4095
#define maxRawKp 4095
#define maxRawKd 4095
#define maxRawCurrent 4095
#define set_zero_sleep 1.5

class AKDriver
{

    public:
        
        void setup(int channel, int bitrate)
        {
            can_.start(channel, bitrate);
            uint8_t msg[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0Xfc};
            can_.send_cmd(1, sizeof(msg), msg);
            reset_position();
            
            

        }

        void set_motor_param(float position, float velocity, float torque, float Kp, float Kd)
        {
            pack_cmd(position, velocity, torque, Kp, Kd);
        }

        void stop_motor()
        {
            uint8_t msg[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0Xfd};
            can_.send_cmd(1, sizeof(msg), msg);
        }

        void reset_position()
        {
            uint8_t msg[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0Xfe};
            can_.send_cmd(1, sizeof(msg), msg);
        }
        

        void get_data(std::vector<float> &data)
        {
            can_.can_read();
            unpack_reply();
            data[0] = position_;
            data[1] = speed_;
            data[2] = torque_;
        }



    private:
        CanCommunication can_;
        AK60_6 param_;
        float position_;
        float speed_;
        float torque_;


        int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
        {
            /// Converts a float to an unsigned int, given range and number of bits ///
            float span = x_max - x_min;
            if(x < x_min) x = x_min;
            else if(x > x_max) x = x_max;
            return (int) ((x- x_min)*((float)((1<<bits)/span)));
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
            
            /// unpack ints from can buffer ///
            int id = can_.frame.data[0]; //驱动 ID 号
            int p_int = (can_.frame.data[1]<<8)|can_.frame.data[2]; //Motor position data
            int v_int = (can_.frame.data[3]<<4)|(can_.frame.data[4]>>4); // Motor speed data
            int i_int = ((can_.frame.data[4]&0xF)<<8)|can_.frame.data[5]; // Motor torque data
            /// convert ints to floats ///
            float p = uint_to_float(p_int, param_.P_min, param_.P_max, 16);
            float v = uint_to_float(v_int, param_.V_min, param_.V_max, 12);
            float i = uint_to_float(i_int, -param_.T_min, param_.T_max, 12);
            if(id == 1){
            position_ = p; //Read the corresponding data according to the ID code
            speed_ = v;
            torque_ = i;
            //std::cout << position << " " << speed << " " << torque << " " << std::endl;
            }
        }

        void pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff)
        {
            /// limit data to be within bounds ///
            
            p_des = fminf(fmaxf(param_.P_min, p_des), param_.P_max);
            v_des = fminf(fmaxf(param_.V_min, v_des), param_.V_max);
            kp = fminf(fmaxf(param_.Kp_min, kp), param_.Kp_max);
            kd = fminf(fmaxf(param_.Kd_min, kd), param_.Kd_max);
            t_ff = fminf(fmaxf(param_.T_min, t_ff), param_.T_max);
            /// convert floats to unsigned ints ///
            int p_int = float_to_uint(p_des, param_.P_min, param_.P_max, 16);
            int v_int = float_to_uint(v_des, param_.V_min, param_.V_max, 12);
            int kp_int = float_to_uint(kp, param_.Kp_min, param_.Kp_max, 12);
            int kd_int = float_to_uint(kd, param_.Kd_min, param_.Kd_max, 12);
            int t_int = float_to_uint(t_ff, param_.T_min, param_.T_max, 12);
            /// pack ints into the can buffer ///
            uint8_t msg[8];
            msg[0] = p_int>>8; // Position 8 higher
            msg[1] = p_int&0xFF; // Position 8 lower
            msg[2] = v_int>>4; // Speed 8 higher
            msg[3] = ((v_int&0xF)<<4)|(kp_int>>8); //Speed 4 bit lower KP 4bit higher
            msg[4] = kp_int&0xFF; // KP 8 bit lower
            msg[5] = kd_int>>4; // Kd 8 bit higher
            msg[6] = ((kd_int&0xF)<<4)|(kp_int>>8); //KP 4 bit lower torque 4 bit higher https://www.cubemars.com/ 51 / 52
            msg[7] = t_int&0xff; // torque 4 bit lower
            
            can_.send_cmd(1, sizeof(msg), msg);

            }

        


};


#endif // CUBEMARS_AK_SERIES_DRIVER__AK_DRIVER_HPP_