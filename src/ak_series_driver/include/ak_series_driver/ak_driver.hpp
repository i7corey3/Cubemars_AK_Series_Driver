#ifndef CUBEMARS_AK_SERIES_DRIVER__AK_DRIVER_HPP_
#define CUBEMARS_AK_SERIES_DRIVER__AK_DRIVER_HPP_

#include "ak_series_driver/can_communication.hpp"
#include "ak_series_driver/mit_params.hpp"
#include "ak_series_driver/motor.hpp"
#include <cmath>
#include <vector>

typedef enum {
    CAN_PACKET_SET_DUTY = 0, // Duty Cycle Mode
    CAN_PACKET_SET_CURRENT, // Current Loop Mode
    CAN_PACKET_SET_CURRENT_BRAKE, // Current Brake Mode
    CAN_PACKET_SET_RPM, // Speed Mode
    CAN_PACKET_SET_POS, // Position Mode
    CAN_PACKET_SET_ORIGIN_HERE, // Set Origin Mode
    CAN_PACKET_SET_POS_SPD, // Position-Speed Loop Mod
} CAN_PACKET_ID;

class AKDriver
{

    public:
        Motor motor1_;
        
        void setup(int channel, int bitrate)
        {
            can_.start(channel, bitrate);
        }

        void setup_motor(std::string name, int can_id, double angle_offset, double gear_ratio)
        {
            motor1_.setup(name, can_id, angle_offset, gear_ratio);
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

        void move_to_position(int addr, float pos)
        {
            
            if (std::abs(motor1_.angle - pos) < 12.5)
            {
                set_motor_param(addr, 0, 3.14, 0, 3, 3);
                start_motor(addr);
            }
            while (position_ > pos + 0.2 || position_ < pos - 0.2)
            {
               
                // else 
                // {
                //     double final_val = motor1_.get_local_position();
                    
                //     set_motor_param(addr, (float) final_val, 0, 0, 3, 3);
                //     start_motor(addr);
                //     break;
                // }
                // motor1_.get_position(position_, pos, motor1_.angle);
                
                
            }
        }

        void comm_can_set_pos(uint8_t controller_id, float pos) {
            int32_t send_index = 0;
            uint8_t buffer[4];
            buffer_append_int32(buffer, (int32_t)(pos), &send_index);
            comm_can_transmit_eid(controller_id |
            ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
        }
        void comm_can_set_pos_spd(uint8_t controller_id, float pos, 
                                    int16_t spd, int16_t RPA ) {
            int32_t send_index = 0;
            int16_t send_index1 = 4;
            uint8_t buffer[8];
            buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
            buffer_append_int16(buffer,spd/ 10.0, &send_index1);
            buffer_append_int16(buffer,RPA/10.0, &send_index1);
            // std::cout << "buffer " << std::endl;
            // for (auto i : buffer) {
            //     printf(std::to_string(i).c_str());
            //     std::cout << std::endl;
            // }
            // std::cout << send_index1;
            // std::cout << std::endl;
           
            comm_can_transmit_eid(controller_id |
            ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index1);
            
        }
        void comm_can_set_rpm(uint8_t controller_id, float rpm) {
            int32_t send_index = 0;
            uint8_t buffer[4];
            buffer_append_int32(buffer, (int32_t)rpm, &send_index);
            //std::cout << "buffer " << std::endl;
            // for (auto i : buffer) {
            //     printf("%X", std::to_string(i).c_str());
            //     std::cout << std::endl;
            // }
            
            // std::cout << std::endl;
            std::cout << "canID" << std::hex << static_cast<int>(controller_id |
            ((uint32_t)CAN_PACKET_SET_RPM << 8)) << std::endl;
            comm_can_transmit_eid(controller_id |
            ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
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
        
        void comm_can_transmit_eid(uint32_t id, const uint8_t *data, 
                                    uint8_t len) {
            
            if (len > 8)
            {
                len = 8;
            }
            //std::cout << std::hex << id << std::endl;
             //printf("%s", std::to_string(len).c_str());
             //std::cout << std::endl;
            uint8_t msg[64];
            for (uint8_t i = 0; i < len; i++)
            {
                msg[i] = data[i];
                //std::cout << std::hex << data[i] << " ";
            }
            //std::cout << std::endl;
            
            
            can_.send_cmd(id, len, msg); //CAN port sends TxMessage data
        }
        
        void buffer_append_int16(uint8_t* buffer, int16_t number, 
                                 int16_t *index) {
            buffer[(*index)++] = number >> 8;
            buffer[(*index)++] = number;
        }
        void buffer_append_int32(uint8_t* buffer, int32_t number, 
                                    int32_t *index) {
            buffer[(*index)++] = number >> 24;
            buffer[(*index)++] = number >> 16;
            buffer[(*index)++] = number >> 8;
            buffer[(*index)++] = number;
            for (int i = 0; i < sizeof(buffer); i++)
            {
                std::cout << std::hex << static_cast<int>(buffer[i]) << " " << std::endl;
            }
        }
        
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
            // int c = (int) ((x-x_min)*((float) ((1<<bits)/span)));
            // std::string s = std::to_string(c);
            
            // printf("%02X ", s.c_str());
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