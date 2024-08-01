#ifndef CUBEMARS_AK_SERIES_DRIVER__AK_MOTOR_HPP_
#define CUBEMARS_AK_SERIES_DRIVER__AK_MOTOR_HPP_

#include <cmath>
#include <vector>
#include <string.h>

class Motor
{

    public:
        std::string name = "";
        int can_id = 0;
        double angle = 0.0;
        double angle_offset = 0.0;
        double gear_ratio = 0.0;
        
        
        
        Motor() = default;

        void setup(std::string &name_, int &can_id_, double &angle_offset_, 
        double &gear_ratio_)
        {
            name_ = name;
            can_id_ = can_id;
            angle_offset_ = angle_offset;
            gear_ratio_ = gear_ratio;
      
        }

        double get_local_position()
        {
            return angle/rotation_count;
        }

        void get_position(float position, float desired_position, double &angle)
        {
            if (current_position < desired_position)
            {
                if (rotation_count % 2 == 0)
                {
                    if (position < prev_position) rotation_count++;
                    current_position = position * rotation_count + angle_offset;
                    current_position *= gear_ratio;
                }
                else
                {
                    if (position > prev_position) rotation_count++;
                    current_position = position * rotation_count + angle_offset;
                    current_position *= gear_ratio;
                }
            }
            else
            {
                if (rotation_count % 2 == 0)
                {
                    if (position > prev_position) rotation_count--;
                    current_position = position * rotation_count + angle_offset;
                    current_position *= gear_ratio;
                }
                else
                {
                    if (position < prev_position) rotation_count--;
                    current_position = position * rotation_count + angle_offset;
                    current_position *= gear_ratio;
                }
            }

            prev_position = position;
            angle = current_position;      
        }

    private:
        int rotation_count = 0;
        double current_position = 0.0;
        double prev_position = 0.0;

        
    


};


#endif // CUBEMARS_AK_SERIES_DRIVER__AK_MOTOR_HPP_