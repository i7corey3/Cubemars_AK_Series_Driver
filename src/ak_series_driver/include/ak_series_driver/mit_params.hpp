/*
structs that contain the parameters of each type of motor, as well as the error
code definitions for the AK-series TMotor actuators.
You could use an optional torque model that accounts for friction losses if one is available.
So far, such a model is only available for the AK80-9.

This model comes from a linear regression with the following constants:
    - a_hat[0] = bias
    - a_hat[1] = standard torque constant multiplier
    - a_hat[2] = nonlinear torque constant multiplier
    - a_hat[3] = coloumb friction
    - a_hat[4] = gearbox friction

The model has the form:
τ = a_hat[0] + gr*(a_hat[1]*kt - a_hat[2]*abs(i))*i - (v/(ϵ + np.abs(v)) )*(a_hat[3] + a_hat[4]*np.abs(i))

with the following values:
    - τ = approximated torque
    - gr = gear ratio
    - kt = nominal torque constant
    - i = current
    - v = velocity
    - ϵ = signum velocity threshold
*/


#ifndef CUBEMARS_AK_SERIES_DRIVER__MIT_PARAMS_HPP_
#define CUBEMARS_AK_SERIES_DRIVER__MIT_PARAMS_HPP_

#define ERROR_CODES(c) (c == 0 ? ("No Error"): c == 1 ? "Over temperature fault" : c == 2 ? "Over current fault" : c == 3 ? "Over voltage fault" : c == 4 ? "Under voltage fault" : c == 5 ? "Encoder fault" : c == 6 ? "Phase current unbalance fault (The hardware may be damaged)" : 0)
                        
struct AK80_9
{
    float P_min = -12.5;
    float P_max = 12.5;
    float V_min = -50.0;
    float V_max = 50.0;
    float T_min = -18.0;
    float T_max = 18.0;
    float Kp_min = 0.0;
    float Kp_max = 500.0;
    float Kd_min = 0.0;
    float Kd_max = 5.0;
    float Kt_TMotor = 0.091;
    float Current_Factor = 0.59;
    float Kt_Actual = 0.115;
    int Gear_Ratio = 9;
    bool Use_derived_torque_constants = true;
    float a_hat[5] = {0.0, 1.15605006e+00, 4.17389589e-04, 2.68556072e-01, 4.90424140e-02};
};

struct AK10_9
{
    float P_min = -12.5;
    float P_max = 12.5;
    float V_min = -50.0;
    float V_max = 50.0;
    float T_min = -65.0;
    float T_max = 65.0;
    float Kp_min = 0.0;
    float Kp_max = 500.0;
    float Kd_min = 0.0;
    float Kd_max = 5.0;
    float Kt_TMotor = 0.16;
    float Current_Factor = 0.59;
    float Kt_Actual = 0.16;
    int Gear_Ratio = 9;
    bool Use_derived_torque_constants = false;
    float a_hat[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
};

struct AK60_6
{
    float P_min = -12.5;
    float P_max = 12.5;
    float V_min = -50.0;
    float V_max = 50.0;
    float T_min = -15.0;
    float T_max = 15.0;
    float Kp_min = 0.0;
    float Kp_max = 500.0;
    float Kd_min = 0.0;
    float Kd_max = 5.0;
    float Kt_TMotor = 0.068;
    float Current_Factor = 0.59;
    float Kt_Actual = 0.087;
    int Gear_Ratio = 9;
    bool Use_derived_torque_constants = false;
    float a_hat[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
};

struct AK70_10
{
    float P_min = -12.5;
    float P_max = 12.5;
    float V_min = -50.0;
    float V_max = 50.0;
    float T_min = -25.0;
    float T_max = 25.0;
    float Kp_min = 0.0;
    float Kp_max = 500.0;
    float Kd_min = 0.0;
    float Kd_max = 5.0;
    float Kt_TMotor = 0.095;
    float Current_Factor = 0.59;
    float Kt_Actual = 0.122;
    int Gear_Ratio = 10;
    bool Use_derived_torque_constants = false;
    float a_hat[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
};

struct AK80_6
{
    float P_min = -12.5;
    float P_max = 12.5;
    float V_min = -76.0;
    float V_max = 76.0;
    float T_min = -12.0;
    float T_max = 12.0;
    float Kp_min = 0.0;
    float Kp_max = 500.0;
    float Kd_min = 0.0;
    float Kd_max = 5.0;
    float Kt_TMotor = 0.091;
    float Current_Factor = 0.59;
    float Kt_Actual = 0.017;
    int Gear_Ratio = 6;
    bool Use_derived_torque_constants = false;
    float a_hat[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
};

struct AK80_64
{
    float P_min = -12.5;
    float P_max = 12.5;
    float V_min = -8.0;
    float V_max = 8.0;
    float T_min = -144.0;
    float T_max = 144.0;
    float Kp_min = 0.0;
    float Kp_max = 500.0;
    float Kd_min = 0.0;
    float Kd_max = 5.0;
    float Kt_TMotor = 0.119;
    float Current_Factor = 0.59;
    float Kt_Actual = 0.153;
    int Gear_Ratio = 80;
    bool Use_derived_torque_constants = false;
    float a_hat[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
};


#endif //CUBEMARS_AK_SERIES_DRIVER__MIT_PARAMS_HPP_