/*
Copyright(c) 2015-2026 Panos Karabelas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

//= INCLUDES ===============================
#pragma once

#ifdef DEBUG
    #define _DEBUG 1
    #undef NDEBUG
#else
    #define NDEBUG 1
    #undef _DEBUG
#endif
#define PX_PHYSX_STATIC_LIB
#include <physx/PxPhysicsAPI.h>
#include <vector>
#include "../Logging/Log.h"
#include "../Core/Engine.h"
#include "../../editor/ImGui/Source/imgui.h"
//==========================================

namespace car
{
    using namespace physx;

    // maximum number of gears (reverse + neutral + up to 9 forward)
    static constexpr int max_gears = 11;

    // car specification - defines all physical characteristics of a specific car
    // the simulation reads from the active spec stored in tuning::spec
    // create presets by writing functions that return a populated car_spec
    // there are no default values - every preset must define every field
    struct car_spec
    {
        const char* name;

        // engine
        float engine_idle_rpm;
        float engine_redline_rpm;
        float engine_max_rpm;
        float engine_peak_torque;
        float engine_peak_torque_rpm;
        float engine_inertia;
        float engine_friction;
        float engine_rpm_smoothing;
        float downshift_blip_amount;
        float downshift_blip_duration;

        // gearbox (indices: 0=reverse, 1=neutral, 2=1st, 3=2nd, ...)
        float gear_ratios[max_gears];
        int   gear_count;
        float final_drive;
        float shift_up_rpm;
        float shift_down_rpm;
        float shift_time;
        float clutch_engagement_rate;
        float drivetrain_efficiency;
        bool  manual_transmission;

        // shift speed thresholds (indexed by gear, km/h)
        float upshift_speed_base[max_gears];
        float upshift_speed_sport[max_gears];
        float downshift_speeds[max_gears];

        // brakes
        float brake_force;
        float brake_bias_front;
        float reverse_power_ratio;
        float brake_ambient_temp;
        float brake_optimal_temp;
        float brake_fade_temp;
        float brake_max_temp;
        float brake_heat_coefficient;
        float brake_cooling_base;
        float brake_cooling_airflow;
        float brake_thermal_mass;

        // input
        float throttle_smoothing;

        // pacejka magic formula coefficients
        float lat_B;
        float lat_C;
        float lat_D;
        float lat_E;
        float long_B;
        float long_C;
        float long_D;
        float long_E;

        // load-dependent stiffness
        float load_B_scale_min;

        // pneumatic trail model
        float pneumatic_trail_max;
        float pneumatic_trail_peak;

        // tire grip
        float tire_friction;
        float min_slip_speed;
        float load_sensitivity;
        float load_reference;
        float rear_grip_ratio;
        float slip_angle_deadband;
        float min_lateral_grip;
        float camber_thrust_coeff;
        float max_slip_angle;

        // tire thermals
        float tire_ambient_temp;
        float tire_optimal_temp;
        float tire_temp_range;
        float tire_heat_from_slip;
        float tire_heat_from_rolling;
        float tire_cooling_rate;
        float tire_cooling_airflow;
        float tire_grip_temp_factor;
        float tire_min_temp;
        float tire_max_temp;
        float tire_relaxation_length;
        float tire_wear_rate;
        float tire_wear_heat_mult;
        float tire_grip_wear_loss;

        // suspension
        float front_spring_freq;
        float rear_spring_freq;
        float damping_ratio;
        float damping_bump_ratio;
        float damping_rebound_ratio;
        float front_arb_stiffness;
        float rear_arb_stiffness;
        float max_susp_force;
        float max_damper_velocity;
        float bump_stop_stiffness;
        float bump_stop_threshold;

        // aerodynamics
        float rolling_resistance;
        float drag_coeff;
        float frontal_area;
        float lift_coeff_front;
        float lift_coeff_rear;
        bool  drs_enabled;
        float drs_rear_cl_factor;
        float side_area;
        bool  ground_effect_enabled;
        float ground_effect_multiplier;
        float ground_effect_height_ref;
        float ground_effect_height_max;
        bool  yaw_aero_enabled;
        float yaw_drag_multiplier;
        float yaw_side_force_coeff;
        bool  pitch_aero_enabled;
        float pitch_sensitivity;
        float aero_center_height;
        float aero_center_front_z;
        float aero_center_rear_z;

        // center of mass
        float center_of_mass_x;
        float center_of_mass_y;
        float center_of_mass_z;

        // steering
        float max_steer_angle;
        float high_speed_steer_reduction;
        float steering_rate;
        float self_align_gain;
        float steering_linearity;

        // alignment (radians)
        float front_camber;
        float rear_camber;
        float front_toe;
        float rear_toe;
        float front_bump_steer;
        float rear_bump_steer;

        // wheels
        float airborne_wheel_decay;
        float bearing_friction;
        float ground_match_rate;
        float handbrake_sliding_factor;
        float handbrake_torque;

        // differential (0 = open, 1 = locked, 2 = lsd)
        float lsd_preload;
        float lsd_lock_ratio_accel;
        float lsd_lock_ratio_decel;
        int   diff_type;

        // input behavior
        float input_deadzone;
        float steering_deadzone;
        float braking_speed_threshold;

        // speed limits
        float max_forward_speed;
        float max_reverse_speed;
        float max_power_reduction;

        // damping
        float linear_damping;
        float angular_damping;

        // abs
        bool  abs_enabled;
        float abs_slip_threshold;
        float abs_release_rate;
        float abs_pulse_frequency;

        // traction control
        bool  tc_enabled;
        float tc_slip_threshold;
        float tc_power_reduction;
        float tc_response_rate;

        // turbo
        bool  turbo_enabled;
        float boost_max_pressure;
        float boost_spool_rate;
        float boost_wastegate_rpm;
        float boost_torque_mult;
        float boost_min_rpm;
    };

    //= car presets ================================================================

    // ferrari laferrari - 6.3l v12 hybrid hypercar, 700 Nm, 7-speed dct, rwd
    inline car_spec preset_laferrari()
    {
        car_spec s = {};
        s.name = "LaFerrari";

        // engine - 6.3l v12 + hy-kers electric motor
        s.engine_idle_rpm         = 1000.0f;
        s.engine_redline_rpm      = 9250.0f;
        s.engine_max_rpm          = 9500.0f;
        s.engine_peak_torque      = 700.0f;
        s.engine_peak_torque_rpm  = 6750.0f;
        s.engine_inertia          = 0.25f;
        s.engine_friction         = 0.02f;
        s.engine_rpm_smoothing    = 6.0f;
        s.downshift_blip_amount   = 0.35f;
        s.downshift_blip_duration = 0.15f;

        // gearbox - 7-speed dual-clutch (f1 derived)
        s.gear_ratios[0]          = -3.15f;  // reverse
        s.gear_ratios[1]          =  0.0f;   // neutral
        s.gear_ratios[2]          =  3.08f;  // 1st
        s.gear_ratios[3]          =  2.19f;  // 2nd
        s.gear_ratios[4]          =  1.63f;  // 3rd
        s.gear_ratios[5]          =  1.29f;  // 4th
        s.gear_ratios[6]          =  1.03f;  // 5th
        s.gear_ratios[7]          =  0.84f;  // 6th
        s.gear_ratios[8]          =  0.69f;  // 7th
        s.gear_count              = 9;
        s.final_drive             = 4.44f;
        s.shift_up_rpm            = 8500.0f;
        s.shift_down_rpm          = 3500.0f;
        s.shift_time              = 0.08f;
        s.clutch_engagement_rate  = 20.0f;
        s.drivetrain_efficiency   = 0.88f;
        s.manual_transmission     = false;

        // shift speed thresholds calibrated for final_drive 4.44
        float up_base[]  = { 0, 0, 60, 85, 115, 150, 190, 230, 0, 0, 0 };
        float up_sport[] = { 0, 0, 80, 115, 155, 195, 245, 300, 0, 0, 0 };
        float down[]     = { 0, 0, 0, 30, 55, 80, 110, 150, 200, 0, 0 };
        for (int i = 0; i < max_gears; i++)
        {
            s.upshift_speed_base[i]  = up_base[i];
            s.upshift_speed_sport[i] = up_sport[i];
            s.downshift_speeds[i]    = down[i];
        }

        // brakes - carbon-ceramic
        s.brake_force            = 12000.0f;
        s.brake_bias_front       = 0.65f;
        s.reverse_power_ratio    = 0.5f;
        s.brake_ambient_temp     = 200.0f;
        s.brake_optimal_temp     = 400.0f;
        s.brake_fade_temp        = 700.0f;
        s.brake_max_temp         = 900.0f;
        s.brake_heat_coefficient = 0.015f;
        s.brake_cooling_base     = 8.0f;
        s.brake_cooling_airflow  = 1.5f;
        s.brake_thermal_mass     = 5.0f;

        // input
        s.throttle_smoothing = 10.0f;

        // pacejka - michelin pilot sport cup 2 compound
        s.lat_B  = 12.0f;
        s.lat_C  = 1.4f;
        s.lat_D  = 1.0f;
        s.lat_E  = 0.6f;
        s.long_B = 20.0f;
        s.long_C = 1.5f;
        s.long_D = 1.0f;
        s.long_E = -0.5f;

        s.load_B_scale_min    = 0.5f;
        s.pneumatic_trail_max  = 0.04f;
        s.pneumatic_trail_peak = 0.08f;

        // tire grip
        s.tire_friction       = 1.5f;
        s.min_slip_speed      = 0.5f;
        s.load_sensitivity    = 0.92f;
        s.load_reference      = 4000.0f;
        s.rear_grip_ratio     = 1.10f;
        s.slip_angle_deadband = 0.01f;
        s.min_lateral_grip    = 0.4f;
        s.camber_thrust_coeff = 0.015f;
        s.max_slip_angle      = 0.40f;

        // tire thermals
        s.tire_ambient_temp      = 50.0f;
        s.tire_optimal_temp      = 90.0f;
        s.tire_temp_range        = 50.0f;
        s.tire_heat_from_slip    = 25.0f;
        s.tire_heat_from_rolling = 0.15f;
        s.tire_cooling_rate      = 2.0f;
        s.tire_cooling_airflow   = 0.05f;
        s.tire_grip_temp_factor  = 0.15f;
        s.tire_min_temp          = 10.0f;
        s.tire_max_temp          = 150.0f;
        s.tire_relaxation_length = 0.3f;
        s.tire_wear_rate         = 0.00001f;
        s.tire_wear_heat_mult    = 2.0f;
        s.tire_grip_wear_loss    = 0.3f;

        // suspension - adaptive magnetorheological dampers
        s.front_spring_freq     = 2.2f;
        s.rear_spring_freq      = 2.0f;
        s.damping_ratio         = 0.70f;
        s.damping_bump_ratio    = 0.7f;
        s.damping_rebound_ratio = 1.3f;
        s.front_arb_stiffness   = 3500.0f;
        s.rear_arb_stiffness    = 1500.0f;
        s.max_susp_force        = 35000.0f;
        s.max_damper_velocity   = 5.0f;
        s.bump_stop_stiffness   = 100000.0f;
        s.bump_stop_threshold   = 0.9f;

        // aerodynamics - active aero flaps, flat underbody
        s.rolling_resistance       = 0.011f;
        s.drag_coeff               = 0.35f;
        s.frontal_area             = 2.2f;
        s.lift_coeff_front         = -0.3f;
        s.lift_coeff_rear          = -0.4f;
        s.drs_enabled              = false;
        s.drs_rear_cl_factor       = 0.3f;
        s.side_area                = 4.0f;
        s.ground_effect_enabled    = true;
        s.ground_effect_multiplier = 1.5f;
        s.ground_effect_height_ref = 0.15f;
        s.ground_effect_height_max = 0.30f;
        s.yaw_aero_enabled         = true;
        s.yaw_drag_multiplier      = 2.5f;
        s.yaw_side_force_coeff     = 1.2f;
        s.pitch_aero_enabled       = true;
        s.pitch_sensitivity        = 0.5f;
        s.aero_center_height       = 0.3f;
        s.aero_center_front_z      = 0.0f;
        s.aero_center_rear_z       = 0.0f;

        // center of mass - mid-rear engine, low cg
        s.center_of_mass_x = 0.0f;
        s.center_of_mass_y = -0.15f;
        s.center_of_mass_z = -0.3f;

        // steering
        s.max_steer_angle            = 0.65f;
        s.high_speed_steer_reduction = 0.4f;
        s.steering_rate              = 1.5f;
        s.self_align_gain            = 0.5f;
        s.steering_linearity         = 1.3f;

        // alignment
        s.front_camber     = -1.5f * (3.14159265f / 180.0f);
        s.rear_camber      = -1.0f * (3.14159265f / 180.0f);
        s.front_toe        =  0.1f * (3.14159265f / 180.0f);
        s.rear_toe         =  0.2f * (3.14159265f / 180.0f);
        s.front_bump_steer = -0.02f;
        s.rear_bump_steer  =  0.01f;

        // wheels
        s.airborne_wheel_decay     = 0.99f;
        s.bearing_friction         = 0.2f;
        s.ground_match_rate        = 8.0f;
        s.handbrake_sliding_factor = 0.75f;
        s.handbrake_torque         = 5000.0f;

        // differential - e-diff (electronic lsd)
        s.lsd_preload          = 150.0f;
        s.lsd_lock_ratio_accel = 0.5f;
        s.lsd_lock_ratio_decel = 0.3f;
        s.diff_type            = 2;

        // input behavior
        s.input_deadzone          = 0.01f;
        s.steering_deadzone       = 0.001f;
        s.braking_speed_threshold = 3.0f;

        // speed limits
        s.max_forward_speed   = 320.0f;
        s.max_reverse_speed   = 80.0f;
        s.max_power_reduction = 0.85f;

        // damping
        s.linear_damping  = 0.001f;
        s.angular_damping = 0.50f;

        // abs
        s.abs_enabled         = false;
        s.abs_slip_threshold  = 0.15f;
        s.abs_release_rate    = 0.7f;
        s.abs_pulse_frequency = 15.0f;

        // traction control
        s.tc_enabled         = false;
        s.tc_slip_threshold  = 0.08f;
        s.tc_power_reduction = 0.8f;
        s.tc_response_rate   = 15.0f;

        // turbo - not applicable (naturally aspirated)
        s.turbo_enabled       = false;
        s.boost_max_pressure  = 0.0f;
        s.boost_spool_rate    = 0.0f;
        s.boost_wastegate_rpm = 0.0f;
        s.boost_torque_mult   = 0.0f;
        s.boost_min_rpm       = 0.0f;

        return s;
    }

    // porsche 911 gt3 (992) - 4.0l flat-6, 470 Nm, 7-speed pdk, rwd
    inline car_spec preset_911_gt3()
    {
        car_spec s = {};
        s.name = "Porsche 911 GT3";

        // engine - 4.0l naturally aspirated flat-6
        s.engine_idle_rpm         = 950.0f;
        s.engine_redline_rpm      = 9000.0f;
        s.engine_max_rpm          = 9200.0f;
        s.engine_peak_torque      = 470.0f;
        s.engine_peak_torque_rpm  = 6100.0f;
        s.engine_inertia          = 0.20f;
        s.engine_friction         = 0.02f;
        s.engine_rpm_smoothing    = 6.0f;
        s.downshift_blip_amount   = 0.40f;
        s.downshift_blip_duration = 0.12f;

        // gearbox - 7-speed pdk
        s.gear_ratios[0]          = -3.28f;  // reverse
        s.gear_ratios[1]          =  0.0f;   // neutral
        s.gear_ratios[2]          =  3.91f;  // 1st
        s.gear_ratios[3]          =  2.29f;  // 2nd
        s.gear_ratios[4]          =  1.58f;  // 3rd
        s.gear_ratios[5]          =  1.18f;  // 4th
        s.gear_ratios[6]          =  0.94f;  // 5th
        s.gear_ratios[7]          =  0.79f;  // 6th
        s.gear_ratios[8]          =  0.62f;  // 7th
        s.gear_count              = 9;
        s.final_drive             = 3.44f;
        s.shift_up_rpm            = 8200.0f;
        s.shift_down_rpm          = 3200.0f;
        s.shift_time              = 0.08f;
        s.clutch_engagement_rate  = 20.0f;
        s.drivetrain_efficiency   = 0.90f;
        s.manual_transmission     = false;

        // shift speed thresholds calibrated for gt3 ratios + final drive
        float up_base[]  = { 0, 0, 55, 90, 125, 165, 210, 250, 0, 0, 0 };
        float up_sport[] = { 0, 0, 75, 120, 160, 210, 255, 305, 0, 0, 0 };
        float down[]     = { 0, 0, 0, 25, 50, 80, 115, 155, 205, 0, 0 };
        for (int i = 0; i < max_gears; i++)
        {
            s.upshift_speed_base[i]  = up_base[i];
            s.upshift_speed_sport[i] = up_sport[i];
            s.downshift_speeds[i]    = down[i];
        }

        // brakes - pccb carbon-ceramic
        s.brake_force            = 10000.0f;
        s.brake_bias_front       = 0.62f;
        s.reverse_power_ratio    = 0.5f;
        s.brake_ambient_temp     = 200.0f;
        s.brake_optimal_temp     = 400.0f;
        s.brake_fade_temp        = 700.0f;
        s.brake_max_temp         = 900.0f;
        s.brake_heat_coefficient = 0.015f;
        s.brake_cooling_base     = 9.0f;
        s.brake_cooling_airflow  = 1.8f;
        s.brake_thermal_mass     = 4.5f;

        // input
        s.throttle_smoothing = 10.0f;

        // pacejka - michelin pilot sport cup 2 r
        s.lat_B  = 12.0f;
        s.lat_C  = 1.4f;
        s.lat_D  = 1.0f;
        s.lat_E  = 0.6f;
        s.long_B = 20.0f;
        s.long_C = 1.5f;
        s.long_D = 1.0f;
        s.long_E = -0.5f;

        s.load_B_scale_min     = 0.5f;
        s.pneumatic_trail_max  = 0.04f;
        s.pneumatic_trail_peak = 0.08f;

        // tire grip - cup 2 r compound, slightly grippier
        s.tire_friction       = 1.6f;
        s.min_slip_speed      = 0.5f;
        s.load_sensitivity    = 0.92f;
        s.load_reference      = 4000.0f;
        s.rear_grip_ratio     = 1.08f;
        s.slip_angle_deadband = 0.01f;
        s.min_lateral_grip    = 0.4f;
        s.camber_thrust_coeff = 0.015f;
        s.max_slip_angle      = 0.40f;

        // tire thermals
        s.tire_ambient_temp      = 50.0f;
        s.tire_optimal_temp      = 90.0f;
        s.tire_temp_range        = 50.0f;
        s.tire_heat_from_slip    = 25.0f;
        s.tire_heat_from_rolling = 0.15f;
        s.tire_cooling_rate      = 2.0f;
        s.tire_cooling_airflow   = 0.05f;
        s.tire_grip_temp_factor  = 0.15f;
        s.tire_min_temp          = 10.0f;
        s.tire_max_temp          = 150.0f;
        s.tire_relaxation_length = 0.3f;
        s.tire_wear_rate         = 0.00001f;
        s.tire_wear_heat_mult    = 2.0f;
        s.tire_grip_wear_loss    = 0.3f;

        // suspension - double wishbone front, multi-link rear, stiffer track setup
        s.front_spring_freq     = 2.8f;
        s.rear_spring_freq      = 2.5f;
        s.damping_ratio         = 0.72f;
        s.damping_bump_ratio    = 0.7f;
        s.damping_rebound_ratio = 1.3f;
        s.front_arb_stiffness   = 4000.0f;
        s.rear_arb_stiffness    = 2000.0f;
        s.max_susp_force        = 35000.0f;
        s.max_damper_velocity   = 5.0f;
        s.bump_stop_stiffness   = 100000.0f;
        s.bump_stop_threshold   = 0.9f;

        // aerodynamics - gt wing, front splitter
        s.rolling_resistance       = 0.012f;
        s.drag_coeff               = 0.39f;
        s.frontal_area             = 2.1f;
        s.lift_coeff_front         = -0.35f;
        s.lift_coeff_rear          = -0.6f;
        s.drs_enabled              = false;
        s.drs_rear_cl_factor       = 0.3f;
        s.side_area                = 3.8f;
        s.ground_effect_enabled    = true;
        s.ground_effect_multiplier = 1.2f;
        s.ground_effect_height_ref = 0.12f;
        s.ground_effect_height_max = 0.28f;
        s.yaw_aero_enabled         = true;
        s.yaw_drag_multiplier      = 2.5f;
        s.yaw_side_force_coeff     = 1.2f;
        s.pitch_aero_enabled       = true;
        s.pitch_sensitivity        = 0.5f;
        s.aero_center_height       = 0.28f;
        s.aero_center_front_z      = 0.0f;
        s.aero_center_rear_z       = 0.0f;

        // center of mass - rear engine, rear weight bias
        s.center_of_mass_x = 0.0f;
        s.center_of_mass_y = -0.12f;
        s.center_of_mass_z = -0.45f;

        // steering
        s.max_steer_angle            = 0.65f;
        s.high_speed_steer_reduction = 0.4f;
        s.steering_rate              = 1.5f;
        s.self_align_gain            = 0.5f;
        s.steering_linearity         = 1.3f;

        // alignment
        s.front_camber     = -2.0f * (3.14159265f / 180.0f);
        s.rear_camber      = -1.5f * (3.14159265f / 180.0f);
        s.front_toe        =  0.0f * (3.14159265f / 180.0f);
        s.rear_toe         =  0.3f * (3.14159265f / 180.0f);
        s.front_bump_steer = -0.02f;
        s.rear_bump_steer  =  0.01f;

        // wheels
        s.airborne_wheel_decay     = 0.99f;
        s.bearing_friction         = 0.2f;
        s.ground_match_rate        = 8.0f;
        s.handbrake_sliding_factor = 0.75f;
        s.handbrake_torque         = 5000.0f;

        // differential - mechanical lsd
        s.lsd_preload          = 120.0f;
        s.lsd_lock_ratio_accel = 0.4f;
        s.lsd_lock_ratio_decel = 0.25f;
        s.diff_type            = 2;

        // input behavior
        s.input_deadzone          = 0.01f;
        s.steering_deadzone       = 0.001f;
        s.braking_speed_threshold = 3.0f;

        // speed limits
        s.max_forward_speed   = 318.0f;
        s.max_reverse_speed   = 80.0f;
        s.max_power_reduction = 0.85f;

        // damping
        s.linear_damping  = 0.001f;
        s.angular_damping = 0.50f;

        // abs
        s.abs_enabled         = false;
        s.abs_slip_threshold  = 0.15f;
        s.abs_release_rate    = 0.7f;
        s.abs_pulse_frequency = 15.0f;

        // traction control
        s.tc_enabled         = false;
        s.tc_slip_threshold  = 0.08f;
        s.tc_power_reduction = 0.8f;
        s.tc_response_rate   = 15.0f;

        // turbo - not applicable (naturally aspirated)
        s.turbo_enabled       = false;
        s.boost_max_pressure  = 0.0f;
        s.boost_spool_rate    = 0.0f;
        s.boost_wastegate_rpm = 0.0f;
        s.boost_torque_mult   = 0.0f;
        s.boost_min_rpm       = 0.0f;

        return s;
    }

    //= preset registry ============================================================
    // mitsubishi lancer evolution ix - 2.0l turbo i4, 400 Nm, 5-speed manual, awd (simulated as rwd)
    inline car_spec preset_evo_ix()
    {
        car_spec s = {};
        s.name = "Mitsubishi Evo IX";

        // engine - 4g63 2.0l turbo inline-4
        s.engine_idle_rpm         = 850.0f;
        s.engine_redline_rpm      = 7500.0f;
        s.engine_max_rpm          = 7800.0f;
        s.engine_peak_torque      = 400.0f;
        s.engine_peak_torque_rpm  = 3500.0f;
        s.engine_inertia          = 0.30f;
        s.engine_friction         = 0.025f;
        s.engine_rpm_smoothing    = 5.0f;
        s.downshift_blip_amount   = 0.35f;
        s.downshift_blip_duration = 0.15f;

        // gearbox - 5-speed close ratio
        s.gear_ratios[0]          = -3.416f;  // reverse
        s.gear_ratios[1]          =  0.0f;    // neutral
        s.gear_ratios[2]          =  3.083f;  // 1st
        s.gear_ratios[3]          =  1.842f;  // 2nd
        s.gear_ratios[4]          =  1.233f;  // 3rd
        s.gear_ratios[5]          =  0.942f;  // 4th
        s.gear_ratios[6]          =  0.761f;  // 5th
        s.gear_ratios[7]          =  0.0f;
        s.gear_ratios[8]          =  0.0f;
        s.gear_count              = 8;  // r + n + 5 forward + 1 unused slot
        s.final_drive             = 4.529f;
        s.shift_up_rpm            = 7000.0f;
        s.shift_down_rpm          = 3000.0f;
        s.shift_time              = 0.15f;
        s.clutch_engagement_rate  = 15.0f;
        s.drivetrain_efficiency   = 0.85f;
        s.manual_transmission     = false;

        // shift speed thresholds calibrated for evo ratios + final drive
        float up_base[]  = { 0, 0, 45, 80, 120, 160, 0, 0, 0, 0, 0 };
        float up_sport[] = { 0, 0, 60, 105, 150, 195, 0, 0, 0, 0, 0 };
        float down[]     = { 0, 0, 0, 20, 45, 75, 110, 0, 0, 0, 0 };
        for (int i = 0; i < max_gears; i++)
        {
            s.upshift_speed_base[i]  = up_base[i];
            s.upshift_speed_sport[i] = up_sport[i];
            s.downshift_speeds[i]    = down[i];
        }

        // brakes - brembo 4-pot front, 2-pot rear
        s.brake_force            = 8000.0f;
        s.brake_bias_front       = 0.60f;
        s.reverse_power_ratio    = 0.5f;
        s.brake_ambient_temp     = 180.0f;
        s.brake_optimal_temp     = 350.0f;
        s.brake_fade_temp        = 650.0f;
        s.brake_max_temp         = 850.0f;
        s.brake_heat_coefficient = 0.015f;
        s.brake_cooling_base     = 8.0f;
        s.brake_cooling_airflow  = 1.2f;
        s.brake_thermal_mass     = 5.5f;

        // input
        s.throttle_smoothing = 8.0f;

        // pacejka - advan neova ad08r / re-71rs style semi-slick
        s.lat_B  = 11.0f;
        s.lat_C  = 1.35f;
        s.lat_D  = 1.0f;
        s.lat_E  = 0.5f;
        s.long_B = 18.0f;
        s.long_C = 1.45f;
        s.long_D = 1.0f;
        s.long_E = -0.4f;

        s.load_B_scale_min     = 0.5f;
        s.pneumatic_trail_max  = 0.04f;
        s.pneumatic_trail_peak = 0.09f;

        // tire grip - 265 section width, semi-slick compound
        s.tire_friction       = 1.4f;
        s.min_slip_speed      = 0.5f;
        s.load_sensitivity    = 0.90f;
        s.load_reference      = 4500.0f;
        s.rear_grip_ratio     = 1.05f;
        s.slip_angle_deadband = 0.01f;
        s.min_lateral_grip    = 0.35f;
        s.camber_thrust_coeff = 0.012f;
        s.max_slip_angle      = 0.38f;

        // tire thermals
        s.tire_ambient_temp      = 45.0f;
        s.tire_optimal_temp      = 85.0f;
        s.tire_temp_range        = 45.0f;
        s.tire_heat_from_slip    = 22.0f;
        s.tire_heat_from_rolling = 0.15f;
        s.tire_cooling_rate      = 2.0f;
        s.tire_cooling_airflow   = 0.04f;
        s.tire_grip_temp_factor  = 0.15f;
        s.tire_min_temp          = 10.0f;
        s.tire_max_temp          = 140.0f;
        s.tire_relaxation_length = 0.35f;
        s.tire_wear_rate         = 0.000012f;
        s.tire_wear_heat_mult    = 2.0f;
        s.tire_grip_wear_loss    = 0.3f;

        // suspension - macpherson front, multi-link rear, rally-stiff springs
        s.front_spring_freq     = 2.4f;
        s.rear_spring_freq      = 2.2f;
        s.damping_ratio         = 0.65f;
        s.damping_bump_ratio    = 0.7f;
        s.damping_rebound_ratio = 1.3f;
        s.front_arb_stiffness   = 3000.0f;
        s.rear_arb_stiffness    = 2500.0f;
        s.max_susp_force        = 30000.0f;
        s.max_damper_velocity   = 5.0f;
        s.bump_stop_stiffness   = 80000.0f;
        s.bump_stop_threshold   = 0.88f;

        // aerodynamics - varis bumper, big rear wing
        s.rolling_resistance       = 0.014f;
        s.drag_coeff               = 0.38f;
        s.frontal_area             = 2.2f;
        s.lift_coeff_front         = -0.10f;
        s.lift_coeff_rear          = -0.25f;
        s.drs_enabled              = false;
        s.drs_rear_cl_factor       = 0.3f;
        s.side_area                = 3.5f;
        s.ground_effect_enabled    = false;
        s.ground_effect_multiplier = 1.0f;
        s.ground_effect_height_ref = 0.12f;
        s.ground_effect_height_max = 0.30f;
        s.yaw_aero_enabled         = true;
        s.yaw_drag_multiplier      = 2.0f;
        s.yaw_side_force_coeff     = 1.0f;
        s.pitch_aero_enabled       = true;
        s.pitch_sensitivity        = 0.4f;
        s.aero_center_height       = 0.30f;
        s.aero_center_front_z      = 0.0f;
        s.aero_center_rear_z       = 0.0f;

        // center of mass - front-mid engine, front biased
        s.center_of_mass_x = 0.0f;
        s.center_of_mass_y = -0.10f;
        s.center_of_mass_z = 0.25f;

        // steering
        s.max_steer_angle            = 0.62f;
        s.high_speed_steer_reduction = 0.45f;
        s.steering_rate              = 1.6f;
        s.self_align_gain            = 0.45f;
        s.steering_linearity         = 1.2f;

        // alignment
        s.front_camber     = -1.5f * (3.14159265f / 180.0f);
        s.rear_camber      = -1.0f * (3.14159265f / 180.0f);
        s.front_toe        =  0.0f * (3.14159265f / 180.0f);
        s.rear_toe         =  0.2f * (3.14159265f / 180.0f);
        s.front_bump_steer = -0.015f;
        s.rear_bump_steer  =  0.01f;

        // wheels
        s.airborne_wheel_decay     = 0.99f;
        s.bearing_friction         = 0.2f;
        s.ground_match_rate        = 8.0f;
        s.handbrake_sliding_factor = 0.80f;
        s.handbrake_torque         = 4500.0f;

        // differential - ayc/acd simulated as mechanical lsd
        s.lsd_preload          = 100.0f;
        s.lsd_lock_ratio_accel = 0.45f;
        s.lsd_lock_ratio_decel = 0.20f;
        s.diff_type            = 2;

        // input behavior
        s.input_deadzone          = 0.01f;
        s.steering_deadzone       = 0.001f;
        s.braking_speed_threshold = 3.0f;

        // speed limits
        s.max_forward_speed   = 255.0f;
        s.max_reverse_speed   = 60.0f;
        s.max_power_reduction = 0.85f;

        // damping
        s.linear_damping  = 0.001f;
        s.angular_damping = 0.55f;

        // abs
        s.abs_enabled         = false;
        s.abs_slip_threshold  = 0.15f;
        s.abs_release_rate    = 0.7f;
        s.abs_pulse_frequency = 12.0f;

        // traction control
        s.tc_enabled         = false;
        s.tc_slip_threshold  = 0.10f;
        s.tc_power_reduction = 0.7f;
        s.tc_response_rate   = 12.0f;

        // turbo - td05h-16g
        s.turbo_enabled       = true;
        s.boost_max_pressure  = 1.5f;
        s.boost_spool_rate    = 3.0f;
        s.boost_wastegate_rpm = 6500.0f;
        s.boost_torque_mult   = 0.25f;
        s.boost_min_rpm       = 3000.0f;

        return s;
    }

    // function pointer type for preset factories
    typedef car_spec (*preset_factory_fn)();

    struct preset_entry
    {
        const char*        name;
        preset_factory_fn  factory;
    };

    // add new presets here - the osd combo box picks them up automatically
    inline preset_entry preset_registry[] =
    {
        { "LaFerrari",       preset_laferrari },
        { "Porsche 911 GT3", preset_911_gt3   },
        { "Mitsubishi Evo IX", preset_evo_ix  },
    };

    inline constexpr int preset_count = sizeof(preset_registry) / sizeof(preset_registry[0]);
    inline int active_preset_index    = 0;

    //= tuning namespace ===========================================================

    // swap active car spec at runtime
    inline void load_car(const car_spec& new_spec);

    namespace tuning
    {
        // active car specification - swap this to change the car
        inline car_spec spec = preset_laferrari();

        // engine
        inline const float& engine_idle_rpm         = spec.engine_idle_rpm;
        inline const float& engine_redline_rpm      = spec.engine_redline_rpm;
        inline const float& engine_max_rpm          = spec.engine_max_rpm;
        inline const float& engine_peak_torque      = spec.engine_peak_torque;
        inline const float& engine_peak_torque_rpm  = spec.engine_peak_torque_rpm;
        inline const float& engine_inertia          = spec.engine_inertia;
        inline const float& engine_friction         = spec.engine_friction;
        inline const float& engine_rpm_smoothing    = spec.engine_rpm_smoothing;
        inline const float& downshift_blip_amount   = spec.downshift_blip_amount;
        inline const float& downshift_blip_duration = spec.downshift_blip_duration;

        // gearbox
        inline auto&        gear_ratios             = spec.gear_ratios;
        inline const int&   gear_count              = spec.gear_count;
        inline const float& final_drive             = spec.final_drive;
        inline const float& shift_up_rpm            = spec.shift_up_rpm;
        inline const float& shift_down_rpm          = spec.shift_down_rpm;
        inline const float& shift_time              = spec.shift_time;
        inline const float& clutch_engagement_rate  = spec.clutch_engagement_rate;
        inline const float& drivetrain_efficiency   = spec.drivetrain_efficiency;
        inline bool&        manual_transmission     = spec.manual_transmission;

        // brakes
        inline const float& brake_force             = spec.brake_force;
        inline const float& brake_bias_front        = spec.brake_bias_front;
        inline const float& reverse_power_ratio     = spec.reverse_power_ratio;
        inline const float& brake_ambient_temp      = spec.brake_ambient_temp;
        inline const float& brake_optimal_temp      = spec.brake_optimal_temp;
        inline const float& brake_fade_temp         = spec.brake_fade_temp;
        inline const float& brake_max_temp          = spec.brake_max_temp;
        inline const float& brake_heat_coefficient  = spec.brake_heat_coefficient;
        inline const float& brake_cooling_base      = spec.brake_cooling_base;
        inline const float& brake_cooling_airflow   = spec.brake_cooling_airflow;
        inline const float& brake_thermal_mass      = spec.brake_thermal_mass;

        // input
        inline const float& throttle_smoothing      = spec.throttle_smoothing;

        // pacejka
        inline const float& lat_B              = spec.lat_B;
        inline const float& lat_C              = spec.lat_C;
        inline const float& lat_D              = spec.lat_D;
        inline const float& lat_E              = spec.lat_E;
        inline const float& long_B             = spec.long_B;
        inline const float& long_C             = spec.long_C;
        inline const float& long_D             = spec.long_D;
        inline const float& long_E             = spec.long_E;
        inline const float& load_B_scale_min   = spec.load_B_scale_min;
        inline const float& pneumatic_trail_max  = spec.pneumatic_trail_max;
        inline const float& pneumatic_trail_peak = spec.pneumatic_trail_peak;

        // tire grip
        inline const float& tire_friction       = spec.tire_friction;
        inline const float& min_slip_speed      = spec.min_slip_speed;
        inline const float& load_sensitivity    = spec.load_sensitivity;
        inline const float& load_reference      = spec.load_reference;
        inline const float& rear_grip_ratio     = spec.rear_grip_ratio;
        inline const float& slip_angle_deadband = spec.slip_angle_deadband;
        inline const float& min_lateral_grip    = spec.min_lateral_grip;
        inline const float& camber_thrust_coeff = spec.camber_thrust_coeff;
        inline const float& max_slip_angle      = spec.max_slip_angle;

        // tire thermals
        inline const float& tire_ambient_temp      = spec.tire_ambient_temp;
        inline const float& tire_optimal_temp      = spec.tire_optimal_temp;
        inline const float& tire_temp_range        = spec.tire_temp_range;
        inline const float& tire_heat_from_slip    = spec.tire_heat_from_slip;
        inline const float& tire_heat_from_rolling = spec.tire_heat_from_rolling;
        inline const float& tire_cooling_rate      = spec.tire_cooling_rate;
        inline const float& tire_cooling_airflow   = spec.tire_cooling_airflow;
        inline const float& tire_grip_temp_factor  = spec.tire_grip_temp_factor;
        inline const float& tire_min_temp          = spec.tire_min_temp;
        inline const float& tire_max_temp          = spec.tire_max_temp;
        inline const float& tire_relaxation_length = spec.tire_relaxation_length;
        inline const float& tire_wear_rate         = spec.tire_wear_rate;
        inline const float& tire_wear_heat_mult    = spec.tire_wear_heat_mult;
        inline const float& tire_grip_wear_loss    = spec.tire_grip_wear_loss;

        // suspension
        inline const float& front_spring_freq     = spec.front_spring_freq;
        inline const float& rear_spring_freq      = spec.rear_spring_freq;
        inline const float& damping_ratio         = spec.damping_ratio;
        inline const float& damping_bump_ratio    = spec.damping_bump_ratio;
        inline const float& damping_rebound_ratio = spec.damping_rebound_ratio;
        inline const float& front_arb_stiffness   = spec.front_arb_stiffness;
        inline const float& rear_arb_stiffness    = spec.rear_arb_stiffness;
        inline const float& max_susp_force        = spec.max_susp_force;
        inline const float& max_damper_velocity   = spec.max_damper_velocity;
        inline const float& bump_stop_stiffness   = spec.bump_stop_stiffness;
        inline const float& bump_stop_threshold   = spec.bump_stop_threshold;

        // aerodynamics - mutable references for runtime aero adjustments
        inline const float& rolling_resistance      = spec.rolling_resistance;
        inline float& drag_coeff                    = spec.drag_coeff;
        inline float& frontal_area                  = spec.frontal_area;
        inline float& lift_coeff_front              = spec.lift_coeff_front;
        inline float& lift_coeff_rear               = spec.lift_coeff_rear;
        inline bool&  drs_enabled                   = spec.drs_enabled;
        inline float& drs_rear_cl_factor            = spec.drs_rear_cl_factor;
        inline float& side_area                     = spec.side_area;
        inline bool&  ground_effect_enabled         = spec.ground_effect_enabled;
        inline float& ground_effect_multiplier      = spec.ground_effect_multiplier;
        inline float& ground_effect_height_ref      = spec.ground_effect_height_ref;
        inline float& ground_effect_height_max      = spec.ground_effect_height_max;
        inline bool&  yaw_aero_enabled              = spec.yaw_aero_enabled;
        inline float& yaw_drag_multiplier           = spec.yaw_drag_multiplier;
        inline float& yaw_side_force_coeff          = spec.yaw_side_force_coeff;
        inline bool&  pitch_aero_enabled            = spec.pitch_aero_enabled;
        inline float& pitch_sensitivity             = spec.pitch_sensitivity;
        inline float& aero_center_height            = spec.aero_center_height;
        inline float& aero_center_front_z           = spec.aero_center_front_z;
        inline float& aero_center_rear_z            = spec.aero_center_rear_z;

        // center of mass
        inline float& center_of_mass_x = spec.center_of_mass_x;
        inline float& center_of_mass_y = spec.center_of_mass_y;
        inline float& center_of_mass_z = spec.center_of_mass_z;

        // steering
        inline const float& max_steer_angle            = spec.max_steer_angle;
        inline const float& high_speed_steer_reduction = spec.high_speed_steer_reduction;
        inline const float& steering_rate              = spec.steering_rate;
        inline const float& self_align_gain            = spec.self_align_gain;
        inline const float& steering_linearity         = spec.steering_linearity;

        // alignment
        inline const float& front_camber     = spec.front_camber;
        inline const float& rear_camber      = spec.rear_camber;
        inline const float& front_toe        = spec.front_toe;
        inline const float& rear_toe         = spec.rear_toe;
        inline const float& front_bump_steer = spec.front_bump_steer;
        inline const float& rear_bump_steer  = spec.rear_bump_steer;

        // wheels
        inline const float& airborne_wheel_decay     = spec.airborne_wheel_decay;
        inline const float& bearing_friction         = spec.bearing_friction;
        inline const float& ground_match_rate        = spec.ground_match_rate;
        inline const float& handbrake_sliding_factor = spec.handbrake_sliding_factor;
        inline const float& handbrake_torque         = spec.handbrake_torque;

        // differential
        inline const float& lsd_preload          = spec.lsd_preload;
        inline const float& lsd_lock_ratio_accel = spec.lsd_lock_ratio_accel;
        inline const float& lsd_lock_ratio_decel = spec.lsd_lock_ratio_decel;
        inline int&         diff_type            = spec.diff_type;

        // input
        inline const float& input_deadzone          = spec.input_deadzone;
        inline const float& steering_deadzone       = spec.steering_deadzone;
        inline const float& braking_speed_threshold = spec.braking_speed_threshold;

        // speed limits
        inline const float& max_forward_speed   = spec.max_forward_speed;
        inline const float& max_reverse_speed   = spec.max_reverse_speed;
        inline const float& max_power_reduction = spec.max_power_reduction;

        // damping
        inline const float& linear_damping  = spec.linear_damping;
        inline const float& angular_damping = spec.angular_damping;

        // abs
        inline bool&        abs_enabled        = spec.abs_enabled;
        inline const float& abs_slip_threshold = spec.abs_slip_threshold;
        inline const float& abs_release_rate   = spec.abs_release_rate;
        inline const float& abs_pulse_frequency = spec.abs_pulse_frequency;

        // traction control
        inline bool&        tc_enabled         = spec.tc_enabled;
        inline const float& tc_slip_threshold  = spec.tc_slip_threshold;
        inline const float& tc_power_reduction = spec.tc_power_reduction;
        inline const float& tc_response_rate   = spec.tc_response_rate;

        // turbo
        inline bool&        turbo_enabled       = spec.turbo_enabled;
        inline const float& boost_max_pressure  = spec.boost_max_pressure;
        inline const float& boost_spool_rate    = spec.boost_spool_rate;
        inline const float& boost_wastegate_rpm = spec.boost_wastegate_rpm;
        inline const float& boost_torque_mult   = spec.boost_torque_mult;
        inline const float& boost_min_rpm       = spec.boost_min_rpm;

        // simulation-level parameters (not part of car spec)
        constexpr float air_density                  = 1.225f;
        constexpr float road_bump_amplitude          = 0.002f;
        constexpr float road_bump_frequency          = 0.5f;
        constexpr float surface_friction_asphalt     = 1.0f;
        constexpr float surface_friction_concrete    = 0.95f;
        constexpr float surface_friction_wet_asphalt = 0.7f;
        constexpr float surface_friction_gravel      = 0.6f;
        constexpr float surface_friction_grass       = 0.4f;
        constexpr float surface_friction_ice         = 0.1f;

        // debug
        inline bool draw_raycasts   = true;
        inline bool draw_suspension = true;
        inline bool log_pacejka     = false;
        inline bool log_telemetry   = false;
        inline bool log_to_file     = false;
    }

    inline void load_car(const car_spec& new_spec) { tuning::spec = new_spec; }
    
    struct aero_debug_data
    {
        PxVec3 position         = PxVec3(0);
        PxVec3 velocity         = PxVec3(0);
        PxVec3 drag_force       = PxVec3(0);
        PxVec3 front_downforce  = PxVec3(0);
        PxVec3 rear_downforce   = PxVec3(0);
        PxVec3 side_force       = PxVec3(0);
        PxVec3 front_aero_pos   = PxVec3(0);
        PxVec3 rear_aero_pos    = PxVec3(0);
        float  ride_height      = 0.0f;
        float  yaw_angle        = 0.0f;
        float  ground_effect_factor = 1.0f;
        bool   valid            = false;
    };
    inline static aero_debug_data aero_debug;

    // stored shape data for visualization (2D projections of convex hull)
    struct shape_2d
    {
        std::vector<std::pair<float, float>> side_profile;   // (z, y) points for side view
        std::vector<std::pair<float, float>> front_profile;  // (x, y) points for front view
        float min_x = 0, max_x = 0;
        float min_y = 0, max_y = 0;
        float min_z = 0, max_z = 0;
        bool valid = false;
    };
    
    // function-local static for odr safety
    inline shape_2d& shape_data_ref()
    {
        static shape_2d instance;
        return instance;
    }

    enum wheel_id { front_left = 0, front_right = 1, rear_left = 2, rear_right = 3, wheel_count = 4 };
    inline constexpr const char* wheel_names[] = { "FL", "FR", "RL", "RR" };
    enum surface_type { surface_asphalt = 0, surface_concrete, surface_wet_asphalt, surface_gravel, surface_grass, surface_ice, surface_count };
    
    struct config
    {
        float length            = 4.5f;
        float width             = 2.0f;
        float height            = 0.5f;
        float mass              = 1500.0f;
        float wheel_radius      = 0.35f;
        float wheel_width       = 0.25f;
        float wheel_mass        = 20.0f;
        float suspension_travel = 0.20f;
        float suspension_height = 0.35f;
    };
    
    struct wheel
    {
        float        compression          = 0.0f;
        float        target_compression   = 0.0f;
        float        prev_compression     = 0.0f;
        float        compression_velocity = 0.0f;
        bool         grounded             = false;
        PxVec3       contact_point        = PxVec3(0);
        PxVec3       contact_normal       = PxVec3(0, 1, 0);
        float        angular_velocity     = 0.0f;
        float        rotation             = 0.0f;
        float        tire_load            = 0.0f;
        float        slip_angle           = 0.0f;
        float        slip_ratio           = 0.0f;
        float        lateral_force        = 0.0f;
        float        longitudinal_force   = 0.0f;
        float        temperature          = tuning::tire_ambient_temp;
        float        brake_temp           = tuning::brake_ambient_temp;
        float        wear                 = 0.0f;
        surface_type contact_surface      = surface_asphalt;
    };
    
    struct input_state
    {
        float throttle  = 0.0f;
        float brake     = 0.0f;
        float steering  = 0.0f;
        float handbrake = 0.0f;
    };

    inline static PxRigidDynamic* body     = nullptr;
    inline static PxMaterial*     material = nullptr;
    inline static config          cfg;
    inline static wheel           wheels[wheel_count];
    inline static input_state     input;
    inline static input_state     input_target;
    inline static PxVec3          wheel_offsets[wheel_count];
    inline static float           wheel_moi[wheel_count];
    inline static float           spring_stiffness[wheel_count];
    inline static float           spring_damping[wheel_count];
    inline static float           abs_phase               = 0.0f;
    inline static bool            abs_active[wheel_count] = {};
    inline static float           tc_reduction            = 0.0f;
    inline static bool            tc_active               = false;
    inline static float           engine_rpm              = tuning::engine_idle_rpm;
    inline static int             current_gear            = 2;
    inline static float           shift_timer             = 0.0f;
    inline static bool            is_shifting             = false;
    inline static float           clutch                  = 1.0f;
    inline static float           shift_cooldown          = 0.0f;
    inline static int             last_shift_direction    = 0;
    inline static float           redline_hold_timer      = 0.0f;
    inline static float           boost_pressure          = 0.0f;
    inline static float           downshift_blip_timer    = 0.0f;
    inline static bool            drs_active              = false;
    inline static float           longitudinal_accel      = 0.0f;
    inline static float           road_bump_phase         = 0.0f;
    inline static PxVec3          prev_velocity           = PxVec3(0);
    
    struct debug_ray
    {
        PxVec3 origin;
        PxVec3 hit_point;
        bool   hit;
    };
    constexpr int debug_rays_per_wheel = 7;
    inline static debug_ray debug_rays[wheel_count][debug_rays_per_wheel];
    inline static PxVec3    debug_suspension_top[wheel_count];
    inline static PxVec3    debug_suspension_bottom[wheel_count];

    inline bool  is_front(int i)                { return i == front_left || i == front_right; }
    inline bool  is_rear(int i)                 { return i == rear_left || i == rear_right; }
    inline float lerp(float a, float b, float t){ return a + (b - a) * t; }
    inline float exp_decay(float rate, float dt){ return 1.0f - expf(-rate * dt); }
    
    inline float pacejka(float slip, float B, float C, float D, float E)
    {
        float Bx = B * slip;
        return D * sinf(C * atanf(Bx - E * (Bx - atanf(Bx))));
    }
    
    inline float load_sensitive_grip(float load)
    {
        if (load <= 0.0f) return 0.0f;
        return load * powf(load / tuning::load_reference, tuning::load_sensitivity - 1.0f);
    }
    
    inline float get_tire_temp_grip_factor(float temperature)
    {
        float penalty = PxClamp(fabsf(temperature - tuning::tire_optimal_temp) / tuning::tire_temp_range, 0.0f, 1.0f);
        return 1.0f - penalty * tuning::tire_grip_temp_factor;
    }
    
    inline float get_camber_grip_factor(int wheel_index, float slip_angle)
    {
        float camber = is_front(wheel_index) ? tuning::front_camber : tuning::rear_camber;
        float effective_camber = camber - slip_angle * 0.3f;
        return 1.0f - fabsf(effective_camber) * 0.1f;
    }
    
    inline float get_surface_friction(surface_type surface)
    {
        static constexpr float friction[] = {
            tuning::surface_friction_asphalt,
            tuning::surface_friction_concrete,
            tuning::surface_friction_wet_asphalt,
            tuning::surface_friction_gravel,
            tuning::surface_friction_grass,
            tuning::surface_friction_ice
        };
        return (surface >= 0 && surface < surface_count) ? friction[surface] : 1.0f;
    }
    
    inline float get_brake_efficiency(float temp)
    {
        if (temp >= tuning::brake_fade_temp)
            return 0.6f;
        
        if (temp < tuning::brake_optimal_temp)
        {
            float t = PxClamp((temp - tuning::brake_ambient_temp) / (tuning::brake_optimal_temp - tuning::brake_ambient_temp), 0.0f, 1.0f);
            return 0.85f + 0.15f * t;
        }
        
        float t = (temp - tuning::brake_optimal_temp) / (tuning::brake_fade_temp - tuning::brake_optimal_temp);
        return 1.0f - 0.4f * t;
    }
    
    inline void update_boost(float throttle, float rpm, float dt)
    {
        if (!tuning::turbo_enabled)
        {
            boost_pressure = lerp(boost_pressure, 0.0f, exp_decay(tuning::boost_spool_rate * 3.0f, dt));
            return;
        }
        
        float target = 0.0f;
        if (throttle > 0.3f && rpm > tuning::boost_min_rpm)
        {
            target = tuning::boost_max_pressure * PxMin((rpm - tuning::boost_min_rpm) / 4000.0f, 1.0f);
            
            if (rpm > tuning::boost_wastegate_rpm)
                target *= PxMax(0.0f, 1.0f - (rpm - tuning::boost_wastegate_rpm) / 2000.0f);
        }
        
        float rate = (target > boost_pressure) ? tuning::boost_spool_rate : tuning::boost_spool_rate * 2.0f;
        boost_pressure = lerp(boost_pressure, target, exp_decay(rate, dt));
    }
    
    inline float get_engine_torque(float rpm)
    {
        rpm = PxClamp(rpm, tuning::engine_idle_rpm, tuning::engine_max_rpm);
        
        float factor;
        if (rpm < 2500.0f)
            factor = 0.55f + ((rpm - tuning::engine_idle_rpm) / 1500.0f) * 0.15f;
        else if (rpm < 4500.0f)
            factor = 0.70f + ((rpm - 2500.0f) / 2000.0f) * 0.15f;
        else if (rpm < tuning::engine_peak_torque_rpm)
            factor = 0.85f + ((rpm - 4500.0f) / (tuning::engine_peak_torque_rpm - 4500.0f)) * 0.15f;
        else if (rpm < tuning::engine_redline_rpm)
        {
            // single smooth falloff from peak to redline avoids slope discontinuity
            float t = (rpm - tuning::engine_peak_torque_rpm) / (tuning::engine_redline_rpm - tuning::engine_peak_torque_rpm);
            factor = 1.0f - t * t * 0.20f;
        }
        else
            factor = 0.80f * (1.0f - ((rpm - tuning::engine_redline_rpm) / (tuning::engine_max_rpm - tuning::engine_redline_rpm)) * 0.8f);
        
        return tuning::engine_peak_torque * factor;
    }
    
    inline float wheel_rpm_to_engine_rpm(float wheel_rpm, int gear)
    {
        if (gear < 0 || gear >= tuning::gear_count || gear == 1)
            return tuning::engine_idle_rpm;
        return fabsf(wheel_rpm * tuning::gear_ratios[gear] * tuning::final_drive);
    }
    
    inline float get_upshift_speed(int from_gear, float throttle)
    {
        if (from_gear < 2 || from_gear >= tuning::spec.gear_count - 1) return 999.0f;
        float t = PxClamp((throttle - 0.3f) / 0.5f, 0.0f, 1.0f);
        return tuning::spec.upshift_speed_base[from_gear] + t * (tuning::spec.upshift_speed_sport[from_gear] - tuning::spec.upshift_speed_base[from_gear]);
    }
    
    inline float get_downshift_speed(int gear)
    {
        return (gear >= 2 && gear < tuning::spec.gear_count) ? tuning::spec.downshift_speeds[gear] : 0.0f;
    }
    
    inline void update_automatic_gearbox(float dt, float throttle, float forward_speed)
    {
        if (shift_cooldown > 0.0f)
            shift_cooldown -= dt;
        
        if (is_shifting)
        {
            shift_timer -= dt;
            if (shift_timer <= 0.0f)
            {
                is_shifting = false;
                shift_timer = 0.0f;
                shift_cooldown = 0.5f;
            }
            return;
        }
        
        if (tuning::manual_transmission)
            return;
        
        float speed_kmh = forward_speed * 3.6f;
        
        // reverse
        if (forward_speed < -1.0f && input.brake > 0.1f && throttle < 0.1f && current_gear != 0)
        {
            current_gear = 0;
            is_shifting = true;
            shift_timer = tuning::shift_time * 2.0f;
            last_shift_direction = -1;
            return;
        }

        // neutral to first: clutch engagement, no shift delay
        if (current_gear == 1 && throttle > 0.1f && forward_speed >= -0.5f)
        {
            current_gear = 2;
            last_shift_direction = 1;
            return;
        }

        // reverse to first
        if (current_gear == 0)
        {
            if ((throttle > 0.1f && forward_speed > -2.0f) || forward_speed > 0.5f)
            {
                current_gear = 2;
                is_shifting = true;
                shift_timer = tuning::shift_time * 2.0f;
                last_shift_direction = 1;
                return;
            }
        }

        // forward gears
        if (current_gear >= 2)
        {
            bool can_shift = shift_cooldown <= 0.0f;
            
            float upshift_threshold = get_upshift_speed(current_gear, throttle);
            if (last_shift_direction == -1)
                upshift_threshold += 10.0f;
            
            bool speed_trigger = speed_kmh > upshift_threshold;
            bool rpm_trigger   = engine_rpm > tuning::shift_up_rpm;

            // track how long the engine has been sitting at redline
            if (engine_rpm > tuning::shift_up_rpm)
                redline_hold_timer += dt;
            else
                redline_hold_timer = 0.0f;

            // force upshift after 0.5s at redline despite wheelspin
            if (rpm_trigger && !speed_trigger)
            {
                // gear-scaled slip threshold
                float slip_threshold = (current_gear <= 3) ? 0.50f : 0.25f;

                float avg_slip = 0.0f;
                int grounded_count = 0;
                for (int i = rear_left; i <= rear_right; i++)
                {
                    if (wheels[i].grounded)
                    {
                        avg_slip += fabsf(wheels[i].slip_ratio);
                        grounded_count++;
                    }
                }
                if (grounded_count > 0)
                    avg_slip /= (float)grounded_count;

                // block upshift during wheelspin, but not past the redline timer
                if (avg_slip > slip_threshold && redline_hold_timer < 0.5f)
                    rpm_trigger = false;
            }

            if (can_shift && (speed_trigger || rpm_trigger) && current_gear < 8 && throttle > 0.1f)
            {
                current_gear++;
                is_shifting = true;
                shift_timer = tuning::shift_time;
                last_shift_direction = 1;
                return;
            }
            
            float downshift_threshold = get_downshift_speed(current_gear);
            if (last_shift_direction == 1)
                downshift_threshold -= 10.0f;

            if (can_shift && speed_kmh < downshift_threshold && current_gear > 2)
            {
                current_gear--;
                is_shifting = true;
                shift_timer = tuning::shift_time;
                last_shift_direction = -1;
                downshift_blip_timer = tuning::downshift_blip_duration;
                return;
            }

            // kickdown: only from cruise (below peak torque, no wheelspin)
            if (can_shift && throttle > 0.9f && current_gear > 2 && engine_rpm < tuning::engine_peak_torque_rpm)
            {
                // check for wheelspin before allowing kickdown
                float avg_slip = 0.0f;
                int grounded = 0;
                for (int i = rear_left; i <= rear_right; i++)
                {
                    if (wheels[i].grounded)
                    {
                        avg_slip += fabsf(wheels[i].slip_ratio);
                        grounded++;
                    }
                }
                if (grounded > 0)
                    avg_slip /= (float)grounded;

                if (avg_slip < 0.15f)
                {
                    int target = current_gear;
                    for (int g = current_gear - 1; g >= 2; g--)
                    {
                        float ratio = fabsf(tuning::gear_ratios[g]) * tuning::final_drive;
                        float potential_rpm = (forward_speed / cfg.wheel_radius) * (60.0f / (2.0f * PxPi)) * ratio;
                        if (potential_rpm < tuning::shift_up_rpm * 0.85f)
                            target = g;
                        else
                            break;
                    }

                    if (target < current_gear)
                    {
                        current_gear = target;
                        is_shifting = true;
                        shift_timer = tuning::shift_time;
                        last_shift_direction = -1;
                        downshift_blip_timer = tuning::downshift_blip_duration;
                    }
                }
            }
        }
    }
    
    inline const char* get_gear_string()
    {
        static const char* names[] = { "R", "N", "1", "2", "3", "4", "5", "6", "7" };
        return (current_gear >= 0 && current_gear < tuning::gear_count) ? names[current_gear] : "?";
    }

    inline void compute_constants()
    {
        float front_z = cfg.length * 0.35f;
        float rear_z  = -cfg.length * 0.35f;
        float half_w  = cfg.width * 0.5f - cfg.wheel_width * 0.5f;
        float y       = -cfg.suspension_height;
        
        wheel_offsets[front_left]  = PxVec3(-half_w, y, front_z);
        wheel_offsets[front_right] = PxVec3( half_w, y, front_z);
        wheel_offsets[rear_left]   = PxVec3(-half_w, y, rear_z);
        wheel_offsets[rear_right]  = PxVec3( half_w, y, rear_z);
        
        float axle_mass[2] = { cfg.mass * 0.40f * 0.5f, cfg.mass * 0.60f * 0.5f };
        float freq[2]      = { tuning::front_spring_freq, tuning::rear_spring_freq };
        
        for (int i = 0; i < wheel_count; i++)
        {
            int axle   = is_front(i) ? 0 : 1;
            float mass = axle_mass[axle];
            float omega = 2.0f * PxPi * freq[axle];
            
            wheel_moi[i]        = 0.7f * cfg.wheel_mass * cfg.wheel_radius * cfg.wheel_radius;
            spring_stiffness[i] = mass * omega * omega;
            spring_damping[i]   = 2.0f * tuning::damping_ratio * sqrtf(spring_stiffness[i] * mass);
        }
    }
    
    inline void destroy()
    {
        if (body)    { body->release();     body = nullptr; }
        if (material){ material->release(); material = nullptr; }
    }

    inline void compute_aero_from_shape(const std::vector<PxVec3>& vertices)
    {
        if (vertices.size() < 4)
            return;

        PxVec3 min_pt(FLT_MAX, FLT_MAX, FLT_MAX);
        PxVec3 max_pt(-FLT_MAX, -FLT_MAX, -FLT_MAX);

        for (const PxVec3& v : vertices)
        {
            min_pt.x = PxMin(min_pt.x, v.x);
            min_pt.y = PxMin(min_pt.y, v.y);
            min_pt.z = PxMin(min_pt.z, v.z);
            max_pt.x = PxMax(max_pt.x, v.x);
            max_pt.y = PxMax(max_pt.y, v.y);
            max_pt.z = PxMax(max_pt.z, v.z);
        }

        float width  = max_pt.x - min_pt.x;
        float height = max_pt.y - min_pt.y;
        float length = max_pt.z - min_pt.z;

        float frontal_fill_factor = 0.82f;
        float computed_frontal_area = width * height * frontal_fill_factor;

        float side_fill_factor = 0.75f;
        float computed_side_area = length * height * side_fill_factor;

        float length_height_ratio = length / PxMax(height, 0.1f);
        float base_cd = 0.32f;
        float ratio_factor = PxClamp(2.5f / length_height_ratio, 0.8f, 1.3f);
        float computed_drag_coeff = base_cd * ratio_factor;

        if (computed_frontal_area > 0.5f && computed_frontal_area < 10.0f)
        {
            tuning::frontal_area = computed_frontal_area;
            SP_LOG_INFO("aero: frontal area = %.2f m²", computed_frontal_area);
        }

        if (computed_side_area > 1.0f && computed_side_area < 20.0f)
        {
            tuning::side_area = computed_side_area;
            SP_LOG_INFO("aero: side area = %.2f m²", computed_side_area);
        }

        if (computed_drag_coeff > 0.2f && computed_drag_coeff < 0.6f)
        {
            tuning::drag_coeff = computed_drag_coeff;
            SP_LOG_INFO("aero: drag coefficient = %.3f", computed_drag_coeff);
        }

        float centroid_y = 0.0f;
        float centroid_z = 0.0f;
        float front_area = 0.0f;
        float rear_area = 0.0f;
        float mid_z = (min_pt.z + max_pt.z) * 0.5f;

        for (const PxVec3& v : vertices)
        {
            float h = v.y - min_pt.y;
            float weight = h * h;
            centroid_y += v.y * weight;
            centroid_z += v.z * weight;

            if (v.z > mid_z)
                front_area += weight;
            else
                rear_area += weight;
        }

        float total_weight = 0.0f;
        for (const PxVec3& v : vertices)
        {
            float h = v.y - min_pt.y;
            total_weight += h * h;
        }

        if (total_weight > 0.0f)
        {
            centroid_y /= total_weight;
            centroid_z /= total_weight;
        }

        tuning::aero_center_height = centroid_y;

        float total_area = front_area + rear_area;
        float front_bias = (total_area > 0.0f) ? front_area / total_area : 0.5f;

        tuning::aero_center_front_z = max_pt.z * 0.8f;
        tuning::aero_center_rear_z = min_pt.z * 0.8f;

        float base_lift = (tuning::lift_coeff_front + tuning::lift_coeff_rear) * 0.5f;
        tuning::lift_coeff_front = base_lift * (0.5f + (front_bias - 0.5f) * 0.5f);
        tuning::lift_coeff_rear = base_lift * (0.5f + (0.5f - front_bias) * 0.5f);

        SP_LOG_INFO("aero: dimensions %.2f x %.2f x %.2f m (L x W x H)", length, width, height);
        SP_LOG_INFO("aero: center height=%.2f, front_z=%.2f, rear_z=%.2f",
            tuning::aero_center_height, tuning::aero_center_front_z, tuning::aero_center_rear_z);
        SP_LOG_INFO("aero: front/rear bias=%.0f%%/%.0f%%, lift F/R=%.2f/%.2f",
            front_bias * 100.0f, (1.0f - front_bias) * 100.0f, tuning::lift_coeff_front, tuning::lift_coeff_rear);

        // compute 2D silhouette profiles for visualization
        // this preserves concave regions like the cabin dip between hood and roof
        shape_2d& sd = shape_data_ref();
        sd.min_x = min_pt.x; sd.max_x = max_pt.x;
        sd.min_y = min_pt.y; sd.max_y = max_pt.y;
        sd.min_z = min_pt.z; sd.max_z = max_pt.z;
        
        // compute 2D convex hull using graham scan algorithm
        // projects 3D convex hull vertices to 2D and computes proper convex outline
        auto compute_hull_2d = [](std::vector<std::pair<float, float>> points) -> std::vector<std::pair<float, float>>
        {
            if (points.size() < 3)
                return points;
            
            // find the bottom-most point (lowest y, then leftmost x as tiebreaker)
            size_t pivot_idx = 0;
            for (size_t i = 1; i < points.size(); i++)
            {
                if (points[i].second < points[pivot_idx].second ||
                    (points[i].second == points[pivot_idx].second && points[i].first < points[pivot_idx].first))
                {
                    pivot_idx = i;
                }
            }
            std::swap(points[0], points[pivot_idx]);
            auto pivot = points[0];
            
            // cross product for orientation
            auto cross = [](const std::pair<float,float>& o, const std::pair<float,float>& a, const std::pair<float,float>& b) -> float
            {
                return (a.first - o.first) * (b.second - o.second) - (a.second - o.second) * (b.first - o.first);
            };
            
            // sort by polar angle relative to pivot
            std::sort(points.begin() + 1, points.end(), [&](const auto& a, const auto& b)
            {
                float c = cross(pivot, a, b);
                if (fabsf(c) < 1e-9f)
                {
                    // collinear: keep the farther point
                    float da = (a.first - pivot.first) * (a.first - pivot.first) + (a.second - pivot.second) * (a.second - pivot.second);
                    float db = (b.first - pivot.first) * (b.first - pivot.first) + (b.second - pivot.second) * (b.second - pivot.second);
                    return da < db;
                }
                return c > 0;
            });
            
            // graham scan - build convex hull
            std::vector<std::pair<float, float>> hull;
            for (const auto& pt : points)
            {
                while (hull.size() > 1 && cross(hull[hull.size()-2], hull[hull.size()-1], pt) <= 0)
                    hull.pop_back();
                hull.push_back(pt);
            }
            
            return hull;
        };
        
        // side view: project convex hull vertices to (z, y) plane
        std::vector<std::pair<float, float>> side_points;
        side_points.reserve(vertices.size());
        for (const PxVec3& v : vertices)
            side_points.push_back({v.z, v.y});
        sd.side_profile = compute_hull_2d(std::move(side_points));
        
        // front view: project convex hull vertices to (x, y) plane
        std::vector<std::pair<float, float>> front_points;
        front_points.reserve(vertices.size());
        for (const PxVec3& v : vertices)
            front_points.push_back({v.x, v.y});
        sd.front_profile = compute_hull_2d(std::move(front_points));
        
        sd.valid = sd.side_profile.size() >= 3 && sd.front_profile.size() >= 3;
    }

    struct setup_params
    {
        PxPhysics*              physics      = nullptr;
        PxScene*                scene        = nullptr;
        PxConvexMesh*           chassis_mesh = nullptr;  // convex hull for collision
        std::vector<PxVec3>     vertices;                // original mesh verts for aero calculation
        config                  car_config;
    };

    inline bool setup(const setup_params& params)
    {
        if (!params.physics || !params.scene)
            return false;

        cfg = params.car_config;
        compute_constants();

        for (int i = 0; i < wheel_count; i++)
        {
            wheels[i] = wheel();
            abs_active[i] = false;
        }
        input = input_state();
        input_target = input_state();
        abs_phase = 0.0f;
        tc_reduction = 0.0f;
        tc_active = false;
        engine_rpm = tuning::engine_idle_rpm;
        current_gear = 2;
        shift_timer = 0.0f;
        is_shifting = false;
        clutch = 1.0f;
        shift_cooldown = 0.0f;
        last_shift_direction = 0;
        boost_pressure = 0.0f;
        downshift_blip_timer = 0.0f;
        drs_active = false;
        longitudinal_accel = 0.0f;
        road_bump_phase = 0.0f;
        prev_velocity = PxVec3(0);

        material = params.physics->createMaterial(0.8f, 0.7f, 0.1f);
        if (!material)
            return false;

        float front_mass_per_wheel = cfg.mass * 0.40f * 0.5f;
        float front_omega = 2.0f * PxPi * tuning::front_spring_freq;
        float front_stiffness = front_mass_per_wheel * front_omega * front_omega;
        float expected_sag = PxClamp((front_mass_per_wheel * 9.81f) / front_stiffness, 0.0f, cfg.suspension_travel * 0.8f);
        float spawn_y = cfg.wheel_radius + cfg.suspension_height + expected_sag;

        body = params.physics->createRigidDynamic(PxTransform(PxVec3(0, spawn_y, 0)));
        if (!body)
        {
            material->release();
            material = nullptr;
            return false;
        }

        // attach chassis shape
        if (params.chassis_mesh)
        {
            PxConvexMeshGeometry geometry(params.chassis_mesh);
            PxShape* shape = params.physics->createShape(geometry, *material);
            if (shape)
            {
                shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);
                shape->setFlag(PxShapeFlag::eVISUALIZATION, true);
                body->attachShape(*shape);
                shape->release();
            }
        }
        else
        {
            PxShape* chassis = params.physics->createShape(
                PxBoxGeometry(cfg.width * 0.5f, cfg.height * 0.5f, cfg.length * 0.5f),
                *material
            );
            if (chassis)
            {
                chassis->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);
                body->attachShape(*chassis);
                chassis->release();
            }
        }

        PxVec3 com(tuning::center_of_mass_x, tuning::center_of_mass_y, tuning::center_of_mass_z);
        PxRigidBodyExt::setMassAndUpdateInertia(*body, cfg.mass, &com);
        body->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
        body->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
        body->setLinearDamping(tuning::linear_damping);
        body->setAngularDamping(tuning::angular_damping);

        params.scene->addActor(*body);

        if (!params.vertices.empty())
            compute_aero_from_shape(params.vertices);

        SP_LOG_INFO("car setup complete: mass=%.0f kg", cfg.mass);
        return true;
    }

    inline bool set_chassis(PxConvexMesh* mesh, const std::vector<PxVec3>& vertices, PxPhysics* physics)
    {
        if (!body || !physics)
            return false;

        PxU32 shape_count = body->getNbShapes();
        if (shape_count > 0)
        {
            std::vector<PxShape*> shapes(shape_count);
            body->getShapes(shapes.data(), shape_count);
            for (PxShape* shape : shapes)
                body->detachShape(*shape);
        }

        if (mesh && material)
        {
            PxConvexMeshGeometry geometry(mesh);
            PxShape* shape = physics->createShape(geometry, *material);
            if (shape)
            {
                shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);
                shape->setFlag(PxShapeFlag::eVISUALIZATION, true);
                body->attachShape(*shape);
                shape->release();
            }
        }

        PxVec3 com(tuning::center_of_mass_x, tuning::center_of_mass_y, tuning::center_of_mass_z);
        PxRigidBodyExt::setMassAndUpdateInertia(*body, cfg.mass, &com);

        if (!vertices.empty())
            compute_aero_from_shape(vertices);

        return true;
    }

    inline void update_mass_properties()
    {
        if (!body)
            return;
        
        PxVec3 com(tuning::center_of_mass_x, tuning::center_of_mass_y, tuning::center_of_mass_z);
        PxRigidBodyExt::setMassAndUpdateInertia(*body, cfg.mass, &com);
        
        SP_LOG_INFO("car center of mass set to (%.2f, %.2f, %.2f)", com.x, com.y, com.z);
    }
    
    inline void set_center_of_mass(float x, float y, float z)
    {
        tuning::center_of_mass_x = x;
        tuning::center_of_mass_y = y;
        tuning::center_of_mass_z = z;
        update_mass_properties();
    }
    
    inline void set_center_of_mass_x(float x) { tuning::center_of_mass_x = x; update_mass_properties(); }
    inline void set_center_of_mass_y(float y) { tuning::center_of_mass_y = y; update_mass_properties(); }
    inline void set_center_of_mass_z(float z) { tuning::center_of_mass_z = z; update_mass_properties(); }
    
    inline float get_center_of_mass_x() { return tuning::center_of_mass_x; }
    inline float get_center_of_mass_y() { return tuning::center_of_mass_y; }
    inline float get_center_of_mass_z() { return tuning::center_of_mass_z; }

    inline float get_frontal_area()     { return tuning::frontal_area; }
    inline float get_side_area()        { return tuning::side_area; }
    inline float get_drag_coeff()       { return tuning::drag_coeff; }
    inline float get_lift_coeff_front() { return tuning::lift_coeff_front; }
    inline float get_lift_coeff_rear()  { return tuning::lift_coeff_rear; }
    
    inline void set_frontal_area(float area)   { tuning::frontal_area = area; }
    inline void set_side_area(float area)      { tuning::side_area = area; }
    inline void set_drag_coeff(float cd)       { tuning::drag_coeff = cd; }
    inline void set_lift_coeff_front(float cl) { tuning::lift_coeff_front = cl; }
    inline void set_lift_coeff_rear(float cl)  { tuning::lift_coeff_rear = cl; }
    
    inline void  set_ground_effect_enabled(bool enabled)  { tuning::ground_effect_enabled = enabled; }
    inline bool  get_ground_effect_enabled()              { return tuning::ground_effect_enabled; }
    inline void  set_ground_effect_multiplier(float mult) { tuning::ground_effect_multiplier = mult; }
    inline float get_ground_effect_multiplier()           { return tuning::ground_effect_multiplier; }

    inline void set_throttle(float v)  { input_target.throttle  = PxClamp(v, 0.0f, 1.0f); }
    inline void set_brake(float v)     { input_target.brake     = PxClamp(v, 0.0f, 1.0f); }
    inline void set_steering(float v)  { input_target.steering  = PxClamp(v, -1.0f, 1.0f); }
    inline void set_handbrake(float v) { input_target.handbrake = PxClamp(v, 0.0f, 1.0f); }

    inline void update_input(float dt)
    {
        float diff = input_target.steering - input.steering;
        float max_change = tuning::steering_rate * dt;
        input.steering = (fabsf(diff) <= max_change) ? input_target.steering : input.steering + ((diff > 0) ? max_change : -max_change);

        input.throttle = (input_target.throttle < input.throttle) ? input_target.throttle
            : lerp(input.throttle, input_target.throttle, exp_decay(tuning::throttle_smoothing, dt));
        input.brake = (input_target.brake < input.brake) ? input_target.brake
            : lerp(input.brake, input_target.brake, exp_decay(tuning::throttle_smoothing, dt));

        input.handbrake = input_target.handbrake;
    }
    
    inline void update_suspension(PxScene* scene, float dt)
    {
        // --- raycast setup ---
        PxTransform pose = body->getGlobalPose();
        PxVec3 local_down  = pose.q.rotate(PxVec3(0, -1, 0));
        PxVec3 local_fwd   = pose.q.rotate(PxVec3(0, 0, 1));
        PxVec3 local_right = pose.q.rotate(PxVec3(1, 0, 0));
        
        const int ray_count = 7;
        float half_width = cfg.wheel_width * 0.4f;
        
        auto get_curvature_height = [&](float x_offset) -> float
        {
            float r = cfg.wheel_radius;
            float x = PxMin(fabsf(x_offset), r * 0.95f);
            return r - sqrtf(r * r - x * x);
        };
        
        float dist_near   = cfg.wheel_radius * 0.4f;
        float dist_far    = cfg.wheel_radius * 0.75f;
        float height_near = get_curvature_height(dist_near);
        float height_far  = get_curvature_height(dist_far);
        
        PxVec3 ray_offsets[ray_count] = {
            PxVec3(0.0f,       0.0f,        0.0f),
            PxVec3(dist_near,  0.0f,        height_near),
            PxVec3(dist_far,   0.0f,        height_far),
            PxVec3(-dist_near, 0.0f,        height_near),
            PxVec3(-dist_far,  0.0f,        height_far),
            PxVec3(0.0f,       -half_width, 0.0f),
            PxVec3(0.0f,        half_width, 0.0f)
        };
        
        PxQueryFilterData filter;
        filter.flags = PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC;
        
        float max_curvature_height = height_far;
        float ray_len = cfg.suspension_travel + cfg.wheel_radius + max_curvature_height + 0.5f;
        float max_dist = cfg.suspension_travel + cfg.wheel_radius;
        
        for (int i = 0; i < wheel_count; i++)
        {
            wheel& w = wheels[i];
            w.prev_compression = w.compression;
            
            PxVec3 attach = wheel_offsets[i];
            attach.y += cfg.suspension_travel;
            PxVec3 world_attach = pose.transform(attach);
            
            float min_ground_dist = FLT_MAX;
            PxVec3 best_contact_point = PxVec3(0);
            PxVec3 accumulated_normal = PxVec3(0);
            int hit_count = 0;
            
            // per-ray hit storage for weighted contact patch averaging
            float  ray_dists[ray_count];
            PxVec3 ray_positions[ray_count];
            PxVec3 ray_normals[ray_count];
            bool   ray_valid[ray_count] = {};
            
            for (int r = 0; r < ray_count; r++)
            {
                PxVec3 offset = local_fwd * ray_offsets[r].x + local_right * ray_offsets[r].y - local_down * ray_offsets[r].z;
                PxVec3 ray_origin = world_attach + offset;
                
                debug_rays[i][r].origin = ray_origin;
                debug_rays[i][r].hit = false;
                
                PxRaycastBuffer hit;
                if (scene->raycast(ray_origin, local_down, ray_len, hit, PxHitFlag::eDEFAULT, filter) &&
                    hit.block.actor && hit.block.actor != body)
                {
                    debug_rays[i][r].hit_point = hit.block.position;
                    debug_rays[i][r].hit = true;
                    
                    float adjusted_dist = hit.block.distance - ray_offsets[r].z;
                    if (adjusted_dist <= max_dist)
                    {
                        ray_dists[hit_count]     = adjusted_dist;
                        ray_positions[hit_count] = hit.block.position;
                        ray_normals[hit_count]   = hit.block.normal;
                        ray_valid[hit_count]     = true;
                        hit_count++;
                        accumulated_normal += hit.block.normal;
                        if (adjusted_dist < min_ground_dist)
                            min_ground_dist = adjusted_dist;
                    }
                }
                else
                {
                    debug_rays[i][r].hit_point = ray_origin + local_down * ray_len;
                }
            }
            
            // --- contact averaging ---
            if (hit_count > 1)
            {
                float weight_margin = cfg.wheel_radius * 0.3f;
                PxVec3 weighted_pos(0);
                PxVec3 weighted_nrm(0);
                float total_w = 0.0f;
                for (int h = 0; h < hit_count; h++)
                {
                    if (ray_dists[h] <= min_ground_dist + weight_margin)
                    {
                        float w = 1.0f / (1.0f + (ray_dists[h] - min_ground_dist) * 10.0f);
                        weighted_pos += ray_positions[h] * w;
                        weighted_nrm += ray_normals[h] * w;
                        total_w += w;
                    }
                }
                if (total_w > 0.0f)
                {
                    best_contact_point = weighted_pos * (1.0f / total_w);
                    accumulated_normal = weighted_nrm;
                }
                else
                {
                    best_contact_point = ray_positions[0];
                }
            }
            else if (hit_count == 1)
            {
                best_contact_point = ray_positions[0];
            }
            
            debug_suspension_top[i] = world_attach;
            PxVec3 wheel_center = world_attach + local_down * (cfg.suspension_travel * (1.0f - w.compression) + cfg.wheel_radius);
            debug_suspension_bottom[i] = wheel_center;
            
            if (hit_count > 0)
            {
                w.grounded = true;
                w.contact_point = best_contact_point;
                w.contact_normal = accumulated_normal.getNormalized();
                float dist_from_rest = min_ground_dist - cfg.wheel_radius;
                
                // --- road bumps ---
                float speed = body->getLinearVelocity().magnitude();
                if (speed > 1.0f && tuning::road_bump_amplitude > 0.0f)
                {
                    float phase = road_bump_phase;
                    float bump = sinf(phase * 17.3f + i * 2.1f) * (0.5f + 0.5f * sinf(phase * 7.1f + i * 4.3f));
                    bump += sinf(phase * 31.7f + i * 1.3f) * 0.3f;
                    dist_from_rest += bump * tuning::road_bump_amplitude;
                }
                
                w.target_compression = PxClamp(1.0f - dist_from_rest / cfg.suspension_travel, 0.0f, 1.0f);
            }
            else
            {
                w.grounded = false;
                w.target_compression = 0.0f;
                w.contact_normal = PxVec3(0, 1, 0);
            }
            
            // --- wheel tracking ---
            float compression_error = w.target_compression - w.compression;
            float wheel_spring_force = spring_stiffness[i] * compression_error;
            float wheel_damper_force = -spring_damping[i] * w.compression_velocity * 0.15f;
            float wheel_accel = (wheel_spring_force + wheel_damper_force) / cfg.wheel_mass;
            
            w.compression_velocity += wheel_accel * dt;
            w.compression += w.compression_velocity * dt;
            
            if (w.compression > 1.0f)      { w.compression = 1.0f; w.compression_velocity = PxMin(w.compression_velocity, 0.0f); }
            else if (w.compression < 0.0f) { w.compression = 0.0f; w.compression_velocity = PxMax(w.compression_velocity, 0.0f); }
        }
    }
    
    inline void apply_suspension_forces(float dt)
    {
        PxTransform pose = body->getGlobalPose();
        float forces[wheel_count];
        
        for (int i = 0; i < wheel_count; i++)
        {
            wheel& w = wheels[i];
            if (!w.grounded)
            {
                forces[i] = 0.0f;
                w.tire_load = 0.0f;
                continue;
            }
            
            float displacement = w.compression * cfg.suspension_travel;
            float spring_f = spring_stiffness[i] * displacement;
            float susp_vel = PxClamp(w.compression_velocity * cfg.suspension_travel, -tuning::max_damper_velocity, tuning::max_damper_velocity);
            float damper_ratio = (susp_vel > 0.0f) ? tuning::damping_bump_ratio : tuning::damping_rebound_ratio;
            float damper_f = spring_damping[i] * susp_vel * damper_ratio;
            
            forces[i] = PxClamp(spring_f + damper_f, 0.0f, tuning::max_susp_force);
            
            // bump stop - progressive stiffness increase near full compression
            if (w.compression > tuning::bump_stop_threshold)
            {
                float penetration = (w.compression - tuning::bump_stop_threshold) / (1.0f - tuning::bump_stop_threshold);
                forces[i] += tuning::bump_stop_stiffness * penetration * penetration * cfg.suspension_travel;
            }
        }
        
        auto apply_arb = [&](int left, int right, float stiffness)
        {
            // arb load transfer
            float diff = wheels[left].compression - wheels[right].compression;
            float arb_force = diff * stiffness;
            if (wheels[left].grounded)  forces[left]  += arb_force;
            if (wheels[right].grounded) forces[right] -= arb_force;
        };
        apply_arb(front_left, front_right, tuning::front_arb_stiffness);
        apply_arb(rear_left, rear_right, tuning::rear_arb_stiffness);
        
        for (int i = 0; i < wheel_count; i++)
        {
            forces[i] = PxClamp(forces[i], 0.0f, tuning::max_susp_force);
            wheels[i].tire_load = forces[i] + cfg.wheel_mass * 9.81f;

            if (forces[i] > 0.0f && wheels[i].grounded)
            {
                PxVec3 force = wheels[i].contact_normal * forces[i];
                PxVec3 pos = pose.transform(wheel_offsets[i]);
                PxRigidBodyExt::addForceAtPos(*body, force, pos, PxForceMode::eFORCE);
            }
        }
        
        // longitudinal weight transfer
        float wheelbase = cfg.length * 0.7f;
        float com_height = fabsf(tuning::center_of_mass_y) + cfg.wheel_radius;
        float weight_transfer = cfg.mass * longitudinal_accel * com_height / PxMax(wheelbase, 0.1f);
        float max_transfer = cfg.mass * 9.81f * 0.1f;
        weight_transfer = PxClamp(weight_transfer, -max_transfer, max_transfer);
        float transfer_per_wheel = weight_transfer * 0.5f * 0.3f;
        for (int i = 0; i < wheel_count; i++)
        {
            if (wheels[i].grounded)
            {
                if (is_front(i))
                    wheels[i].tire_load -= transfer_per_wheel;
                else
                    wheels[i].tire_load += transfer_per_wheel;
                wheels[i].tire_load = PxMax(wheels[i].tire_load, 0.0f);
            }
        }
    }
    
    inline void apply_tire_forces(float wheel_angles[wheel_count], float dt)
    {
        // --- setup ---
        PxTransform pose = body->getGlobalPose();
        PxVec3 chassis_fwd   = pose.q.rotate(PxVec3(0, 0, 1));
        PxVec3 chassis_right = pose.q.rotate(PxVec3(1, 0, 0));
        
        if (tuning::log_pacejka)
            SP_LOG_INFO("=== tire forces: speed=%.1f m/s ===", body->getLinearVelocity().magnitude());
        
        for (int i = 0; i < wheel_count; i++)
        {
            wheel& w = wheels[i];
            const char* wheel_name = wheel_names[i];
            
            // --- airborne branch ---
            if (!w.grounded || w.tire_load <= 0.0f)
            {
                if (tuning::log_pacejka)
                    SP_LOG_INFO("[%s] airborne: grounded=%d, tire_load=%.1f", wheel_name, w.grounded, w.tire_load);
                w.slip_angle = w.slip_ratio = w.lateral_force = w.longitudinal_force = 0.0f;
                
                PxVec3 vel = body->getLinearVelocity();
                float car_fwd_speed = vel.dot(chassis_fwd);
                float target_w = car_fwd_speed / cfg.wheel_radius;
                
                if (input.handbrake > tuning::input_deadzone && is_rear(i))
                {
                    // progressive handbrake friction even when airborne
                    float hb_torque = tuning::handbrake_torque * input.handbrake;
                    float hb_sign = (w.angular_velocity > 0.0f) ? -1.0f : 1.0f;
                    float new_w = w.angular_velocity + hb_sign * hb_torque / wheel_moi[i] * dt;
                    w.angular_velocity = ((w.angular_velocity > 0.0f && new_w < 0.0f) || (w.angular_velocity < 0.0f && new_w > 0.0f)) ? 0.0f : new_w;
                }
                else
                    w.angular_velocity = lerp(w.angular_velocity, target_w, exp_decay(5.0f, dt));
                
                // airborne cooling: 3x rate since no road contact heat
                float temp_above = w.temperature - tuning::tire_ambient_temp;
                if (temp_above > 0.0f)
                {
                    float cooling_factor = PxMin(temp_above / 30.0f, 1.0f);
                    w.temperature -= tuning::tire_cooling_rate * 3.0f * cooling_factor * dt;
                }
                w.temperature = PxMax(w.temperature, tuning::tire_ambient_temp);
                w.rotation += w.angular_velocity * dt;
                continue;
            }
            
            PxVec3 world_pos = pose.transform(wheel_offsets[i]);
            PxVec3 wheel_vel = body->getLinearVelocity() + body->getAngularVelocity().cross(world_pos - pose.p);
            wheel_vel -= w.contact_normal * wheel_vel.dot(w.contact_normal);
            
            float cs = cosf(wheel_angles[i]), sn = sinf(wheel_angles[i]);
            PxVec3 wheel_fwd = chassis_fwd * cs + chassis_right * sn;
            PxVec3 wheel_lat = chassis_right * cs - chassis_fwd * sn;
            
            float vx = wheel_vel.dot(wheel_fwd);
            float vy = wheel_vel.dot(wheel_lat);
            float wheel_speed  = w.angular_velocity * cfg.wheel_radius;
            float ground_speed = sqrtf(vx * vx + vy * vy);
            float max_v = PxMax(fabsf(wheel_speed), fabsf(vx));
            
            if (tuning::log_pacejka)
                SP_LOG_INFO("[%s] vx=%.3f, vy=%.3f, ws=%.3f", wheel_name, vx, vy, wheel_speed);
            
            float wear_factor    = 1.0f - w.wear * tuning::tire_grip_wear_loss;
            float base_grip     = tuning::tire_friction * load_sensitive_grip(PxMax(w.tire_load, 0.0f)) * wear_factor;
            float temp_factor   = get_tire_temp_grip_factor(w.temperature);
            float camber_factor = get_camber_grip_factor(i, w.slip_angle);
            float surface_factor = get_surface_friction(w.contact_surface);
            float peak_force    = base_grip * temp_factor * camber_factor * surface_factor;
            
            if (tuning::log_pacejka)
                SP_LOG_INFO("[%s] load=%.0f, peak_force=%.0f", wheel_name, w.tire_load, peak_force);
            
            float lat_f = 0.0f, long_f = 0.0f;
            
            // --- at-rest branch ---
            bool at_rest = ground_speed < 0.1f && fabsf(wheel_speed) < 0.2f;
            if (at_rest)
            {
                w.slip_ratio = w.slip_angle = 0.0f;
                w.angular_velocity = lerp(w.angular_velocity, 0.0f, exp_decay(20.0f, dt));
                w.rotation += w.angular_velocity * dt;
                
                float friction_force = peak_force * 0.8f;
                float friction_gain = cfg.mass * 10.0f;
                lat_f  = PxClamp(-vy * friction_gain, -friction_force, friction_force);
                long_f = PxClamp(-vx * friction_gain, -friction_force, friction_force);
                w.lateral_force = lat_f;
                w.longitudinal_force = long_f;
                PxRigidBodyExt::addForceAtPos(*body, wheel_lat * lat_f + wheel_fwd * long_f, world_pos, PxForceMode::eFORCE);
                
                // at-rest cooling: 1x rate, no slip-induced heat
                float temp_above_ambient = w.temperature - tuning::tire_ambient_temp;
                if (temp_above_ambient > 0.0f)
                {
                    float cooling_factor = PxMin(temp_above_ambient / 30.0f, 1.0f);
                    w.temperature -= tuning::tire_cooling_rate * cooling_factor * dt;
                    w.temperature = PxMax(w.temperature, tuning::tire_ambient_temp);
                }
                
                if (tuning::log_pacejka)
                    SP_LOG_INFO("[%s] at rest: vx=%.3f, vy=%.3f, friction long_f=%.1f, lat_f=%.1f", wheel_name, vx, vy, long_f, lat_f);
                continue;
            }
            
            // --- normal slip regime ---
            if (max_v > tuning::min_slip_speed)
            {
                // sae slip ratio: denom = max(|vx|, |wheel_speed|)
                float abs_vx = fabsf(vx);
                float abs_ws = fabsf(wheel_speed);
                float slip_denom = PxMax((wheel_speed >= vx) ? abs_ws : abs_vx, 0.01f);
                float raw_slip_ratio = PxClamp((wheel_speed - vx) / slip_denom, -1.0f, 1.0f);
                float raw_slip_angle = atan2f(vy, PxMax(abs_vx, 0.5f));
                
                // tire relaxation: smooth slip over distance traveled
                float long_distance = PxMax(ground_speed, fabsf(wheel_speed)) * dt;
                float lat_distance  = ground_speed * dt;
                float long_blend = 1.0f - expf(-long_distance / tuning::tire_relaxation_length);
                float lat_blend  = 1.0f - expf(-lat_distance / tuning::tire_relaxation_length);
                w.slip_ratio = lerp(w.slip_ratio, raw_slip_ratio, long_blend);
                w.slip_angle = lerp(w.slip_angle, raw_slip_angle, lat_blend);

                if (tuning::log_pacejka)
                    SP_LOG_INFO("[%s] slip: sr=%.4f, sa=%.4f", wheel_name, w.slip_ratio, w.slip_angle);

                float effective_slip_angle = w.slip_angle;
                if (fabsf(effective_slip_angle) < tuning::slip_angle_deadband)
                {
                    float factor = fabsf(effective_slip_angle) / tuning::slip_angle_deadband;
                    effective_slip_angle *= factor * factor;
                }

                // clamp slip angle for pacejka to prevent friction drop-off at large angles (sliding sideways)
                float pacejka_slip_angle = PxClamp(effective_slip_angle, -tuning::max_slip_angle, tuning::max_slip_angle);
                
                // load-dependent B coefficient scaling
                float load_norm = w.tire_load / tuning::load_reference;
                float B_load_scale = 1.0f / PxMax(load_norm, tuning::load_B_scale_min);
                float lat_B_eff  = tuning::lat_B * B_load_scale;
                float long_B_eff = tuning::long_B * B_load_scale;
                
                // combined slip via equivalent slip projection
                float sin_a = sinf(pacejka_slip_angle);
                float cos_a = cosf(pacejka_slip_angle);
                float sigma = sqrtf(w.slip_ratio * w.slip_ratio + sin_a * sin_a);
                
                float lat_mu, long_mu;
                if (sigma > 0.001f)
                {
                    // combined regime - both slip axes active
                    float sigma_lat  = sin_a / sigma;
                    float sigma_long = w.slip_ratio / sigma;
                    
                    // evaluate pacejka with the total slip magnitude, then project back
                    float mu_lat_raw  = pacejka(sigma, lat_B_eff, tuning::lat_C, tuning::lat_D, tuning::lat_E);
                    float mu_long_raw = pacejka(sigma, long_B_eff, tuning::long_C, tuning::long_D, tuning::long_E);
                    
                    lat_mu  = mu_lat_raw * sigma_lat;
                    long_mu = mu_long_raw * sigma_long;
                }
                else
                {
                    // pure slip regime - standard independent evaluation
                    lat_mu  = pacejka(pacejka_slip_angle, lat_B_eff, tuning::lat_C, tuning::lat_D, tuning::lat_E);
                    long_mu = pacejka(w.slip_ratio, long_B_eff, tuning::long_C, tuning::long_D, tuning::long_E);
                }
                
                // lateral grip floor
                float lat_abs  = fabsf(lat_mu);
                float long_abs = fabsf(long_mu);
                if (lat_abs < tuning::min_lateral_grip * long_abs && fabsf(effective_slip_angle) > 0.001f)
                {
                    float scale = tuning::min_lateral_grip * long_abs / PxMax(lat_abs, 0.001f);
                    lat_mu *= PxMin(scale, 2.0f);
                }

                lat_f  = -lat_mu * peak_force;
                long_f =  long_mu * peak_force;
                if (is_rear(i))
                    lat_f *= tuning::rear_grip_ratio;

                float camber = is_front(i) ? tuning::front_camber : tuning::rear_camber;
                bool is_left_wheel = (i == front_left || i == rear_left);
                float camber_thrust = camber * w.tire_load * tuning::camber_thrust_coeff;
                lat_f += is_left_wheel ? -camber_thrust : camber_thrust;

                if (tuning::log_pacejka)
                    SP_LOG_INFO("[%s] pacejka: lat_mu=%.3f, long_mu=%.3f, lat_f=%.1f, long_f=%.1f", wheel_name, lat_mu, long_mu, lat_f, long_f);
            }
            // --- low-speed branch ---
            else
            {
                w.slip_ratio = w.slip_angle = 0.0f;
                float speed_factor = PxClamp(max_v / tuning::min_slip_speed, 0.0f, 1.0f);
                float low_speed_force = peak_force * speed_factor * speed_factor * 0.2f;
                long_f = PxClamp((wheel_speed - vx) / tuning::min_slip_speed, -1.0f, 1.0f) * low_speed_force;
                lat_f  = PxClamp(-vy / tuning::min_slip_speed, -1.0f, 1.0f) * low_speed_force;
                
                if (tuning::log_pacejka)
                    SP_LOG_INFO("[%s] low-speed: max_v=%.3f, speed_factor=%.2f, long_f=%.1f, lat_f=%.1f", wheel_name, max_v, speed_factor, long_f, lat_f);
            }
            
            // --- heating / cooling ---
            float rolling_heat = fabsf(wheel_speed) * tuning::tire_heat_from_rolling;
            float cooling = tuning::tire_cooling_rate + ground_speed * tuning::tire_cooling_airflow;
            float temp_delta = w.temperature - tuning::tire_ambient_temp;
            float force_magnitude = sqrtf(long_f * long_f + lat_f * lat_f);
            float normalized_force = force_magnitude / tuning::load_reference;
            float friction_work = (max_v > tuning::min_slip_speed)
                ? normalized_force * (fabsf(w.slip_angle) + fabsf(w.slip_ratio))
                : normalized_force * 0.01f;
            
            float heating = friction_work * tuning::tire_heat_from_slip + rolling_heat;
            float cooling_factor = (temp_delta > 0.0f) ? PxMin(temp_delta / 30.0f, 1.0f) : 0.0f;
            w.temperature += (heating - cooling * cooling_factor) * dt;
            w.temperature = PxClamp(w.temperature, tuning::tire_min_temp, tuning::tire_max_temp);
            
            // --- tire wear ---
            float heat_excess = PxMax(w.temperature - tuning::tire_optimal_temp, 0.0f) / tuning::tire_temp_range;
            float wear_rate = tuning::tire_wear_rate * (1.0f + heat_excess * tuning::tire_wear_heat_mult);
            float wear_amount = wear_rate * (fabsf(w.slip_angle) + fabsf(w.slip_ratio)) * ground_speed * dt;
            w.wear = PxMin(w.wear + PxMax(wear_amount, 0.0f), 1.0f);
            
            if (is_rear(i) && input.handbrake > tuning::input_deadzone)
            {
                float sliding_f = tuning::handbrake_sliding_factor * peak_force;
                long_f = (fabsf(vx) > 0.01f) ? ((vx > 0.0f ? -1.0f : 1.0f) * sliding_f * input.handbrake) : 0.0f;
                lat_f *= (1.0f - 0.5f * input.handbrake);
            }
            
            w.lateral_force = lat_f;
            w.longitudinal_force = long_f;
            
            PxRigidBodyExt::addForceAtPos(*body, wheel_lat * lat_f + wheel_fwd * long_f, world_pos, PxForceMode::eFORCE);
            
            if (is_rear(i) && input.handbrake > tuning::input_deadzone)
            {
                // progressive handbrake - high friction torque instead of instant lock
                float hb_torque = tuning::handbrake_torque * input.handbrake;
                float hb_sign = (w.angular_velocity > 0.0f) ? -1.0f : 1.0f;
                float new_w = w.angular_velocity + hb_sign * hb_torque / wheel_moi[i] * dt;
                if ((w.angular_velocity > 0.0f && new_w < 0.0f) || (w.angular_velocity < 0.0f && new_w > 0.0f))
                    new_w = 0.0f;
                w.angular_velocity = new_w;
            }
            else
            {
                w.angular_velocity += (-long_f * cfg.wheel_radius / wheel_moi[i]) * dt;
                
                bool coasting = input.throttle < 0.01f && input.brake < 0.01f;
                // sync undriven/coasting wheels to ground speed
                bool should_match = coasting || is_front(i) || (ground_speed < tuning::min_slip_speed && (is_front(i) || input.throttle < 0.01f));
                if (should_match)
                {
                    float target_w = vx / cfg.wheel_radius;
                    float match_rate = coasting ? 50.0f : ((ground_speed < tuning::min_slip_speed) ? tuning::ground_match_rate * 2.0f : tuning::ground_match_rate);
                    w.angular_velocity = lerp(w.angular_velocity, target_w, exp_decay(match_rate, dt));
                }
                
                w.angular_velocity *= (1.0f - tuning::bearing_friction * dt);
            }
            w.rotation += w.angular_velocity * dt;
            
            if (tuning::log_pacejka)
                SP_LOG_INFO("[%s] ang_vel=%.4f, lat_f=%.1f, long_f=%.1f", wheel_name, w.angular_velocity, lat_f, long_f);
        }
        if (tuning::log_pacejka)
            SP_LOG_INFO("=== pacejka tick end ===\n");
    }
    
    inline void apply_self_aligning_torque()
    {
        // pneumatic trail: shifts force point within contact patch
        float sat = 0.0f;
        for (int i = 0; i < 2; i++)
        {
            if (!wheels[i].grounded)
                continue;
            
            float abs_sa = fabsf(wheels[i].slip_angle);
            float sa_norm = abs_sa / tuning::pneumatic_trail_peak;
            
            // trail profile: starts at max, linearly drops to zero at peak slip, then goes negative
            float trail = tuning::pneumatic_trail_max * (1.0f - sa_norm);
            
            sat += wheels[i].lateral_force * trail;
        }
        
        PxVec3 up = body->getGlobalPose().q.rotate(PxVec3(0, 1, 0));
        body->addTorque(up * sat * tuning::self_align_gain, PxForceMode::eFORCE);
    }
    
    inline void apply_lsd_torque(float total_torque, float dt)
    {
        if (tuning::diff_type == 0)
        {
            // open differential - equal torque split, wheels free to spin independently
            wheels[rear_left].angular_velocity  += (total_torque * 0.5f) / wheel_moi[rear_left] * dt;
            wheels[rear_right].angular_velocity += (total_torque * 0.5f) / wheel_moi[rear_right] * dt;
        }
        else if (tuning::diff_type == 1)
        {
            // locked differential - forces both wheels to same speed
            float avg_w = (wheels[rear_left].angular_velocity + wheels[rear_right].angular_velocity) * 0.5f;
            wheels[rear_left].angular_velocity  = avg_w + (total_torque * 0.5f) / wheel_moi[rear_left] * dt;
            wheels[rear_right].angular_velocity = avg_w + (total_torque * 0.5f) / wheel_moi[rear_right] * dt;
        }
        else
        {
            // lsd - clutch-pack limited slip with separate accel/decel lock ratios
            float w_left  = wheels[rear_left].angular_velocity;
            float w_right = wheels[rear_right].angular_velocity;
            float delta_w = w_left - w_right;
            
            // deadband prevents the lsd from hunting when wheels are nearly matched
            float effective_delta = (fabsf(delta_w) > 0.5f) ? delta_w : 0.0f;
            
            float lock_ratio = (total_torque >= 0.0f) ? tuning::lsd_lock_ratio_accel : tuning::lsd_lock_ratio_decel;
            
            float lock_torque = tuning::lsd_preload + fabsf(effective_delta) * lock_ratio * fabsf(total_torque);
            lock_torque = PxMin(lock_torque, fabsf(total_torque) * 0.9f);
            
            float bias_sign = (delta_w > 0.0f) ? -1.0f : 1.0f;
            
            wheels[rear_left].angular_velocity  += (total_torque * 0.5f + bias_sign * lock_torque * 0.5f) / wheel_moi[rear_left] * dt;
            wheels[rear_right].angular_velocity += (total_torque * 0.5f - bias_sign * lock_torque * 0.5f) / wheel_moi[rear_right] * dt;
        }
    }
    
    inline void apply_drivetrain(float forward_speed_kmh, float dt)
    {
        float forward_speed_ms = forward_speed_kmh / 3.6f;

        // --- gearbox ---
        update_automatic_gearbox(dt, input.throttle, forward_speed_ms);

        if (downshift_blip_timer > 0.0f)
            downshift_blip_timer -= dt;

        float avg_wheel_rpm = (wheels[rear_left].angular_velocity + wheels[rear_right].angular_velocity) * 0.5f * 60.0f / (2.0f * PxPi);
        float wheel_driven_rpm = wheel_rpm_to_engine_rpm(fabsf(avg_wheel_rpm), current_gear);

        bool coasting = input.throttle < tuning::input_deadzone && input.brake < tuning::input_deadzone;
        if (coasting && current_gear >= 2)
        {
            float ground_wheel_rpm = fabsf(forward_speed_ms) / cfg.wheel_radius * 60.0f / (2.0f * PxPi);
            float ground_driven_rpm = wheel_rpm_to_engine_rpm(ground_wheel_rpm, current_gear);
            wheel_driven_rpm = PxMax(wheel_driven_rpm, ground_driven_rpm);
        }

        // --- clutch / rpm ---
        if (is_shifting)                                                  clutch = 0.8f;
        else if (current_gear == 1)                                       clutch = 0.0f;
        else if (fabsf(forward_speed_ms) < 2.0f && input.throttle > 0.1f) clutch = lerp(clutch, 1.0f, exp_decay(tuning::clutch_engagement_rate, dt));
        else                                                              clutch = 1.0f;

        float blip = (downshift_blip_timer > 0.0f) ? tuning::downshift_blip_amount * (downshift_blip_timer / tuning::downshift_blip_duration) : 0.0f;
        float effective_throttle_for_rpm = PxMax(input.throttle, blip);
        float free_rev_rpm = tuning::engine_idle_rpm + effective_throttle_for_rpm * (tuning::engine_redline_rpm - tuning::engine_idle_rpm) * 0.7f;
        
        // in-gear: engine tracks wheel speed, floor prevents idle stall
        float target_rpm;
        if (current_gear == 1)
        {
            target_rpm = free_rev_rpm;
        }
        else
        {
            // throttle floor decays with clutch to avoid decoupling engine from wheels
            float throttle_floor = tuning::engine_idle_rpm + effective_throttle_for_rpm * 500.0f * (1.0f - clutch * 0.8f);
            target_rpm = PxMax(wheel_driven_rpm, throttle_floor);
        }

        // engine rpm smoothing (inertia model)
        float rpm_diff = target_rpm - engine_rpm;
        float smoothing_rate;
        if (rpm_diff >= 0.0f)
        {
            smoothing_rate = tuning::engine_rpm_smoothing;
        }
        else
        {
            // heavier rotating assembly decelerates slower, producing subtle rev hang
            smoothing_rate = tuning::engine_rpm_smoothing / (1.0f + tuning::engine_inertia);
        }
        engine_rpm = lerp(engine_rpm, target_rpm, exp_decay(smoothing_rate, dt));
        engine_rpm = PxClamp(engine_rpm, tuning::engine_idle_rpm, tuning::engine_max_rpm);

        // --- engine braking ---
        if (input.throttle < tuning::input_deadzone && clutch > 0.5f && current_gear >= 2)
        {
            float wheel_brake_torque = tuning::engine_friction * engine_rpm * 0.1f * fabsf(tuning::gear_ratios[current_gear]) * tuning::final_drive * 0.5f;
            for (int i = rear_left; i <= rear_right; i++)
                if (wheels[i].angular_velocity > 0.0f)
                    wheels[i].angular_velocity -= wheel_brake_torque / wheel_moi[i] * dt;
        }
        
        update_boost(input.throttle, engine_rpm, dt);
        
        // --- traction control / torque delivery ---
        if (input.throttle > tuning::input_deadzone && current_gear >= 2)
        {
            float base_torque = get_engine_torque(engine_rpm);
            float boosted_torque = base_torque * (1.0f + boost_pressure * tuning::boost_torque_mult);
            float engine_torque = boosted_torque * input.throttle;
            
            tc_active = false;
            if (tuning::tc_enabled)
            {
                // tc uses raw wheel speed, not smoothed slip ratio
                float ground_v = PxMax(fabsf(forward_speed_ms), 0.1f);
                float max_slip = 0.0f;
                for (int i = rear_left; i <= rear_right; i++)
                {
                    if (!wheels[i].grounded) continue;
                    float wheel_v = fabsf(wheels[i].angular_velocity * cfg.wheel_radius);
                    float raw_slip = (wheel_v - ground_v) / PxMax(wheel_v, ground_v);
                    if (raw_slip > 0.0f)
                        max_slip = PxMax(max_slip, raw_slip);
                }
                
                float target_reduction = 0.0f;
                if (max_slip > tuning::tc_slip_threshold)
                {
                    tc_active = true;
                    target_reduction = PxClamp((max_slip - tuning::tc_slip_threshold) * 5.0f, 0.0f, tuning::tc_power_reduction);
                }
                
                tc_reduction = lerp(tc_reduction, target_reduction, exp_decay(tuning::tc_response_rate, dt));
                engine_torque *= (1.0f - tc_reduction);
            }
            else
            {
                tc_reduction = 0.0f;
            }
            
            float gear_ratio = tuning::gear_ratios[current_gear] * tuning::final_drive;
            float wheel_torque = engine_torque * gear_ratio * clutch * tuning::drivetrain_efficiency;

            apply_lsd_torque(wheel_torque, dt);
        }
        else if (input.throttle > tuning::input_deadzone && current_gear == 0)
        {
            float brake_torque = tuning::brake_force * cfg.wheel_radius * input.throttle * 0.5f;
            for (int i = 0; i < wheel_count; i++)
            {
                if (wheels[i].angular_velocity < 0.0f)
                {
                    wheels[i].angular_velocity += brake_torque / wheel_moi[i] * dt;
                    if (wheels[i].angular_velocity > 0.0f) wheels[i].angular_velocity = 0.0f;
                }
            }
        }
        else
        {
            tc_reduction = lerp(tc_reduction, 0.0f, exp_decay(tuning::tc_response_rate * 2.0f, dt));
            tc_active = false;
        }
        
        // --- braking / abs ---
        if (input.brake > tuning::input_deadzone)
        {
            if (forward_speed_kmh > tuning::braking_speed_threshold)
            {
                float total_torque = tuning::brake_force * cfg.wheel_radius * input.brake;
                float front_t = total_torque * tuning::brake_bias_front * 0.5f;
                float rear_t  = total_torque * (1.0f - tuning::brake_bias_front) * 0.5f;
                
                abs_phase += tuning::abs_pulse_frequency * dt;
                if (abs_phase > 1.0f)
                    abs_phase -= 1.0f;
                
                for (int i = 0; i < wheel_count; i++)
                {
                    float t = is_front(i) ? front_t : rear_t;
                    
                    float brake_efficiency = get_brake_efficiency(wheels[i].brake_temp);
                    t *= brake_efficiency;
                    
                    float heat = fabsf(wheels[i].angular_velocity) * t * tuning::brake_heat_coefficient * dt;
                    wheels[i].brake_temp += heat;
                    wheels[i].brake_temp = PxMin(wheels[i].brake_temp, tuning::brake_max_temp);
                    
                    abs_active[i] = false;
                    if (tuning::abs_enabled && wheels[i].grounded && -wheels[i].slip_ratio > tuning::abs_slip_threshold)
                    {
                        abs_active[i] = true;
                        t *= (abs_phase < 0.5f) ? tuning::abs_release_rate : 1.0f;
                    }
                    
                    float sign = wheels[i].angular_velocity >= 0.0f ? -1.0f : 1.0f;
                    float new_w = wheels[i].angular_velocity + sign * t / wheel_moi[i] * dt;
                    
                    wheels[i].angular_velocity = ((wheels[i].angular_velocity > 0 && new_w < 0) || (wheels[i].angular_velocity < 0 && new_w > 0))
                        ? 0.0f : new_w;
                }
            }
            else
            {
                for (int i = 0; i < wheel_count; i++)
                    abs_active[i] = false;
                
                if (current_gear == 0)
                {
                    float engine_torque = get_engine_torque(engine_rpm) * input.brake * tuning::reverse_power_ratio;
                    float gear_ratio = tuning::gear_ratios[0] * tuning::final_drive;
                    apply_lsd_torque(engine_torque * gear_ratio * clutch, dt);
                }
                // reverse: full stop + brake hold required
                else if (fabsf(forward_speed_ms) < 0.5f && input.brake > 0.8f && input.throttle < tuning::input_deadzone && current_gear >= 2 && !is_shifting)
                {
                    current_gear = 0;
                    is_shifting = true;
                    shift_timer = tuning::shift_time * 2.0f;
                }
            }
        }
        else
        {
            for (int i = 0; i < wheel_count; i++)
                abs_active[i] = false;
        }
        
        // --- handbrake ---
        if (input.handbrake > tuning::input_deadzone)
        {
            for (int i = rear_left; i <= rear_right; i++)
            {
                float hb_torque = tuning::handbrake_torque * input.handbrake;
                float hb_sign = (wheels[i].angular_velocity > 0.0f) ? -1.0f : 1.0f;
                float new_w = wheels[i].angular_velocity + hb_sign * hb_torque / wheel_moi[i] * dt;
                if ((wheels[i].angular_velocity > 0.0f && new_w < 0.0f) || (wheels[i].angular_velocity < 0.0f && new_w > 0.0f))
                    new_w = 0.0f;
                wheels[i].angular_velocity = new_w;
            }
        }

        // --- coasting wheel sync ---
        if (input.throttle < tuning::input_deadzone && input.brake < tuning::input_deadzone && input.handbrake < tuning::input_deadzone)
        {
            float target_angular_v = forward_speed_ms / cfg.wheel_radius;
            for (int i = rear_left; i <= rear_right; i++)
            {
                float error = fabsf(wheels[i].angular_velocity - target_angular_v);
                float ground_speed = fabsf(forward_speed_ms);
                if (ground_speed > 1.0f && error > ground_speed * 0.5f / cfg.wheel_radius)
                    wheels[i].angular_velocity = lerp(wheels[i].angular_velocity, target_angular_v, exp_decay(tuning::ground_match_rate, dt));
            }
        }
    }
    
    inline void apply_aero_and_resistance()
    {
        PxTransform pose = body->getGlobalPose();
        PxVec3 vel = body->getLinearVelocity();
        float speed = vel.magnitude();

        // aero application points from mesh-computed center
        float aero_height = tuning::aero_center_height;
        PxVec3 front_pos = pose.p + pose.q.rotate(PxVec3(0, aero_height, tuning::aero_center_front_z));
        PxVec3 rear_pos  = pose.p + pose.q.rotate(PxVec3(0, aero_height, tuning::aero_center_rear_z));
        
        aero_debug.valid = false;
        aero_debug.position = pose.p;
        aero_debug.velocity = vel;
        aero_debug.front_aero_pos = front_pos;
        aero_debug.rear_aero_pos = rear_pos;
        aero_debug.ride_height = cfg.suspension_height + cfg.wheel_radius; // default ride height
        aero_debug.ground_effect_factor = 1.0f;
        aero_debug.yaw_angle = 0.0f;
        aero_debug.drag_force = PxVec3(0);
        aero_debug.front_downforce = PxVec3(0);
        aero_debug.rear_downforce = PxVec3(0);
        aero_debug.side_force = PxVec3(0);
        
        if (speed < 0.5f)
        {
            float tire_load = 0.0f;
            for (int i = 0; i < wheel_count; i++)
                if (wheels[i].grounded)
                    tire_load += wheels[i].tire_load;
            if (speed > 0.1f && tire_load > 0.0f)
                body->addForce(-vel.getNormalized() * tuning::rolling_resistance * tire_load, PxForceMode::eFORCE);
            aero_debug.valid = true;
            return;
        }
        
        PxVec3 local_fwd   = pose.q.rotate(PxVec3(0, 0, 1));
        PxVec3 local_up    = pose.q.rotate(PxVec3(0, 1, 0));
        PxVec3 local_right = pose.q.rotate(PxVec3(1, 0, 0));
        
        float forward_speed = vel.dot(local_fwd);
        float lateral_speed = vel.dot(local_right);
        
        float yaw_angle = 0.0f;
        if (speed > 1.0f)
        {
            PxVec3 vel_norm = vel.getNormalized();
            float cos_yaw = PxClamp(vel_norm.dot(local_fwd), -1.0f, 1.0f);
            yaw_angle = acosf(fabsf(cos_yaw));
        }
        
        float front_compression = (wheels[front_left].compression + wheels[front_right].compression) * 0.5f;
        float rear_compression  = (wheels[rear_left].compression + wheels[rear_right].compression) * 0.5f;
        float pitch_angle = (rear_compression - front_compression) * cfg.suspension_travel / (cfg.length * 0.7f);
        
        float avg_compression = (front_compression + rear_compression) * 0.5f;
        float ride_height = cfg.suspension_height - avg_compression * cfg.suspension_travel + cfg.wheel_radius;
        
        // drag
        float base_drag = 0.5f * tuning::air_density * tuning::drag_coeff * tuning::frontal_area * speed * speed;
        
        float yaw_drag_factor = 1.0f;
        if (tuning::yaw_aero_enabled && yaw_angle > 0.01f)
        {
            float yaw_factor = sinf(yaw_angle);
            yaw_drag_factor = 1.0f + yaw_factor * (tuning::yaw_drag_multiplier - 1.0f);
        }
        
        PxVec3 drag_force_vec = -vel.getNormalized() * base_drag * yaw_drag_factor;
        body->addForce(drag_force_vec, PxForceMode::eFORCE);

        // side force
        PxVec3 side_force_vec(0);
        if (tuning::yaw_aero_enabled && fabsf(lateral_speed) > 1.0f)
        {
            float side_force = 0.5f * tuning::air_density * tuning::yaw_side_force_coeff * tuning::side_area * lateral_speed * fabsf(lateral_speed);
            side_force_vec = -local_right * side_force;
            body->addForce(side_force_vec, PxForceMode::eFORCE);
        }
        
        // downforce
        PxVec3 front_downforce_vec(0);
        PxVec3 rear_downforce_vec(0);
        float ground_effect_factor = 1.0f;
        
        if (speed > 10.0f)
        {
            float dyn_pressure = 0.5f * tuning::air_density * speed * speed;
            
            float front_cl = tuning::lift_coeff_front;
            float rear_cl  = tuning::lift_coeff_rear;
            
            // drs reduces rear downforce for higher straight-line speed
            if (tuning::drs_enabled && drs_active)
                rear_cl *= tuning::drs_rear_cl_factor;
            
            if (tuning::ground_effect_enabled)
            {
                if (ride_height < tuning::ground_effect_height_max)
                {
                    float height_ratio = PxClamp((tuning::ground_effect_height_max - ride_height) / 
                                                 (tuning::ground_effect_height_max - tuning::ground_effect_height_ref), 0.0f, 1.0f);
                    ground_effect_factor = 1.0f + height_ratio * (tuning::ground_effect_multiplier - 1.0f);
                }
            }
            
            if (tuning::pitch_aero_enabled)
            {
                float pitch_shift = pitch_angle * tuning::pitch_sensitivity;
                front_cl *= (1.0f - pitch_shift);
                rear_cl  *= (1.0f + pitch_shift);
            }
            
            float yaw_downforce_factor = 1.0f;
            if (tuning::yaw_aero_enabled && yaw_angle > 0.1f)
                yaw_downforce_factor = PxMax(0.3f, 1.0f - sinf(yaw_angle) * 0.7f);
            
            float front_downforce = front_cl * dyn_pressure * tuning::frontal_area * ground_effect_factor * yaw_downforce_factor;
            float rear_downforce  = rear_cl  * dyn_pressure * tuning::frontal_area * ground_effect_factor * yaw_downforce_factor;
            
            front_downforce_vec = local_up * front_downforce;
            rear_downforce_vec  = local_up * rear_downforce;
            
            PxRigidBodyExt::addForceAtPos(*body, front_downforce_vec, front_pos, PxForceMode::eFORCE);
            PxRigidBodyExt::addForceAtPos(*body, rear_downforce_vec, rear_pos, PxForceMode::eFORCE);
        }
        
        // per-wheel rolling resistance along chassis forward direction
        // at high slip angles, applying along velocity would incorrectly steer the car
        for (int i = 0; i < wheel_count; i++)
        {
            if (wheels[i].grounded && wheels[i].tire_load > 0.0f)
            {
                float rr_sign = (forward_speed > 0.0f) ? -1.0f : 1.0f;
                PxVec3 rr_force = local_fwd * rr_sign * tuning::rolling_resistance * wheels[i].tire_load;
                PxVec3 wheel_pos = pose.transform(wheel_offsets[i]);
                PxRigidBodyExt::addForceAtPos(*body, rr_force, wheel_pos, PxForceMode::eFORCE);
            }
        }
        
        aero_debug.drag_force = drag_force_vec;
        aero_debug.front_downforce = front_downforce_vec;
        aero_debug.rear_downforce = rear_downforce_vec;
        aero_debug.side_force = side_force_vec;
        aero_debug.front_aero_pos = front_pos;
        aero_debug.rear_aero_pos = rear_pos;
        aero_debug.ride_height = ride_height;
        aero_debug.yaw_angle = yaw_angle;
        aero_debug.ground_effect_factor = ground_effect_factor;
        aero_debug.valid = true;
    }
    
    inline void calculate_steering(float forward_speed, float speed_kmh, float out_angles[wheel_count])
    {
        float reduction = (speed_kmh > 80.0f)
            ? 1.0f - tuning::high_speed_steer_reduction * PxClamp((speed_kmh - 80.0f) / 120.0f, 0.0f, 1.0f)
            : 1.0f;

        float curved_input = copysignf(powf(fabsf(input.steering), tuning::steering_linearity), input.steering);
        float base = curved_input * tuning::max_steer_angle * reduction;

        // bump steer
        float front_left_bump  = wheels[front_left].compression * cfg.suspension_travel * tuning::front_bump_steer;
        float front_right_bump = wheels[front_right].compression * cfg.suspension_travel * tuning::front_bump_steer;
        float rear_left_bump   = wheels[rear_left].compression * cfg.suspension_travel * tuning::rear_bump_steer;
        float rear_right_bump  = wheels[rear_right].compression * cfg.suspension_travel * tuning::rear_bump_steer;

        out_angles[rear_left]  = tuning::rear_toe + rear_left_bump;
        out_angles[rear_right] = -tuning::rear_toe - rear_right_bump;

        if (fabsf(base) < tuning::steering_deadzone)
        {
            out_angles[front_left]  = tuning::front_toe + front_left_bump;
            out_angles[front_right] = -tuning::front_toe - front_right_bump;
            return;
        }

        // ackermann geometry
        if (forward_speed >= 0.0f)
        {
            float wheelbase  = cfg.length * 0.7f;
            float half_track = (cfg.width - cfg.wheel_width) * 0.5f;
            float turn_r     = wheelbase / tanf(fabsf(base));

            float inner = atanf(wheelbase / PxMax(turn_r - half_track, 0.1f));
            float outer = atanf(wheelbase / PxMax(turn_r + half_track, 0.1f));

            if (base > 0.0f)
            {
                out_angles[front_right] =  inner - tuning::front_toe + front_right_bump;
                out_angles[front_left]  =  outer + tuning::front_toe + front_left_bump;
            }
            else
            {
                out_angles[front_left]  = -inner + tuning::front_toe + front_left_bump;
                out_angles[front_right] = -outer - tuning::front_toe + front_right_bump;
            }
        }
        else
        {
            out_angles[front_left]  = base + tuning::front_toe + front_left_bump;
            out_angles[front_right] = base - tuning::front_toe - front_right_bump;
        }
    }

    inline void tick(float dt)
    {
        if (!body) return;

        PxScene* scene = body->getScene();
        if (!scene) return;

        // --- input ---
        update_input(dt);

        PxTransform pose = body->getGlobalPose();
        PxVec3 fwd = pose.q.rotate(PxVec3(0, 0, 1));
        PxVec3 vel = body->getLinearVelocity();
        float forward_speed = vel.dot(fwd);
        float speed_kmh = vel.magnitude() * 3.6f;

        // longitudinal accel for weight transfer (heavy low-pass, steady-state only)
        PxVec3 accel_vec = (vel - prev_velocity) / PxMax(dt, 0.001f);
        float raw_accel = accel_vec.dot(fwd);
        longitudinal_accel = lerp(longitudinal_accel, raw_accel, exp_decay(1.5f, dt));
        prev_velocity = vel;
        
        // advance road bump phase based on travel distance
        road_bump_phase += vel.magnitude() * tuning::road_bump_frequency * dt;
        
        // brake cooling
        float airspeed = vel.magnitude();
        for (int i = 0; i < wheel_count; i++)
        {
            float temp_above_ambient = wheels[i].brake_temp - tuning::brake_ambient_temp;
            if (temp_above_ambient > 0.0f)
            {
                float h = tuning::brake_cooling_base + airspeed * tuning::brake_cooling_airflow;
                float cooling_power = h * temp_above_ambient;
                float temp_drop = (cooling_power / tuning::brake_thermal_mass) * dt;
                wheels[i].brake_temp -= temp_drop;
                wheels[i].brake_temp = PxMax(wheels[i].brake_temp, tuning::brake_ambient_temp);
            }
        }
        
        // --- physics subsystems ---
        float wheel_angles[wheel_count];
        calculate_steering(forward_speed, speed_kmh, wheel_angles);
        
        apply_drivetrain(forward_speed * 3.6f, dt);
        update_suspension(scene, dt);
        apply_suspension_forces(dt);
        apply_tire_forces(wheel_angles, dt);
        apply_self_aligning_torque();
        apply_aero_and_resistance();
        
        body->addForce(PxVec3(0, -9.81f * cfg.mass, 0), PxForceMode::eFORCE);
        
        // --- wheel speed correction (wide band safety net) ---
        float ground_angular_v = fabsf(forward_speed) / cfg.wheel_radius;
        if (ground_angular_v > 5.0f && input.handbrake < tuning::input_deadzone)
        {
            float sign = (forward_speed >= 0.0f) ? 1.0f : -1.0f;
            float target_w = sign * ground_angular_v;
            for (int i = rear_left; i <= rear_right; i++)
            {
                float wheel_v = fabsf(wheels[i].angular_velocity);
                if (wheel_v < ground_angular_v * 0.3f || wheel_v > ground_angular_v * 1.5f)
                    wheels[i].angular_velocity = lerp(wheels[i].angular_velocity, target_w, exp_decay(tuning::ground_match_rate * 2.0f, dt));
            }
        }
        
        // --- telemetry ---
        if (tuning::log_telemetry)
        {
            float avg_wheel_w = (wheels[rear_left].angular_velocity + wheels[rear_right].angular_velocity) * 0.5f;
            float wheel_surface_speed = avg_wheel_w * cfg.wheel_radius * 3.6f;
            SP_LOG_INFO("rpm=%.0f, speed=%.0f km/h, gear=%s%s, wheel_speed=%.0f km/h, throttle=%.0f%%",
                engine_rpm, speed_kmh, get_gear_string(), is_shifting ? "(shifting)" : "",
                wheel_surface_speed, input.throttle * 100.0f);
        }
        
        // telemetry csv dump
        {
            static FILE* telemetry_file = nullptr;
            static int frame_counter = 0;

            if (tuning::log_to_file)
            {
                if (!telemetry_file)
                {
                    fopen_s(&telemetry_file, "car_telemetry.csv", "w");
                    if (telemetry_file)
                    {
                        fprintf(telemetry_file,
                            "frame,dt,"
                            "engine_rpm,speed_kmh,forward_speed_ms,"
                            "gear,is_shifting,shift_timer,shift_cooldown,"
                            "clutch,throttle,brake,"
                            "rl_ang_vel,rr_ang_vel,rl_slip_ratio,rr_slip_ratio,"
                            "rl_tire_load,rr_tire_load,rl_long_force,rr_long_force,"
                            "rl_grounded,rr_grounded,"
                            "tc_active,tc_reduction\n");
                    }
                    frame_counter = 0;
                }

                if (telemetry_file)
                {
                    float fwd_speed = body->getLinearVelocity().dot(body->getGlobalPose().q.rotate(PxVec3(0, 0, 1)));
                    fprintf(telemetry_file,
                        "%d,%.4f,"
                        "%.1f,%.2f,%.3f,"
                        "%d,%d,%.4f,%.4f,"
                        "%.4f,%.3f,%.3f,"
                        "%.3f,%.3f,%.4f,%.4f,"
                        "%.1f,%.1f,%.1f,%.1f,"
                        "%d,%d,"
                        "%d,%.4f\n",
                        frame_counter, dt,
                        engine_rpm, speed_kmh, fwd_speed,
                        current_gear, is_shifting ? 1 : 0, shift_timer, shift_cooldown,
                        clutch, input.throttle, input.brake,
                        wheels[rear_left].angular_velocity, wheels[rear_right].angular_velocity,
                        wheels[rear_left].slip_ratio, wheels[rear_right].slip_ratio,
                        wheels[rear_left].tire_load, wheels[rear_right].tire_load,
                        wheels[rear_left].longitudinal_force, wheels[rear_right].longitudinal_force,
                        wheels[rear_left].grounded ? 1 : 0, wheels[rear_right].grounded ? 1 : 0,
                        tc_active ? 1 : 0, tc_reduction);

                    if (frame_counter % 200 == 0)
                        fflush(telemetry_file);

                    frame_counter++;
                }
            }
            else if (telemetry_file)
            {
                fclose(telemetry_file);
                telemetry_file = nullptr;
                frame_counter = 0;
            }
        }
    }

    inline float get_speed_kmh()        { return body ? body->getLinearVelocity().magnitude() * 3.6f : 0.0f; }
    inline float get_throttle()         { return input.throttle; }
    inline float get_brake()            { return input.brake; }
    inline float get_steering()         { return input.steering; }
    inline float get_handbrake()        { return input.handbrake; }
    inline float get_suspension_travel(){ return cfg.suspension_travel; }
    
    inline bool is_valid_wheel(int i) { return i >= 0 && i < wheel_count; }
    inline const char* get_wheel_name(int i)
    {
        return is_valid_wheel(i) ? wheel_names[i] : "??";
    }
    
    #define WHEEL_GETTER(name, field) inline float get_wheel_##name(int i) { return is_valid_wheel(i) ? wheels[i].field : 0.0f; }
    WHEEL_GETTER(compression, compression)
    WHEEL_GETTER(slip_angle, slip_angle)
    WHEEL_GETTER(slip_ratio, slip_ratio)
    WHEEL_GETTER(tire_load, tire_load)
    WHEEL_GETTER(lateral_force, lateral_force)
    WHEEL_GETTER(longitudinal_force, longitudinal_force)
    WHEEL_GETTER(angular_velocity, angular_velocity)
    WHEEL_GETTER(rotation, rotation)
    WHEEL_GETTER(temperature, temperature)
    #undef WHEEL_GETTER
    
    inline bool is_wheel_grounded(int i) { return is_valid_wheel(i) && wheels[i].grounded; }
    
    inline float get_wheel_suspension_force(int i)
    {
        if (!is_valid_wheel(i) || !wheels[i].grounded) return 0.0f;
        return spring_stiffness[i] * wheels[i].compression * cfg.suspension_travel;
    }
    
    inline float get_wheel_temp_grip_factor(int i)
    {
        return is_valid_wheel(i) ? get_tire_temp_grip_factor(wheels[i].temperature) : 1.0f;
    }
    
    inline float get_chassis_visual_offset_y()
    {
        const float offset = 0.1f;
        return -(cfg.height * 0.5f + cfg.suspension_height) + offset;
    }
    
    inline void set_abs_enabled(bool enabled) { tuning::abs_enabled = enabled; }
    inline bool get_abs_enabled()             { return tuning::abs_enabled; }
    inline bool is_abs_active(int i)          { return is_valid_wheel(i) && abs_active[i]; }
    inline bool is_abs_active_any()           { for (int i = 0; i < wheel_count; i++) if (abs_active[i]) return true; return false; }
    
    inline void  set_tc_enabled(bool enabled) { tuning::tc_enabled = enabled; }
    inline bool  get_tc_enabled()             { return tuning::tc_enabled; }
    inline bool  is_tc_active()               { return tc_active; }
    inline float get_tc_reduction()           { return tc_reduction; }
    
    inline void set_manual_transmission(bool enabled) { tuning::manual_transmission = enabled; }
    inline bool get_manual_transmission()             { return tuning::manual_transmission; }
    
    inline void begin_shift(int direction)
    {
        is_shifting = true;
        shift_timer = tuning::shift_time;
        last_shift_direction = direction;
    }
    
    inline void shift_up()
    {
        if (!tuning::manual_transmission || is_shifting || current_gear >= tuning::gear_count - 1) return;
        current_gear = (current_gear == 0) ? 1 : current_gear + 1; // from reverse, go to neutral first
        begin_shift(1);
    }
    
    inline void shift_down()
    {
        if (!tuning::manual_transmission || is_shifting || current_gear <= 0) return;
        current_gear = (current_gear == 1) ? 0 : current_gear - 1; // from neutral, go to reverse
        begin_shift(-1);
    }
    
    inline void shift_to_neutral()
    {
        if (!tuning::manual_transmission || is_shifting) return;
        current_gear = 1;
        begin_shift(0);
    }
    
    inline int         get_current_gear()          { return current_gear; }
    inline const char* get_current_gear_string()   { return get_gear_string(); }
    inline float       get_current_engine_rpm()    { return engine_rpm; }
    inline bool        get_is_shifting()           { return is_shifting; }
    inline float       get_clutch()                { return clutch; }
    inline float       get_engine_torque_current() { return get_engine_torque(engine_rpm) * (1.0f + boost_pressure * tuning::boost_torque_mult); }
    inline float       get_redline_rpm()           { return tuning::engine_redline_rpm; }
    inline float       get_max_rpm()               { return tuning::engine_max_rpm; }
    inline float       get_idle_rpm()              { return tuning::engine_idle_rpm; }
    
    inline void  set_turbo_enabled(bool enabled) { tuning::turbo_enabled = enabled; }
    inline bool  get_turbo_enabled()             { return tuning::turbo_enabled; }
    inline float get_boost_pressure()            { return boost_pressure; }
    inline float get_boost_max_pressure()        { return tuning::boost_max_pressure; }
    
    // drs
    inline void set_drs_enabled(bool enabled) { tuning::drs_enabled = enabled; }
    inline bool get_drs_enabled()             { return tuning::drs_enabled; }
    inline void set_drs_active(bool active)   { drs_active = active; }
    inline bool get_drs_active()              { return drs_active; }
    
    // differential type
    inline void set_diff_type(int type)       { tuning::diff_type = PxClamp(type, 0, 2); }
    inline int  get_diff_type()               { return tuning::diff_type; }
    inline const char* get_diff_type_name()
    {
        static const char* names[] = { "Open", "Locked", "LSD" };
        return (tuning::diff_type >= 0 && tuning::diff_type <= 2) ? names[tuning::diff_type] : "?";
    }
    
    // tire wear
    inline float get_wheel_wear(int i)              { return is_valid_wheel(i) ? wheels[i].wear : 0.0f; }
    inline void  reset_tire_wear()                  { for (int i = 0; i < wheel_count; i++) wheels[i].wear = 0.0f; }
    inline float get_wheel_wear_grip_factor(int i)  { return is_valid_wheel(i) ? (1.0f - wheels[i].wear * tuning::tire_grip_wear_loss) : 1.0f; }
    
    inline float get_wheel_brake_temp(int i)       { return is_valid_wheel(i) ? wheels[i].brake_temp : 0.0f; }
    inline float get_wheel_brake_efficiency(int i) { return is_valid_wheel(i) ? get_brake_efficiency(wheels[i].brake_temp) : 1.0f; }
    
    inline void set_wheel_surface(int i, surface_type surface)
    {
        if (is_valid_wheel(i))
            wheels[i].contact_surface = surface;
    }
    inline surface_type get_wheel_surface(int i) { return is_valid_wheel(i) ? wheels[i].contact_surface : surface_asphalt; }
    inline const char* get_surface_name(surface_type surface)
    {
        static const char* names[] = { "Asphalt", "Concrete", "Wet", "Gravel", "Grass", "Ice" };
        return (surface >= 0 && surface < surface_count) ? names[surface] : "Unknown";
    }
    
    inline float get_front_camber() { return tuning::front_camber; }
    inline float get_rear_camber()  { return tuning::rear_camber; }
    inline float get_front_toe()    { return tuning::front_toe; }
    inline float get_rear_toe()     { return tuning::rear_toe; }
    
    inline void set_wheel_offset(int wheel, float x, float z)
    {
        if (wheel >= 0 && wheel < wheel_count)
        {
            wheel_offsets[wheel].x = x;
            wheel_offsets[wheel].z = z;
        }
    }
    
    inline PxVec3 get_wheel_offset(int wheel)
    {
        if (wheel >= 0 && wheel < wheel_count)
            return wheel_offsets[wheel];
        return PxVec3(0);
    }
    
    inline void set_draw_raycasts(bool enabled)   { tuning::draw_raycasts = enabled; }
    inline bool get_draw_raycasts()               { return tuning::draw_raycasts; }
    inline void set_draw_suspension(bool enabled) { tuning::draw_suspension = enabled; }
    inline bool get_draw_suspension()             { return tuning::draw_suspension; }
    inline void set_log_pacejka(bool enabled)     { tuning::log_pacejka = enabled; }
    inline bool get_log_pacejka()                 { return tuning::log_pacejka; }
    
    inline const aero_debug_data& get_aero_debug() { return aero_debug; }
    inline const shape_2d& get_shape_data() { return shape_data_ref(); }
    
    inline void get_debug_ray(int wheel, int ray, PxVec3& origin, PxVec3& hit_point, bool& hit)
    {
        if (wheel >= 0 && wheel < wheel_count && ray >= 0 && ray < debug_rays_per_wheel)
        {
            origin    = debug_rays[wheel][ray].origin;
            hit_point = debug_rays[wheel][ray].hit_point;
            hit       = debug_rays[wheel][ray].hit;
        }
    }
    
    inline void get_debug_suspension(int wheel, PxVec3& top, PxVec3& bottom)
    {
        if (wheel >= 0 && wheel < wheel_count)
        {
            top    = debug_suspension_top[wheel];
            bottom = debug_suspension_bottom[wheel];
        }
    }
    
    inline int get_debug_rays_per_wheel() { return debug_rays_per_wheel; }

    // debug window - call this during tick to display car telemetry
    inline void debug_window(bool* visible = nullptr)
    {
        if (!spartan::Engine::IsFlagSet(spartan::EngineMode::EditorVisible))
            return;
        if (visible && !*visible)
            return;
        if (!body)
            return;
        if (!ImGui::Begin("Car Telemetry", visible, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoResize))
        {
            ImGui::End();
            return;
        }

        // car preset selector
        if (ImGui::BeginCombo("Car", tuning::spec.name))
        {
            for (int i = 0; i < preset_count; i++)
            {
                bool is_selected = (i == active_preset_index);
                if (ImGui::Selectable(preset_registry[i].name, is_selected))
                {
                    active_preset_index = i;
                    load_car(preset_registry[i].factory());
                }
                if (is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        ImGui::Separator();
        float speed = get_speed_kmh();
        ImGui::Text("Speed: %.1f km/h", speed);
        ImGui::Text("Gear: %s %s", get_gear_string(), is_shifting ? "(shifting)" : "");
        ImGui::Text("RPM: %.0f / %.0f", engine_rpm, tuning::engine_redline_rpm);

        float rpm_fraction = engine_rpm / tuning::engine_max_rpm;
        ImVec4 rpm_color = (engine_rpm > tuning::engine_redline_rpm) ? ImVec4(1, 0, 0, 1) : ImVec4(0.2f, 0.8f, 0.2f, 1);
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, rpm_color);
        ImGui::ProgressBar(rpm_fraction, ImVec2(-1, 0), "");
        ImGui::PopStyleColor();

        ImGui::Text("Throttle: %.0f%%  Brake: %.0f%%  Clutch: %.0f%%", input.throttle * 100, input.brake * 100, clutch * 100);

        ImGui::Separator();
        ImGui::Text("Driver Aids:");
        ImGui::Text("  ABS: %s %s", tuning::abs_enabled ? "ON" : "OFF", is_abs_active_any() ? "(active)" : "");
        ImGui::Text("  TC:  %s %s", tuning::tc_enabled ? "ON" : "OFF", tc_active ? "(active)" : "");
        if (tuning::turbo_enabled)
            ImGui::Text("  Boost: %.2f bar", boost_pressure);
        if (tuning::drs_enabled)
            ImGui::Text("  DRS: %s", drs_active ? "OPEN" : "closed");
        ImGui::Text("  Diff: %s", get_diff_type_name());

        ImGui::Separator();
        if (ImGui::BeginTable("wheels", 8, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
        {
            ImGui::TableSetupColumn("Wheel");
            ImGui::TableSetupColumn("Grounded");
            ImGui::TableSetupColumn("Load (N)");
            ImGui::TableSetupColumn("Slip Ratio");
            ImGui::TableSetupColumn("Slip Angle");
            ImGui::TableSetupColumn("Tire Temp");
            ImGui::TableSetupColumn("Brake Temp");
            ImGui::TableSetupColumn("Wear");
            ImGui::TableHeadersRow();

            for (int i = 0; i < wheel_count; i++)
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn(); ImGui::Text("%s", wheel_names[i]);
                ImGui::TableNextColumn(); ImGui::Text("%s", wheels[i].grounded ? "yes" : "no");
                ImGui::TableNextColumn(); ImGui::Text("%.0f", wheels[i].tire_load);
                ImGui::TableNextColumn(); ImGui::Text("%.3f", wheels[i].slip_ratio);
                ImGui::TableNextColumn(); ImGui::Text("%.2f", wheels[i].slip_angle * 57.2958f); // to degrees
                ImGui::TableNextColumn();
                {
                    float temp = wheels[i].temperature;
                    ImVec4 color = (temp > tuning::tire_optimal_temp + 20) ? ImVec4(1, 0.5f, 0, 1) :
                                   (temp < tuning::tire_optimal_temp - 20) ? ImVec4(0.5f, 0.5f, 1, 1) :
                                   ImVec4(0.2f, 1, 0.2f, 1);
                    ImGui::TextColored(color, "%.0f C", temp);
                }
                ImGui::TableNextColumn();
                {
                    float temp = wheels[i].brake_temp;
                    ImVec4 color = (temp > tuning::brake_fade_temp) ? ImVec4(1, 0, 0, 1) :
                                   (temp > tuning::brake_optimal_temp) ? ImVec4(1, 0.5f, 0, 1) :
                                   ImVec4(0.8f, 0.8f, 0.8f, 1);
                    ImGui::TextColored(color, "%.0f C", temp);
                }
                ImGui::TableNextColumn();
                {
                    float wear_pct = wheels[i].wear * 100.0f;
                    ImVec4 color = (wear_pct > 70.0f) ? ImVec4(1, 0, 0, 1) :
                                   (wear_pct > 40.0f) ? ImVec4(1, 0.7f, 0, 1) :
                                   ImVec4(0.5f, 1, 0.5f, 1);
                    ImGui::TextColored(color, "%.1f%%", wear_pct);
                }
            }
            ImGui::EndTable();
        }

        ImGui::Separator();
        if (ImGui::BeginTable("forces", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
        {
            ImGui::TableSetupColumn("Wheel");
            ImGui::TableSetupColumn("Lateral (N)");
            ImGui::TableSetupColumn("Longitudinal (N)");
            ImGui::TableSetupColumn("Suspension (N)");
            ImGui::TableHeadersRow();

            for (int i = 0; i < wheel_count; i++)
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn(); ImGui::Text("%s", wheel_names[i]);
                ImGui::TableNextColumn(); ImGui::Text("%.0f", wheels[i].lateral_force);
                ImGui::TableNextColumn(); ImGui::Text("%.0f", wheels[i].longitudinal_force);
                ImGui::TableNextColumn(); ImGui::Text("%.0f", get_wheel_suspension_force(i));
            }
            ImGui::EndTable();
        }

        if (aero_debug.valid)
        {
            ImGui::Separator();
            ImGui::Text("Aerodynamics:");
            ImGui::Text("  Ride Height: %.3f m", aero_debug.ride_height);
            ImGui::Text("  Yaw Angle: %.1f deg", aero_debug.yaw_angle * 57.2958f);
            ImGui::Text("  Ground Effect: %.2fx", aero_debug.ground_effect_factor);
            ImGui::Text("  Drag: %.0f N", aero_debug.drag_force.magnitude());
            ImGui::Text("  Downforce F/R: %.0f / %.0f N", aero_debug.front_downforce.magnitude(), aero_debug.rear_downforce.magnitude());
        }

        ImGui::End();
    }
}
