#pragma once
#include <cstdint>

struct StampedDouble {
    double timestamp;      
    double time_variance;  
    double value;         

    // Add a constructor that takes a float/double value
    StampedDouble(double val = 0.0) : timestamp(0.0), time_variance(0.0), value(val) {}
    
    // Add a full constructor
    StampedDouble(double ts, double var, double val) 
        : timestamp(ts), time_variance(var), value(val) {}
};

struct ImuSignal {
    double timestamp;
    float acceleration_x, acceleration_y, acceleration_z;
    float angular_velocity_x, angular_velocity_y, angular_velocity_z;
    float magnetic_field_x, magnetic_field_y, magnetic_field_z;
    float roll, pitch, yaw;

    // Add a empty constructor
    ImuSignal() : timestamp(0.0), acceleration_x(0.0), acceleration_y(0.0), acceleration_z(0.0), angular_velocity_x(0.0), angular_velocity_y(0.0), angular_velocity_z(0.0), magnetic_field_x(0.0), magnetic_field_y(0.0), magnetic_field_z(0.0), roll(0.0), pitch(0.0), yaw(0.0) {}
}; 

struct CraneStates {
    double polar_angle;
    double polar_radius;
    double polar_height;
    double polar_angle_vel;
    double polar_radius_vel;
    double polar_height_vel;
};

struct StampedCraneStates {
    double timestamp;
    CraneStates crane_states;
};

struct BatteryData {
    float voltage;        // in Volts
    float current;        // in Amperes
    float percentage;     // in %
    float remaining_capacity; // in mAh
    int8_t remaining_hours;   // estimated remaining time in hours
    int8_t remaining_minutes; // estimated remaining time in minutes
    int charging_status;  // 0: idle, 1: discharging, 2: charging

    // Add a constructor that initializes all values to zero
    BatteryData() : voltage(0.0), current(0.0), percentage(0.0), remaining_hours(0), remaining_minutes(0), charging_status(0) {}
};

struct StampedBatteryData {
    double timestamp;
    BatteryData battery_data;
    int battery_management_status; // 0: disconnected, 1: idle, 2: stopping, 3: booting, 4: running, 5: error
    int time_to_shutdown; // in seconds
};