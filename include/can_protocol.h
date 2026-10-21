#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H
#include <stdint.h>


#define Control_ID        0x100
#define VehicleStatus_ID  0x101
#define MotorTemps_ID     0x200
#define MotorData_ID      0x201
#define BMSData_ID        0x300
#define BMSExtra_ID       0x301
#define GPS_ID            0x400
#define Position_ID       0x401

#pragma pack(push, 1)
typedef struct {
    int8_t throttle;
    uint8_t enable;
    uint8_t alive_counter:4;
    uint8_t padding:4;
} control_msg_t;
#pragma pack(pop)


typedef enum{
    VEHICLE_STATE_INIT = 0,
    VEHICLE_STATE_STANDBY = 1,
    VEHICLE_STATE_READY = 2,
    VEHICLE_STATE_DRIVE = 3,
    vEHICLE_STATE_SAFE_STATE = 4
}vehicle_state_t;

#pragma pack(push, 1)
typedef struct {
    vehicle_state_t state:8;      
    uint8_t throttle_derating_active : 1;
    uint8_t reserved_flags           : 7;               
    uint16_t fault_mask;         
} vehicle_status_msg_t;
#pragma pack(pop)

typedef struct {
    int16_t motor_temp;
    int16_t controller_temp;
} motor_temps_msg_t;

typedef struct {
    int16_t rpm;
    int16_t current;
    int16_t torque;
} motor_data_msg_t;

typedef struct {
    int16_t voltage;
    int16_t current;
    uint16_t soc;
} bms_data_msg_t;

typedef struct {
    int16_t temperature;
    int16_t mppt_current;
    int16_t fan_rpm;
} bms_extra_msg_t;

typedef struct {
    int32_t latitude;
    int32_t longitude;
} gps_msg_t;


typedef enum {
    GPS_FIX_NONE = 0,
    GPS_FIX_2D   = 1,
    GPS_FIX_3D   = 2
} gps_fix_quality_t;

#pragma pack(push, 1)
typedef struct {
    uint16_t speed_over_ground; 
    uint16_t course;            
    gps_fix_quality_t  fix_quality:8;
    uint8_t  satellites;        
    uint16_t reserved;          
} position_msg_t;
#pragma pack(pop)


#endif