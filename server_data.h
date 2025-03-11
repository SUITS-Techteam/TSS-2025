#ifndef SERVER_DATA_H
#define SERVER_DATA_H

#include <stdbool.h>
#include <time.h>
#include <stdint.h>
#include "cJSON.h"
#include <stdlib.h>
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////////
//                                  Simulation Values
///////////////////////////////////////////////////////////////////////////////////

#define BATT_TIME_CAP 21600
#define BATT_FILL_RATE BATT_TIME_CAP / 150.0f

#define OXY_TIME_CAP 10800
#define OXY_PRESSURE_CAP 3000.0f
#define OXY_FILL_RATE 0.8f

#define DEPRESS_TIME 15
#define RESTING_HEART_RATE 90.0f
#define EVA_HEART_RATE 140.0f
#define HAB_OXY_PRESSURE   3.0723f
#define HAB_CO2_PRESSURE   0.0059f
#define HAB_OTHER_PRESSURE 11.5542f
#define SUIT_OXY_PRESSURE   4.0f
#define SUIT_CO2_PRESSURE   0.001f
#define SUIT_OTHER_PRESSURE 0.0f

#define SUIT_OXY_CONSUMPTION 0.1f
#define SUIT_CO2_PRODUCTION 0.1f

#define SUIT_FAN_SPIN_UP_RATE 0.9f
#define SUIT_FAN_RPM 30000.0f
#define SUIT_FAN_ERROR_RPM 5000.0f

#define SUIT_SCRUBBER_CAP 1.0f
#define SUIT_SCRUBBER_FILL_RATE 0.8f
#define SUIT_SCRUBBER_FLUSH_RATE 0.85f

#define SUIT_COOLANT_NOMINAL_TEMP 65.0f
#define SUIT_COOLANT_NOMINAL_PRESSURE 500.0f

//////////////////////////////////////////////////////////////// Pressurized Rover Values ////////////////////////////////////////////////////////////////////////////////////////

#define MAX_LIDAR_SIZE 13

// (Power consumption rates are in battery percentage per second consumed)

#define THROTTLE_CONSUMPTION_RATE 0.01f
#define THROTTLE_MAX_ABS_VALUE 100.0f

#define PASSIVE_POWER_CONSUMPTION_RATE 0.0005f

#define MAX_SOLAR_PANEL_DUST_ACCUM 100.0f
#define PANEL_DUST_ACCUM_RATE 0.01f
#define PANEL_DUST_WIPER_CONSUMPTION_RATE 0.001f
#define PANEL_DUST_WIPER_CLEAN_RATE 10.0f
#define SOLAR_PANEL_RECHARGE_RATE 0.01f

#define EXTERNAL_LIGHTS_CONSUMPTION_RATE 0.001f
#define INTERNAL_LIGHTS_CONSUMPTION_RATE 0.0005f

#define CO2_SCRUBBER_CONSUMPTION_RATE 0.001f

#define AC_COOLING_CONSUMPTION_RATE 0.005f
#define AC_HEATING_CONSUMPTION_RATE 0.005f

// Temperature values (Cooling/heating rates modify k value in Newton's cooling law formula)
#define E 2.718281828f

#define MOON_HIGH_TEMP_RATE -0.1f
#define MOON_HIGH_TEMPERATURE 121.0f
#define MOON_LOW_TEMP_RATE -0.1f
#define MOON_LOW_TEMPERATURE -133.0f

#define CABIN_HIGH_RATE -0.00005f
#define CABIN_HIGH_TEMPERATURE 121.0f
#define CABIN_LOW_RATE -0.00005f
#define CABIN_LOW_TEMPERATURE -133.0f

#define CABIN_COOLING_RATE -0.004f
#define CABIN_COOLING_TEMP 21.0f // Cooling target temperature
#define CABIN_HEATING_RATE -0.004f
#define CABIN_HEATING_TEMP 23.0f // Heating target temperature

#define NOMINAL_CABIN_TEMPERATURE 22.0f
#define NOMINAL_CABIN_PRESSURE 4.0f

#define NOMINAL_COOLANT_LEVEL 44.5f
#define NOMINAL_COOLANT_TANK 100.0f
#define NOMINAL_COOLANT_PRESSURE 500.0f

//Coolant and Oxygen values/rates

#define PR_COOLANT_TANK_CAP 100.0f
#define PR_COOLANT_TANK_DRAIN_RATE 0.05f
#define PR_PASSIVE_COOLANT_DRAIN 0.0001f
#define PR_COOLANT_MIN_PRESSURE 0.0f

#define PR_OXYGEN_TANK_DRAIN_RATE 0.05f
#define PR_PASSIVE_OXYGEN_DRAIN 0.0001f
#define PR_OXYGEN_PRESSURE_CAP 3000.0f
#define PR_OXYGEN_TANK_CAP 100.0f

//Fans
#define PR_FAN_SPIN_UP_RATE 0.9f
#define PR_FAN_RPM 30000.0f

//Other
#define exp7           10000000i64   //1E+7     
#define exp9         1000000000i64   //1E+9
#define w2ux 116444736000000000i64   //1.jan1601 to 1.jan1970

//

///////////////////////////////////////////////////////////////////////////////////
//                                  Structs
///////////////////////////////////////////////////////////////////////////////////

struct uia_data_t {

    // External Device sends Switch Values to backend
    bool eva1_power;
    bool eva1_oxy;
    bool eva1_water_waste;
    bool eva1_water_supply;

    bool eva2_power;
    bool eva2_oxy;
    bool eva2_water_waste;
    bool eva2_water_supply;

    bool oxy_vent;
    bool depress;

};

struct dcu_data_t {
    
    // EVA1 Switches
    bool eva1_batt;
    bool eva1_oxy;
    bool eva1_comm;
    bool eva1_fan;
    bool eva1_pump;
    bool eva1_co2;

    // EVA2 Switches
    bool eva2_batt;
    bool eva2_oxy;
    bool eva2_comm;
    bool eva2_fan;
    bool eva2_pump;
    bool eva2_co2;

};

struct imu_data_t {

    // EVA1 IMU
    float eva1_posx;
    float eva1_posy;
    float eva1_heading;
    
    // EVA2 IMU
    float eva2_posx;
    float eva2_posy;
    float eva2_heading;

};

struct rover_data_t {

    float pos_x;
    float pos_y;
    float poi_1_x;
    float poi_1_y;
    float poi_2_x;
    float poi_2_y;
    float poi_3_x;
    float poi_3_y;
    bool  ping;
    // other stuff

};

struct spec_data_t {

    // External Device sends Rock ID Values to backend
    int eva1_rock_id;
    int eva2_rock_id;

};

struct comm_data_t {

    bool comm_tower_online;

};

struct telemetry_data_t {

    // Battery
    float batt_time;                // The number of seconds of power the tank has

    // Oxy Tanks
    float oxy_pri_tank_fill;        // The number of seconds of oxygen the tank has
    float oxy_sec_tank_fill;
    float oxy_pri_tank_pressure;    // The pressure of the oxygen in the tank
    float oxy_sec_tank_pressure;

    // Life Support
    float heart_rate;               // A random value within a range that determines the oxy_consumption and co2_production
    float oxy_consumption;
    float co2_production;

    float suit_oxy_pressure;
    float suit_co2_pressure;
    float suit_other_pressure;      // Mostly nitrogen in the air (this value is set to zero during depress)
    int   depress_time;

    float helmet_co2_pressure;      // co2_production goes straight to the helmet, the fans push it to the rest of the suit.
    float fan_pri_rpm;
    float fan_sec_rpm;
    float scrubber_A_co2_captured;
    float scrubber_B_co2_captured;

    // Temperature Regulation
    float temperature;
    float coolant_tank;
    float coolant_liquid_pressure;
    float coolant_gaseous_pressure;

    // Communications
    int com_channel;

};

// Pressurized Rover Data
struct pr_data_t {

    bool ac_heating;
    bool ac_cooling;
    bool lights_on;
    bool internal_lights_on;

    // Drive Commands
    bool brakes;
    float throttle;
    float steering;

    // Data From Unreal
    bool in_sunlight; 
    float current_pos_x;
    float current_pos_y;
    float current_pos_alt;
    float heading;
    float pitch;
    float roll;
    float distance_traveled;
    float speed;
    float surface_incline;
    float lidar[MAX_LIDAR_SIZE];

    // Life Support
    float oxygen_levels;
    bool co2_scrubber;
    float ac_fan_pri;
    float ac_fan_sec;
    bool fan_pri;
    float cabin_pressure;
    float cabin_temperature;
    float oxygen_tank;
    float oxygen_pressure;

    // Power System
    float battery_level;
    float power_consumption_rate;
    float solar_panel_efficiency;

    // Thermal System
    float external_temp;
    float pr_coolant_level;
    float pr_coolant_pressure;
    float pr_coolant_tank;
    float radiator;

    // Propulsion
    float motor_power_consumption;

    // Environmental
    bool dust_wiper;
    float terrain_condition;
    float solar_panel_dust_accum;

    // Mission Info
    float mission_elapsed_time;
    float mission_planned_time;
    float point_of_no_return;
    float distance_from_base;
    bool sim_running;
    bool sim_paused;
    bool sim_completed;

    // Destination
    bool switch_dest;
    float dest_x;
    float dest_y;
    float dest_z;

    float latitude;
    float longitude;
};

struct pr_sim_data_t {

    float old_k;
    float target_temp;
    float object_temp;
    uint32_t start_time;

    float external_target_temp;
    float external_object_temp;
    uint32_t external_start_time;

};

struct eva_failures_t {
    
    bool oxy_error;
    bool fan_error;
    bool pump_error;
    bool power_error;

};

struct eva_data_t {

    // General EVA Info
    bool     started;
    bool     paused;
    bool     completed;
    uint32_t total_time;

    // time at each station
    bool     started_UIA;
    bool     completed_UIA;
    uint32_t time_at_UIA;
  
    bool     started_DCU;
    bool     completed_DCU;
    uint32_t time_at_DCU;
  
    bool     started_ROVER;
    bool     completed_ROVER;
    uint32_t time_at_ROVER;
  
    bool     started_SPEC;
    bool     completed_SPEC;
    uint32_t time_at_SPEC;

    // Suit Simulated Telemetry 
    struct telemetry_data_t eva1;
    struct telemetry_data_t eva2;

    // Procedure Info
    int procedure_step;

};

#define NUMBER_OF_TEAMS 11

struct backend_data_t {

    uint32_t start_time;
    uint32_t server_up_time;
    int running_pr_sim;
    bool pr_sim_paused;

    // Data for each team
    struct eva_data_t       evas[NUMBER_OF_TEAMS];
    struct pr_sim_data_t    pr_sim[NUMBER_OF_TEAMS];
    struct pr_data_t        p_rover[NUMBER_OF_TEAMS];

    // struct telemetry_data_t telemetry[NUMBER_OF_TEAMS];

    // Global Data
    struct uia_data_t       uia;
    struct dcu_data_t       dcu;
    struct imu_data_t       imu;
    struct rover_data_t     rover;
    struct spec_data_t      spec;
    struct comm_data_t      comm;

    // Simulated Data
    struct eva_failures_t   failures;
};



///////////////////////////////////////////////////////////////////////////////////
//                                 Functions
///////////////////////////////////////////////////////////////////////////////////

// initialize the backend with default values and clean up at the end
struct backend_data_t* init_backend();
void cleanup_backend(struct backend_data_t*  backend);
void reset_telemetry(struct telemetry_data_t* telemetry, float seed);
void reset_pr_telemetry(struct backend_data_t* backend, int teamIndex);

// build json files when values update
bool build_json_meta_data(struct backend_data_t* backend);
bool build_json_uia      (struct uia_data_t* uia);
bool build_json_dcu      (struct dcu_data_t* dcu);
bool build_json_imu      (struct imu_data_t* imu);
bool build_json_rover    (struct rover_data_t* rover);
bool build_json_spec     (struct spec_data_t* spec);
bool build_json_comm     (struct comm_data_t* comm);
bool build_json_error    (struct eva_failures_t* error);
bool build_json_eva      (struct eva_data_t* eva, int team_index, bool completed);
bool build_json_telemetry(struct eva_data_t* eva, int team_index, bool completed);
bool build_json_pr_telemetry(struct pr_data_t* rover, int team_index, bool completed);

// Update locally stored variables
bool update_uia      (char* request_content, struct uia_data_t* uia);
bool update_dcu      (char* request_content, struct dcu_data_t* dcu);
bool update_imu      (char* request_content, struct imu_data_t* imu);
bool update_rover    (char* request_content, struct rover_data_t* rover);
bool update_spec     (char* request_content, struct spec_data_t* spec);
bool update_comm     (char* request_content, struct comm_data_t* comm);
bool update_error    (char* request_content, struct eva_failures_t* error);
bool update_eva      (char* request_content, struct backend_data_t* backend);
bool update_telemetry(struct telemetry_data_t* telemetry, uint32_t eva_time, struct backend_data_t* backend, bool isEVA1);
bool update_pr_telemetry(char* request_content, struct backend_data_t* backend, int teamIndex);
bool update_resource(char* request_content, struct backend_data_t* backend); // Entry Point to all other update functions

// Simulate the backend
float fourier_sin(float x);
float randomized_sine_value(float x, float avg, float amp, float phase, float freq);
void simulate_telemetry(struct telemetry_data_t telemetry);
void simulate_pr_telemetry(struct pr_data_t* p_rover, uint32_t server_time, struct backend_data_t* backend);
void simulate_cabin_temperature(struct backend_data_t* backend);
void simulate_external_temperature(struct backend_data_t* backend);
void simulate_backend  (struct backend_data_t* backend);

// UDP GET functions
bool udp_get_dcu(unsigned int command, unsigned char* data);
bool udp_get_uia(unsigned int command, unsigned char* data);
bool udp_get_spec(unsigned int command, unsigned char* data);
bool udp_get_imu(unsigned int command, unsigned char* data);
bool udp_get_error(unsigned int command, unsigned char* data);
bool udp_get_rover(unsigned int command, unsigned char* data);
bool udp_get_comm(unsigned char* data);
bool udp_get_teams(unsigned char* request_content);
bool udp_get_telemetry(unsigned int command, unsigned int team_number, unsigned char* data);
bool udp_get_pr_telemetry(unsigned int command, unsigned char* data, struct backend_data_t* backend);
bool udp_get_eva(unsigned int command, unsigned int team_number, unsigned char* data);
void handle_udp_get_request(unsigned int command, unsigned char* data, struct backend_data_t* backend);
void udp_get_pr_lidar(char* lidar, struct backend_data_t* backend);

// UDP POST functions
bool udp_post_pr_telemetry(unsigned int command, unsigned char* data, struct backend_data_t* backend);
void udp_post_pr_lidar(char* request, struct backend_data_t* backend, int received_bytes);
void handle_udp_post_request(unsigned int command, char* data, char* request, struct backend_data_t* backend, int received_bytes);

//Helper functions
void reverse_bytes(unsigned char* bytes);
bool big_endian();

#endif