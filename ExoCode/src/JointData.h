/**
 * @file JointData.h
 *
 * @brief Declares a class used to store data for joint to access 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef JointData_h
#define JointData_h

#include "Arduino.h"

#include "MotorData.h"
#include "ControllerData.h"
#include "ParseIni.h"
#include "Board.h"

#include <stdint.h>
#include <queue>

//Forward declaration
class ExoData;

/**
 * @brief Class to store information related to joint.
 * 
 */
class JointData {
	public:
        JointData(config_defs::joint_id id, uint8_t* config_to_send, float joint_RoM, bool flip_ankle_angle, float torque_offset);
        
        /**
         * @brief Reconfigures the the joint data if the configuration changes after constructor called.
         * 
         * @param configuration array
         */
        void reconfigure(uint8_t* config_to_send);
        
        config_defs::joint_id id;       /**< Id of the joint */
        MotorData motor;                /**< Data for the motor attached to the joint */
        ControllerData controller;      /**< Data for the controller running the joint */
        float torque_reading;           /**< Calibrated reading from the torque sensor */ 
        float previous_torque_reading;   /**< Previous calibrated reading from the torque sensor, used for torque fault detection */
        bool is_left;                   /**< If the leg is left */
        bool flip_direction;            /**< If true, invert the sign of the torque readings for the current joint (for example, left ankle) */
        bool is_used;                   /**< Stores if the joint is used, joint is skipped if it is not used */
        bool calibrate_torque_sensor;   /**< Flag for if we should calibrate the torque sensor. */ 
        bool calibrate_angle_sensor;    /**< Flag for if we should calibrate the angle sensor. */
        
        float position;                 /**< The position of the joint, this should be motor position with compensated for gearing. */
        float velocity;                 /**< The velocity of the joint, this should be motor velocity with compensated for gearing. */

        float joint_position;                       /**< The position of the joint, after any transmission */
        float joint_global_angle;                   /**< The angle of the joint relative to the ground */
        float prev_joint_position;                  /**< The previous position of the joint, after any transmission */
        float joint_velocity;                       /**< The velocity of the joint, after any transmission */
        const float joint_position_alpha = 1;
        const float joint_velocity_alpha = 0.05f;
		const float joint_RoM;                      /**< Joint Range of Motion */
		bool do_flip_angle;                         /**< If true invert the angle */

		float torque_reading_microSD;				/**< Torque reading based on the stored offset on the SD card */
		const float torque_offset;					/**< Torque offset pulled from the SD card for the current torque sensor */
		float torque_offset_reading;				/**< True torque offset for the current torque sensor; note that the true offset may change */

        //Torque tracking check
        float torque_output_alpha = 0.2;            /**< Low pass to describe the lag of the low level controller relative to setpoint. */
        float post_transmission_torque = 0;         /**< Torque after torque_output_alpha. */

        //Torque clamps
        float max_sensor_torque = 60;                       /**< Maximum value of the the filtered torque reading. */
        float max_desired_torque = 60;                  /**< Maximum value of the desired torque from the high level controller. */
        float max_driver_torque = 60;                   /**< Maximum torque that we will allow driver to request. */

        //Torque input faults
        float max_sensor_torque_rate = 5;              /**< Maximum rate of change of the torque reading. */
        uint8_t max_sensor_torque_rate_cycle_limit = 50;     /**< Number of cycles that the torque rate can be above the threshold before flagging an error. */
        uint8_t max_sensor_torque_cycle_limit = 50;          /**< Number of cycles that the torque reading can be above the threshold before flagging an error. */

        //Torque output faults
        float max_desired_torque_rate = 5;           /**< Maximum rate of change of the desired torque. */
        uint8_t max_desired_torque_rate_cycle_limit = 50;  /**< Number of cycles that the desired torque rate can be above the threshold before flagging an error. */
        uint8_t max_desired_torque_cycle_limit = 50;         /**< Number of cycles that the desired torque can be above the threshold before flagging an error. */
        float max_driver_torque_rate = 5;            /**< Maximum rate of change of the driver requested torque. */
        uint8_t max_driver_torque_rate_cycle_limit = 50;   /**< Number of cycles that the driver requested torque rate can be above the threshold before flagging an error. */
        uint8_t max_driver_torque_cycle_limit = 50;         /**< Number of cycles that the driver requested torque can be above the threshold before flagging an error. */
        uint8_t static_driver_torque_cycle_limit = 50;     /**< Number of cycles that the driver requested torque can be static and non zero before flagging an error. */

        //Fault cycle counters
        uint8_t max_sensor_torque_cycle_count = 0;
        uint8_t max_desired_torque_cycle_count = 0;
        uint8_t max_driver_torque_cycle_count = 0;
        uint8_t max_sensor_torque_rate_cycle_count = 0;
        uint8_t max_desired_torque_rate_cycle_count = 0;
        uint8_t max_driver_torque_rate_cycle_count = 0;
        uint8_t static_driver_torque_cycle_count = 0;

        //Torque variance check
        const int torque_failure_count_max = 2;         /**< Amount of samples outside of error bounds. */
        const int torque_std_dev_multiple = 10;         /**< Number of standard deviations from the mean to be considered an error. */
        const float torque_data_window_max_size = 100;  /**< Number of samples to use for the standard deviation calculation. */
        std::queue<float> torque_data_window;           /**< Queue to store torque sensor values. */
        int torque_failure_count = 0;                   /**< Number of successive samples outside of error bounds. */

        //Transmission efficiency check
        const float transmission_efficiency_threshold = 0.001;
        const float motor_torque_smoothing = 0.1;
        const float torque_error_smoothing = 1;
        float smoothed_motor_torque = 0;
        const float close_to_zero_tolerance = 0.1;
        float torque_error = 0;

};


#endif