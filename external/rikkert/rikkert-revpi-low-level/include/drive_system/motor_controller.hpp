#pragma once

#include "RoboClaw.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <functional>
#include "states.hpp"

/**
 * Class for the motor controller object
*/
class MotorController
{
public:
	/**
	 * Constructor for motor controller class
	 * @param enabled is whether the motors are enabled or not
	 * @param stop is whether the system is fully stopped
	 * @param cur_state is a container for the current state of the system 
	*/
	MotorController(std::shared_ptr<bool> enabled, 
					std::shared_ptr<bool> stop, 
					std::shared_ptr<systemState> cur_state);
	/**
	 * Destructor of the motor controller class
	*/
	~MotorController();
	
	/**
	 * Reads the data from the mobile controller
	*/
	void dataReadout();

	/**
	 * Processes incoming commands to direct the wheels
	*/
	void controlMotors();

	/**
	 * Loop to run the control functions of the motor controller class
	*/
	void driveSystemLoop();


	//Connection parameters
	std::string comm_port_; 	//Name of usb port connected with the motor controller
	long baudrate_;				//Baudrate of usb connection
	uint8_t timeout_;			//Timeout of the usb connection
	uint8_t address_;			//Physical address of the motor controller
	RoboClaw driver_;			//Initialisation of roboclaw driver lib
	int32_t ticks_left_;		//Amount of ticks from the left wheel encoder
	int32_t ticks_right_;		//Amount of ticks from the right wheel encoder
	uint16_t voltage_i_;		//Voltage level of the onboard battery
	double voltage_d_; 
	uint16_t temp_i_;
	double temp_d_;
	uint32_t speed_left_;
	uint32_t speed_right_;
	uint8_t reset_encoders_ = 0;
	uint8_t reset_done_ = 1;

	std::shared_ptr<bool> enabled_;
	std::shared_ptr<bool> stop_;
	std::shared_ptr<bool> ems_;
	std::shared_ptr<systemState> cur_state_;
	systemState prev_state_;

	bool init_error_ = false;

	double v_l_ = 0.0;
	double v_r_ = 0.0;

	// std::shared_ptr<std::thread> t_1_;
};
