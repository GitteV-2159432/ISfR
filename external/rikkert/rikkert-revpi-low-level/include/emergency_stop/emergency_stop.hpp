#pragma once

#include <iostream>
#include <string.h>
#include <math.h>
#include <thread>
#include <chrono>
#include <csignal>
#include <functional>
#include "emergency_stop/piControlIf.hpp"
#include "states.hpp"

class EmergencyStop
{
public:
	/**
	 * Constructor for emergency stop class
	 * @param enables is whether or not the motors are enabled
	 * @param stop is whether or not the system is fully stopped
	*/
	EmergencyStop(std::shared_ptr<bool> enabled, 
				  std::shared_ptr<bool> stop);
	/**
	 * Destructor of emergency stop class
	*/
	~EmergencyStop();

	/**
	 * Function to write an output of the Revpi
	*/
	bool writeVariable(char* pszVariableName, 
					   uint32_t i32uValue);

	/**
	 * Function to read an input from the Revpi
	*/
	ioState readInput(char* input);

	/**
	 * Loop to run the emergency stop functions
	*/
	void emergencyStopLoop();
	
	//Variables
	bool led_red_, led_green_, led_blue_, led_orange_;		//Booleans to check if an LED is enabled or disabled
	char *pred_, *pgreen_, *pblue_, *porange_;				//Addresses of LED lights
	char *penable_, *pemergency_input_;						//Addresses of the enable relay and EMS
	std::shared_ptr<bool> enabled_;
	std::shared_ptr<bool> stop_;
	std::shared_ptr<systemState> cur_state_;				//Current state of the platform
	std::shared_ptr<systemState> prev_state_;				//Previous state of the platform
	piControl revPi_;										//RevPi control object

	bool init_error_ = false;
	uint8_t acknowledged_ = 0;
	
	// std::shared_ptr<std::thread> t_1_;
};

