#pragma once

enum class ioState
{
	ERROR = -1,		//Error while reading IO
	LOW = 0,		//IO set to low
	HIGH = 1		//IO set to high
};

enum class systemState
{
	ERROR = -1,			//Configuration error
	RUNNING = 0,		//Emergency stop pressed, system inactive
	EMS = 1, 			//No emergency stop pressed,system is active
	ACKNOWLEDGE = 2,	//Acknowledgement required
};
