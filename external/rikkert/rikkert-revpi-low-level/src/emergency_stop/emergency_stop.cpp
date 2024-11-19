#include "emergency_stop/emergency_stop.hpp"

// bool stop = false;

EmergencyStop::EmergencyStop(std::shared_ptr<bool> enabled, 
							 std::shared_ptr<bool> stop)
{
	enabled_ = enabled;
	stop_ = stop;

	//Init addresses
	pgreen_ = (char*)"O_1";
	pred_ = (char*)"O_2";
	porange_ = (char*)"O_3";
	pblue_ = (char*)"O_4";
	penable_ = (char*)"O_5";
	pemergency_input_ = (char*)"I_1";
	//Init LED states
	led_red_ = false;
	led_green_ = false;
	led_blue_ = false;
	led_orange_ = false;
	//Init states
	cur_state_ =  std::make_shared<systemState>(systemState::ERROR);
	prev_state_ = std::make_shared<systemState>(systemState::ERROR);

	// //Read state of emergency input
	// ioState ems = readInput(pemergency_input_);
	// switch(ems)
	// {
	// 	case ioState::ERROR:
	// 		std::cerr << "Error during IO readout!" << std::endl;
	// 		return;
	// 	case ioState::LOW:
	// 		cur_state_ = systemState::EMS;
	// 		break;
	// 	case ioState::HIGH:
	// 		cur_state_ = systemState::RUNNING;
	// 		break;
	// 	default: 
	// 		std::cerr << "Unknown IO return value" << std::endl;
	// 		return;
	// }		
	revPi_ = piControl();
}
EmergencyStop::~EmergencyStop()
{
	//Turn off lights
	writeVariable(pred_, 0);
	writeVariable(pgreen_, 0);
	writeVariable(pblue_, 0);
	writeVariable(porange_, 0);
}

ioState EmergencyStop::readInput(char* input)
{
	int rc;
	SPIVariable sPiVariable;
	SPIValue sPiValue;
	uint8_t i8uValue;
	uint16_t i16uValue;
	uint32_t i32uValue;
	
	strncpy(sPiVariable.strVarName, input, sizeof(sPiVariable.strVarName));
	rc = revPi_.GetVariableInfo(&sPiVariable);
	
	if(rc < 0)
	{
		std::cerr << "Cannot find variable name: " << sPiVariable.strVarName << std::endl;
		return ioState::ERROR;
	}
	if (sPiVariable.i16uLength == 1)
	{
		sPiValue.i16uAddress = sPiVariable.i16uAddress;
		sPiValue.i8uBit = sPiVariable.i8uBit;
		
		rc = revPi_.GetBitValue(&sPiValue);
		if(rc < 0)
		{
			std::cerr << "Got bit error!" << std::endl;
			return ioState::ERROR;
		}
		else
		{
			if(sPiValue.i8uValue == 1)
			{
				return ioState::HIGH;
			}
			else
			{
				return ioState::LOW;
			}
		}
	}
	else
	{
		std::cerr << "Could not read variable name" << std::endl;
		return ioState::ERROR;
	}
}

bool EmergencyStop::writeVariable(char* pszVariableName, 
								  uint32_t i32uValue)
{
	int rc;
	SPIVariable sPiVariable;
	SPIValue sPiValue;
	uint8_t i8uValue;
	uint16_t i16uValue; 
	
	strncpy(sPiVariable.strVarName, pszVariableName, sizeof(sPiVariable.strVarName));
	rc = revPi_.GetVariableInfo(&sPiVariable);
	
	if(rc < 0)
	{
		std::cerr << "Cannot find variable!" << std::endl;
		return false;
	}
	
	if(sPiVariable.i16uLength == 1)
	{
		sPiValue.i16uAddress = sPiVariable.i16uAddress;
		sPiValue.i8uBit = sPiVariable.i8uBit;
		sPiValue.i8uValue = i32uValue;
		rc = revPi_.SetBitValue(&sPiValue);
		
		if(rc < 0)
		{
			std::cout << "Set bit error!" << std::endl;
			return false;
		}
	}
	return true;
}

void EmergencyStop::emergencyStopLoop()
{
	while(!*stop_)
	{
		ioState ems = readInput(pemergency_input_);
		if(*cur_state_ != systemState::ACKNOWLEDGE)
		{
			//Check state of emergency button
			switch(ems)
			{
				case ioState::ERROR:
					std::cerr << "Error during IO readout!" << std::endl;
					return;
				case ioState::LOW:
					*cur_state_ = systemState::EMS;
					break;
				case ioState::HIGH:
					*cur_state_ = systemState::RUNNING;
					break;
				default: 
					std::cerr << "Unknown IO return value" << std::endl;
					return;
			}

			//Check for acknowledgement
			if((*prev_state_ == systemState::EMS) && (*cur_state_ == systemState::RUNNING))
			{
				*cur_state_ = systemState::ACKNOWLEDGE;
				// writeVariable(penable_, 0);
				*enabled_ = false;
			}
			//Check system state
			switch(*cur_state_)
			{
				case systemState::ERROR:
					std::cerr << "Error during IO readout!" << std::endl;
					return;
				case systemState::RUNNING:
					writeVariable(pgreen_, 1);
					writeVariable(pred_, 0);
					writeVariable(porange_, 0);
					writeVariable(pblue_, 0);
					break;
				case systemState::EMS:
					writeVariable(pgreen_, 0);
					writeVariable(pred_, 1);
					writeVariable(porange_, 0);
					writeVariable(pblue_, 0);
					writeVariable(penable_, 0);
					break;
				case systemState::ACKNOWLEDGE:
					writeVariable(pgreen_, 0);
					writeVariable(pred_, 0);
					writeVariable(porange_, 1);
					writeVariable(pblue_, 0);
					break;
					
				default: 
					std::cerr << "Unknown IO return value" << std::endl;
					return;
			}	
		}
		else
		{
			if(ems == ioState::LOW)
			{
				*cur_state_ = systemState::EMS;
			}
			else if(acknowledged_ == 1)
			{
				writeVariable(penable_, 1);
				// std::this_thread::sleep_for(std::chrono::milliseconds(500));
				*enabled_ = true;
				*cur_state_ = systemState::RUNNING;
			}
			acknowledged_ = 0;
		}

		*prev_state_ = *cur_state_;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

// void signalHandler(int signum)
// {
// 	stop = true;
// }

// int main(int argc, char** argv)
// {
// 	// Register signal handler
// 	signal(SIGINT, signalHandler);
	
// 	EmergencyStop es;

// 	while(!stop){}

// 	return 0;
// }

