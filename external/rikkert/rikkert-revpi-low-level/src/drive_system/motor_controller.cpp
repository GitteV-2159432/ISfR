#include "drive_system/motor_controller.hpp"

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
MotorController::MotorController(std::shared_ptr<bool> enabled, 
								 std::shared_ptr<bool> stop, 
								 std::shared_ptr<systemState> cur_state)
{	
	enabled_ = enabled;
	stop_ = stop;
	cur_state_ = cur_state;

	//Init
	comm_port_ = "/dev/serial/by-id/usb-Basicmicro_Inc._MCP233_2x30A-if00";
	int serial_port = open(comm_port_.c_str(), O_RDWR);
	while(serial_port < 0)
	{
		if(*stop_)
		{
			init_error_ = true;
			return;
		}
		std::cerr << "Error while trying to find serial device, waiting ..." << std::endl;
		serial_port = open(comm_port_.c_str(), O_RDWR);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	baudrate_ = B115200;
	timeout_ = 5;
	address_ = 0x80;
		
	ticks_left_ = 0;
	ticks_right_ = 0;
	
	driver_ = RoboClaw(comm_port_, timeout_, address_);
	bool ok = driver_.begin(baudrate_);
	while(!stop_ && !ok)
	{
		std::cerr << "Problem encountered during RoboClaw initialisation \n Retrying in 1 s" << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
		ok = driver_.begin(baudrate_);
	}

	char version[512];
	
	//Check motor driver connection
	driver_.ReadVersion(version);
	std::cout << "Motor driver version: " << version << std::endl;
	
	driver_.ResetEncoders(address_);
	std::cout << "Encoder reset done\n";
	
	//Get parameters of driver
	float kp, ki, kd;
	uint32_t qpps;
	driver_.ReadM1VelocityPID(kp, ki, kd, qpps);
	std::cout << "M1 velocity PID: " << kp << ", " << ki << ", " << kd << ", " << qpps << std::endl;
	driver_.ReadM2VelocityPID(kp, ki, kd, qpps);
	std::cout << "M2 velocity PID: " << kp << ", " << ki << ", " << kd << ", " << qpps << std::endl;

	uint16_t temperature;
	bool valid = driver_.ReadTemp(temperature);
	std::cout << "Temperature: " << temperature / 10.0 << " degrees Celcius\n";
}
MotorController::~MotorController()
{
}

void MotorController::dataReadout()
{
	//Battery voltage
	bool valid;
	voltage_i_ = driver_.ReadMainBatteryVoltage(&valid);
	voltage_d_ = voltage_i_ / 10.0;
	
	//Temperature
	valid = driver_.ReadTemp(temp_i_);
	temp_d_ = temp_i_ / 10.0;
	
	//Encoder ticks
	bool validM1, validM2;
	uint8_t statusE1, statusE2;
	if(reset_encoders_ == 0)
	{
		ticks_left_ = driver_.ReadEncM1(&statusE1, &validM1);
		ticks_right_ = driver_.ReadEncM2(&statusE2, &validM2);
	}
	else if (reset_encoders_ == 1)
	{
		driver_.ResetEncoders(address_);
		reset_encoders_ = 0;
		reset_done_ = 1;
	}

	//Wheel velocity
	bool validM1S, validM2S;
	uint8_t statusM1S, statusM2S;
	speed_left_ = driver_.ReadSpeedM1(&statusM1S, &validM1S);
	speed_right_ = driver_.ReadSpeedM2(&statusM2S, &validM2S);
}

void MotorController::controlMotors()
{
	//Left wheel
	int u_l = (int)(v_l_ * 127/2.5);
	int u_r = (int)(v_r_ * 127/2.5);

	if(u_l > 0 && u_r > 0)
	{
		if(u_l > 125)
		{driver_.ForwardM1(125);}
		else
		{driver_.ForwardM1((uint8_t) std::abs(u_l));}
		if(u_r > 125)
		{driver_.ForwardM2(125);}
		else
		{driver_.ForwardM2((uint8_t) std::abs(u_r));}
	}
	else if(u_l < 0 && u_r < 0)
	{
		if(std::abs(u_l) > 125)
		{driver_.BackwardM1(125);}
		else
		{driver_.BackwardM1((uint8_t) std::abs(u_l));}
		if(std::abs(u_r) > 125)
		{driver_.BackwardM2(125);}
		else
		{driver_.BackwardM2((uint8_t) std::abs(u_r));}
	}
	else if (u_l > 0 && u_r < 0)
	{
		if(std::abs(u_l) > 125)
		{driver_.ForwardM1(125);}
		else
		{driver_.ForwardM1((uint8_t) std::abs(u_l));}
		if(std::abs(u_r) > 125)
		{driver_.BackwardM2(125);}
		else
		{driver_.BackwardM2((uint8_t) std::abs(u_r));}
	}
	else if (u_l < 0 && u_r > 0)
	{
		if(std::abs(u_l) > 125)
		{driver_.BackwardM1(125);}
		else
		{driver_.BackwardM1((uint8_t) std::abs(u_l));}
		if(std::abs(u_r) > 125)
		{driver_.ForwardM2(125);}
		else
		{driver_.ForwardM2((uint8_t) std::abs(u_r));}
	}
	else
	{
		driver_.ForwardM1(0);
		driver_.ForwardM2(0);
	}
}

void MotorController::driveSystemLoop()
{
	while(!*stop_)
	{
		dataReadout();
		controlMotors();
		prev_state_ = *cur_state_;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}
