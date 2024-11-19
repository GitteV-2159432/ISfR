#include <iostream>
#include <functional>
#include "drive_system/motor_controller.hpp"
#include "emergency_stop/emergency_stop.hpp"
#include "tcp_socket/tcp_socket.hpp"
#include "tcp_socket/data_structure.hpp"

std::shared_ptr<bool> stop_ = std::make_shared<bool>(false);

/**
 * Callback function to receive data back from the ROS component
*/
void fromROS(uint8_t* data, uint64_t size);

class MobilePlatform
{
public:
	MobilePlatform();
	~MobilePlatform();
	void runServer();

	//Components
	std::shared_ptr<EmergencyStop> emergency_stop_;
	std::shared_ptr<MotorController> motor_controller_;
	std::shared_ptr<Server> server_;

private:
	bool writeVariable(char* pszVariableName, uint32_t i32uValue);
	
	//Threads
	std::shared_ptr<std::thread> es_thread_;
	std::shared_ptr<std::thread> mc_thread_;
	std::shared_ptr<std::thread> tcp_thread_;

	//RevPi
	piControl revPi_;

	char *penable_;
	std::shared_ptr<bool> enabled_;
};

void signalHandler(int signum);