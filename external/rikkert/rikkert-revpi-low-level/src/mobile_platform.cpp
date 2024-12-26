#include "mobile_platform.hpp"

MobilePlatform *platform;

void fromROS(void* obj, uint8_t* data, uint64_t size)
{
	platform = (MobilePlatform* ) obj;
    printf("Received %llu bytes\n", size);
	if(size != 0)
	{
		DataROS data_struct;
		printf("Size of data struct:,  %lu\n", sizeof(DataROS));

		memcpy(&data_struct, data, sizeof(DataROS));
		printf("V_L = %f\n", data_struct.v_l);
		printf("V_R = %f\n", data_struct.v_r);
		printf("ACK = %u\n", data_struct.ACK);
		printf("reset_encoders = %u\n", data_struct.encoder_reset);

		platform->emergency_stop_->acknowledged_ = data_struct.ACK;
		platform->motor_controller_->v_l_ = data_struct.v_l;
		platform->motor_controller_->v_r_ = data_struct.v_r;
		// platform->motor_controller_->reset_encoders_ = data_struct.encoder_reset;
		if ((platform->motor_controller_->reset_done_ == 1) && (data_struct.encoder_reset == 1))
		{
			platform->motor_controller_->reset_encoders_ = 1;
			platform->motor_controller_->reset_done_ = 0;
		}
	}
}

MobilePlatform::MobilePlatform()
{
	//Init Enable Relay
	penable_ = (char*)"O_5";
	enabled_ = std::make_shared<bool>(false);
	revPi_ = piControl();

	//Enable Rikkert
	writeVariable(penable_, 1);
	*enabled_ = true;
	emergency_stop_ = std::make_shared<EmergencyStop>(enabled_, stop_);

	motor_controller_ = std::make_shared<MotorController>(enabled_, stop_, emergency_stop_->cur_state_);

	if((emergency_stop_->init_error_) ||
		(motor_controller_->init_error_))
	{
		std::cerr << "Initialisation error encountered!" << std::endl;
		return;
	}

	//Init TCP server
	server_ = std::make_shared<Server>();
	server_->set_callback(this, fromROS);
	server_->start(5050);

	//Create threads for components
	es_thread_ = std::make_shared<std::thread>(std::bind(&EmergencyStop::emergencyStopLoop, emergency_stop_));
	mc_thread_ = std::make_shared<std::thread>(std::bind(&MotorController::driveSystemLoop, motor_controller_));
	tcp_thread_ = std::make_shared<std::thread>(std::bind(&MobilePlatform::runServer, this));

}
MobilePlatform::~MobilePlatform()
{
	//Disable Rikkert
	writeVariable(penable_, 0);
	*enabled_ = false;
	//Trigger destructors
	emergency_stop_->~EmergencyStop();
	motor_controller_->~MotorController();
	//Shutdown thread	
	es_thread_->detach();
	mc_thread_->detach();
}

void MobilePlatform::runServer()
{
	uint8_t *data = new uint8_t[sizeof(DataPlatform)];

	while(!*stop_)
	{
		//Fill data struct
		DataPlatform data_struct;
		data_struct.ticks_l = motor_controller_->ticks_left_;
		data_struct.ticks_r = motor_controller_->ticks_right_;
		data_struct.voltage = motor_controller_->voltage_i_;
		data_struct.temp = motor_controller_->temp_i_;
		data_struct.state = *emergency_stop_->cur_state_;
		data_struct.reset_done = motor_controller_->reset_done_;

		memcpy(data, &data_struct, sizeof(DataPlatform));
		server_->write(data, sizeof(DataPlatform));

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	delete data;
}

bool MobilePlatform::writeVariable(char* pszVariableName, uint32_t i32uValue)
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

void signalHandler(int signum)
{
	*stop_ = true;
}

int main(int argc, char** argv)
{
	// Register signal handler
	signal(SIGINT, signalHandler);
	//Create mobile platform object
	MobilePlatform mp;
	//Loop until termination signal is received
	while(!*stop_){}
	return 0;
}