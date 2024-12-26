from module import Module, CommunicationHandler
from typing import List, Any

# Example of a specific handler, such as a TCP handler, that could inherit CommunicationHandler
class TCPCommunicationHandler(CommunicationHandler):
    def send(self, message: Any):
        # Specific implementation for sending a message over TCP
        print(f"Sending message over TCP: {message}")

    def receive(self) -> Any:
        # Specific implementation for receiving a message over TCP
        return "TCP message received"

# Example subclass for a specific robot module
class ArmController(Module):
    def execute(self):
        print(f"{self.module_name} is executing specialized functionalities.")
        # Add arm control logic here

# Usage example:
if __name__ == "__main__":
    arm_controller = ArmController("RobotArmModule")
    tcp_handler = TCPCommunicationHandler("ArmTCPHandler")
    
    arm_controller.add_communication_handler(tcp_handler)
    arm_controller.initialize_process(target_function=arm_controller.execute)
    
    # Simulate communication
    arm_controller.communicate("Move to position X")
    arm_controller.listen()

    # When done
    arm_controller.shutdown_process()