import multiprocessing
import abc
from typing import List, Any

class CommunicationHandler:
    """
    Generic communication handler. Each module will set up its specific communication logic by subclassing this.
    """
    def __init__(self, name: str):
        self.name = name

    def send(self, message: Any):
        raise NotImplementedError("Must be implemented in the subclass.")

    def receive(self) -> Any:
        raise NotImplementedError("Must be implemented in the subclass.")

class Module(abc.ABC):
    """
    A generic module base class
    """
    def __init__(self, module_name: str):
        self.module_name = module_name
        self.communication_handlers: List[CommunicationHandler] = []
        self.process = None

    def add_communication_handler(self, handler: CommunicationHandler):
        self.communication_handlers.append(handler)

    def initialize_process(self, target_function):
        """
        Initializes a new process for the module.
        """
        self.process = multiprocessing.Process(target=target_function, args=(self,))
        self.process.start()

    def shutdown_process(self):
        """
        Terminates the process for the module.
        """
        if self.process:
            self.process.terminate()
            self.process.join()

    @abc.abstractmethod
    def execute(self):
        """
        Abstract method to be implemented in subclasses for specific module functionality.
        """
        pass

    def communicate(self, message: Any):
        """
        Generic communication method to send messages via handlers.
        """
        for handler in self.communication_handlers:
            handler.send(message)
    
    def listen(self):
        """
        Generic communication method to receive messages via handlers.
        """
        for handler in self.communication_handlers:
            response = handler.receive()
            print(f"Received: {response}")


