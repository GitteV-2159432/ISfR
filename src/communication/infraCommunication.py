from pyld import jsonld
import json
import sys
import cbor2 as cbor  
import paho.mqtt.client as mqtt
import time

class infraCommunication:
    def __init__(self, broker:str, port:int, username: str, password: str, topic:str) -> None:
        """ 
        :param broker: broker to copy from hivemq
        :param port: port to be used for communication
        :param username: username of account used for hivemq
        :param password: password of account used for hivemq
        :param topic: name of topic to where the messages will be published to
        """
        self.context = {}
        self.broker: str = broker
        self.port: int = port
        self.username: str = username
        self.password: str = password
        self.topic: str = topic
        self.client = mqtt.Client(client_id="robotData", transport= 'tcp', protocol=mqtt.MQTTv5)
        self.client.tls_set(tls_version= mqtt.ssl.PROTOCOL_TLS)
        self.received_lidar_data = None
        
    #not used now but can be used later
    def set_lidar_context(self):
        self.context = {
            "@context":{
                "sensor": "http://example.org/lidar-data",
                "lidar_coordinates" : "sensor:lidar_coordinates",
                "x": "sensor:x",
                "y": "sensor:y",
                "z": "sensor:z"
            }
        }
        
    #establish a connection with connect
    def connect(self) ->None:
        self.client.username_pw_set(self.username, self.password)
        self.client.connect(self.broker,port=self.port,clean_start=mqtt.MQTT_CLEAN_START_FIRST_ONLY, keepalive=60)
        self.client.loop_start()
        print("connected to broker!")
    
    #disconnect by using this function
    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()
        
    #encode the data with the dumps function
    def encode_data(self, data):
        return cbor.dumps(data)
        
    #decode the received data
    def decode_data(self, encoded_data):
        return cbor.loads(encoded_data)
    
    #subscribe to a topic
    def subscribe(self,topic):
        def on_message(client, userdata, msg):
            #voor robot:
            #received_bytes = bytes.fromhex(msg.payload)
            #self.received_lidar_data = cbor.loads(received_bytes)
            
            #via simulatie:
            self.received_lidar_data = msg.payload
        def on_subscribe(client, userdata,mid,granted_qos, properties=None):
            print(topic)
        self.client.subscribe(topic)
        self.client.on_message = on_message
        self.client.on_subscribe = on_subscribe
    
    #publishing the lidar points to the right topic
    def publish_lidar_points(self, topic, lidar_points):
    #check for type of data
        if isinstance(lidar_points, list):
            if isinstance(lidar_points[0], dict):
                dataToEncode = lidar_points
            elif isinstance(lidar_points[0], (list, tuple)):
                dataToEncode = [{"x": point[0], "y": point[1], "z": point[2]} for point in lidar_points]
            else:
                raise ValueError("invalid format for lidar points")
        else:
            raise ValueError("lidar_points has to be a list of dicts.")

        encoded_data = encoded_data(dataToEncode)
        self.client.publish(topic, encoded_data)

    #not used: function to publish the doubles for the dwa
    def publish_dwa_doubles(self, double1, double2,topicname):
        dataToEncode = {double1, double2}
        cborEncodedDoubles = self.encode_data(dataToEncode)
        #voor simulatie:
        self.client.publish(topicname, cborEncodedDoubles)
        
    #getter for the lidar data
    def get_lidar_data(self):
        return self.received_lidar_data