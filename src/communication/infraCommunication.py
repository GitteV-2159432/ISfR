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

    def set_lidar_context(self):
        self.context = {
            "@context":{
                #ask this next time, for more info about the usage of it
                #maybe ssn can be used
                ###ALSO:https://github.com/digitalbazaar/cborld/blob/main/lib/Converter.js
                #Link can be useful to understand cbor-ld
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
    
    #disconnect by using this function
    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()
        
    def encode_data(self, data):
        return cbor.dumps(data)
        
    def decode_data(self, encoded_data):
        return cbor.loads(encoded_data)
    
    def publish_lidar_points(self,lidar_points : list):
        """dataToEncode = {
            "@context": self.context,
            "lidar_coordinates": [{"x": point[0], "y": point[1], "z": point[2]} for point in lidar_points]
        }"""
        #voorlopig zonder de context, indien met context enkel de array eruit halen
        dataToEncode= [{"x": point[0], "y": point[1], "z": point[2]} for point in lidar_points]
        #print(dataToEncode)
        cborEncodedData = self.encode_data(dataToEncode)
        self.client.publish(self.topic, cborEncodedData)
        
    def publish_dwa_doubles(self, double1, double2,topicname):
        dataToEncode = double1, double2
        cborEncodedDoubles = self.encode_data(dataToEncode)
        self.client.publish(topicname, cborEncodedDoubles)
        
        