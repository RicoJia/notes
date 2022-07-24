"""
This script is a minimum example of MQTT client. In the meantime, it measures the pub-sub communication time with another MQTT client
    MQTT Command line basics:
        - 
"""
import paho.mqtt.client as mqtt
import time

now = time.time()
PUBLISH_MSG_COUNT = 10

def on_connect(client, userdata, flags, rc):  # The callback for when the client connects to the broker
    print("Connected with result code {0}".format(str(rc)))  # Print result of connection attempt
    client.subscribe("esp/joint_states")  # Subscribe to the topic “digitest/test1”, receive any messages published on it


def on_message(client, userdata, msg):  # The callback for when a PUBLISH message is received from the server.
    global now
    if on_message.publish_msg_count < PUBLISH_MSG_COUNT: 
        if on_message.publish_msg_count == 0:
            now = time.time()
        on_message.publish_msg_count += 1
        print(f"pub msg: {on_message.publish_msg_count}/{PUBLISH_MSG_COUNT}")
    else: 
        print((time.time() - now)/PUBLISH_MSG_COUNT)
        time.sleep(1)
        print("================")
        on_message.publish_msg_count = 0
    client.publish("esp/plan", "1234567890"*3) 
on_message.publish_msg_count=0

client = mqtt.Client("digi_mqtt_test")  # Create instance of client with client ID “digi_mqtt_test”
client.on_connect = on_connect  # Define callback function for successful connection
client.on_message = on_message  # Define callback function for receipt of a message
# client.connect("m2m.eclipse.org", 1883, 60)  # Connect to (broker, port, keepalive-time)
client.connect('127.0.0.1', 1883)
client.publish("esp/plan", "1234567890"*10) 
client.loop_forever()  # Start networking daemon

 
