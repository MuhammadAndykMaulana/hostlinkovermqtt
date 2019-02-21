import paho.mqtt.client as mqtt
import json, time, random

#========= For MQTT ===========
def on_connect(mqttc, obj, flags, rc):
    print("rc: " + str(rc))

def on_message(mqttc, obj, msg):
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
    msg_topic=msg.topic
    #Parse data    
def on_publish(mqttc, obj, mid):
    pass
    #print("")
    #print("publish: " + str(mid))
    
def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))
    
def on_log(mqttc, obj, level, string):
    print(string)

mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe
mqttc.connect("192.168.10.151", 1883, 60)
mqttc.subscribe("ev_second", 1)

#mqttc.loop_forever()
run = True
msg=['0','1']

while run:
    #Insert Data to billboard from Sensor
    mqttc.publish("reset",random.choice(msg),0)
    time.sleep (10)
    mqttc.loop()
