import paho.mqtt.client as mqtt
import serial.tools.list_ports, warnings
import time, json, serial

## Variables for serial data response
connected = False
resp_from_plc=''
#Flag for error
e_f=0

def autodetect():
    plc_ports = [
        p.device
        for p in serial.tools.list_ports.comports()
        if 'USB-SERIAL CH340' or 'USB2.0-Serial' in p.description #USB-SERIAL CH340 for Windows, USB2.0-Serial for Linux
    ]
    if not plc_ports:
        raise IOError("No PLC serial found")
    if len(plc_ports) > 1:
        warnings.warn('Multiple PLC ports found - using the 1st ')
    return plc_ports[0]

device=autodetect()
while not connected:
    try:
        ser_to_plc = serial.Serial(device,      #port
                            9600,               #baudrate
                            serial.SEVENBITS,   #bytesize
                            serial.PARITY_EVEN, #parity
                            serial.STOPBITS_TWO,#stop bit
                            0,                  #timeout
                            False,              #xonxoff
                            False,              #rtscts
                            0,                  #write_timeout
                            False,              #dsrdtr
                            None,               #inter byte to
                            None                #exclusive
                            )
        connected=True
    except:
        connected=False
        print ("trying to connect to ", device)
        time.sleep(1.5)
if connected:
    serin = ser_to_plc.read()
    print ("Connected to ",device)
def getId():
    iD = "0000000000000000"
    try:
        f = open('/proc/cpuinfo','r')
        for line in f:
            if line[0:6]=='Serial':
                iD = line[10:26]
        f.close()
    except:
        iD = "ERROR00000000000"
        f.close()
    return iD

def MsgConstruct(msgTemp): #FCS Calculating
    fcsTemp = 0
    msgTemp = msgTemp

    for x in range(0,len(msgTemp)):
        fcsTemp = fcsTemp ^ ord(msgTemp[x])
    fcs1 = (fcsTemp >> 4) & 0xf
    fcs2 = (fcsTemp) & 0xf
    msgTemp = (msgTemp + hex(fcs1)[2] + hex(fcs2)[2] + "*\r\n")
    return msgTemp

#========= For MQTT ===========
def on_connect(mqttc, obj, flags, rc):
    print("rc: " + str(rc))

serReset=1
def on_message(mqttc, obj, msg):
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
    msg_topic=msg.topic
    if (msg.payload.decode()=='1'):
        SetMonitorModePLC()
        serReset=1
        print("message payload gan with serReset "+serReset)
    else :
        SetRunModePLC()
        serReset=0

def on_publish(mqttc, obj, mid):
    pass
    #print("")
    #print("publish: " + str(mid))
    
def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))
    
def on_log(mqttc, obj, level, string):
    print(string)
def on_disconnect(mqttc, userdata, rc):
    print ('Problem : '+ str(rc))
    # Trying to reconnect to the server
    ready_f=0
    while 1:
        try:
            #mqttc.connect("192.168.10.151", 1883, 60)
            broker="192.168.10.151"
            mqttc.connect(broker, 1883, 60)
            print("Connected to broker "+broker)
            mqttc.loop()
            ready_f=1
        except:
            print ('Trying to reconnect to the server...')
            if ready_f==1:
               break
mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe
mqttc.connect("192.168.10.151", 1883, 60)
mqttc.subscribe("reset", 1)

def SetMonitorModePLC():
    global resp_from_plc
    cmd=(MsgConstruct('@00SC02')).upper() #send to monitor mode
    ser_to_plc.write(cmd.encode('ascii'))
    print("Send to Monitor Mode...\n")
    while 1:
        if (ser_to_plc.inWaiting()):
            x=ser_to_plc.read()
            x=x.decode('ascii')
            resp_from_plc+=x
            # print(resp_from_plc)
            if x == '\r':
                print("Response set Monitor Mode: "+resp_from_plc)
                resp_from_plc=""
                break
def SetRunModePLC():
    global resp_from_plc
    cmd=(MsgConstruct('@00SC03')).upper() #send to run mode
    ser_to_plc.write(cmd.encode('ascii'))
    print("Send to Run Mode...\n")
    while 1:
        if (ser_to_plc.inWaiting()):
            x=ser_to_plc.read()
            x=x.decode('ascii')
            resp_from_plc+= x
            # print(resp_from_plc)
            if x == '\r':
                print("Response set Run Mode: "+resp_from_plc)
                resp_from_plc=""
                break
run = True
while run:
    try:
        mqttc.publish("ev-second",{"data": {"s1":"3","s2":"2","s3":"0","s4":"0","s5":"0","s6":"0","s7":"0","s8":"0","s9":"0","s10":"0","s11":"0","s12":"0","s13":"0","s14":"0","s15":"0","s16":"0","s17":"3","s18":"0","s19":"1","s20":"0","s21":"0","s22":"0","s23":"0","s24":"0","s25":"0","s26":"0","s27":"0","s28":"0","ts":"1550552662552"},0)
        time.sleep(1)
        mqttc.loop()
    except KeyboardInterrupt:
        print ("Connection is closed!")
        break
    except ValueError as e:
        print("ValueError ",e)
        pass
    except Exception as e:
        print ('Error : '+ str(e))
        break
    except:
        print("Unexpected error:", sys.exc_info()[0])
        raise
