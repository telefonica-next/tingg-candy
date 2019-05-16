#!/usr/bin/env python
# coding: utf8

# Prerequirements
# pip3 install paho-mqtt

import sys
if sys.version_info[0] < 3:
    raise Exception("Please use Python3 !")

####################

import os
import serial
import time
import RPi.GPIO as GPIO
import threading
import datetime
import json
import paho.mqtt.client as mqtt
import math

####################
# TINGG.IO SETTINGS

TINGG_ID = "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"
TINGG_KEY = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

####################
# SETTINGS

DEBUG_MODE = True

PATH = '/home/pi/tingg-candy/RPi_Firmware/'

interval = 0.05           # Time between keyboard updates in seconds, smaller responds faster but uses more processor time

# Define some colors
BLACK  = (   0,   0,   0)
WHITE  = ( 255, 255, 255)

GPIO_JOYSTICK_UP     = 20
GPIO_JOYSTICK_RIGHT  = 26
GPIO_JOYSTICK_DOWN   = 21
GPIO_JOYSTICK_LEFT   = 16
GPIO_BUTTON_UP       = 12
GPIO_BUTTON_DOWN     = 19
GPIO_BUTTON_OPEN     = 13
GPIO_BUTTON_CLOSE    = 6
GPIO_BUTTON_RESET    = 5
GPIO_BUTTON_STOP     = 24
GPIO_BUTTON_SPARE_1  = 23
GPIO_BUTTON_SPARE_2  = 27

GPIO_BOUNCETIME_MS   = 25

####################
# GENERAL DEFINITONS

# inverted, becauso of Pull-Up and connected to GND when pressed
BUTTON_PRESSED     = 0
BUTTON_RELEASED    = 1

####################
# CLASSES

# https://raspberrypi.stackexchange.com/questions/76667/debouncing-buttons-with-rpi-gpio-too-many-events-detected
class ButtonHandler(threading.Thread):
  def __init__(self, pin, func, edge='both', bouncetime=200):
    super().__init__(daemon=True)

    self.edge = edge
    self.func = func
    self.pin = pin
    self.bouncetime = float(bouncetime)/1000

    self.lastpinval = GPIO.input(self.pin)
    self.lock = threading.Lock()

  def __call__(self, *args):
    if not self.lock.acquire(blocking=False):
      return

    t = threading.Timer(self.bouncetime, self.read, args=args)
    t.start()

  def read(self, *args):
    pinval = GPIO.input(self.pin)

    if ( ((pinval == BUTTON_PRESSED and self.lastpinval == BUTTON_RELEASED) and
         (self.edge in ['falling', 'both'])) or
        ((pinval == BUTTON_RELEASED and self.lastpinval == BUTTON_PRESSED) and
         (self.edge in ['rising', 'both'])) ):
      self.func(pinval, *args)

    self.lastpinval = pinval
    self.lock.release()

####################
# FUNCTIONS

# Button functions
def cb_debounced_jskUP(pinval, *args):
  if pinval == BUTTON_PRESSED:
    ser.write("DX-1\r\n".encode('utf-8'))
  else:
    ser.write("DX0\r\n".encode('utf-8'))

def cb_debounced_jskDOWN(pinval,*args):
  if pinval == BUTTON_PRESSED:
    ser.write("DX+1\r\n".encode('utf-8'))
  else:
    ser.write("DX0\r\n".encode('utf-8'))

def cb_debounced_jskLEFT(pinval,*args):
  if pinval == BUTTON_PRESSED:
    ser.write("DY-1\r\n".encode('utf-8'))
  else:
    ser.write("DY0\r\n".encode('utf-8'))
    
def cb_debounced_jskRIGHT(pinval,*args):
  if pinval == BUTTON_PRESSED:
    ser.write("DY+1\r\n".encode('utf-8'))
  else:
    ser.write("DY0\r\n".encode('utf-8'))

def cb_debounced_btnUP(pinval,*args):
  if pinval == BUTTON_PRESSED:
    ser.write("DZ-1\r\n".encode('utf-8'))
  else:
    ser.write("DZ0\r\n".encode('utf-8'))

def cb_debounced_btnDOWN(pinval,*args):
  if pinval == BUTTON_PRESSED:
    ser.write("DZ+1\r\n".encode('utf-8'))
  else:
    ser.write("DZ0\r\n".encode('utf-8'))

def cb_debounced_btnOPEN(pinval,*args):
  if pinval == BUTTON_PRESSED:
    ser.write("C-1\r\n".encode('utf-8'))
  else:
    ser.write("C0\r\n".encode('utf-8'))

def cb_debounced_btnCLOSE(pinval,*args):
  if pinval == BUTTON_PRESSED:
    ser.write("C+1\r\n".encode('utf-8'))
  else:
    ser.write("C0\r\n".encode('utf-8'))

def cb_debounced_btnSTOP(pinval,*args):
  if pinval == BUTTON_PRESSED:
    ser.write("S\r\n".encode('utf-8')) # stop

def cb_debounced_btnRESET(pinval,*args):
  if pinval == BUTTON_PRESSED:
    # dont go to Home when already there
    if x != 0 or y != 0 or z != 0:
      ser.write("H\r\n".encode('utf-8')) # go to home (position 0,0,0)
  
# other functions
def observeGoToHomeProcessTillEndAndInitData():
  print("Wait for XYZ-plotter to get to position 0,0,0")
  while True: # check serial input for "start" to know taht the xy-plotter is ready
    read_data = ser.readline().rstrip('\r\n'.encode('utf-8')).decode('utf-8')
    if read_data != "":
      print(read_data)
      if read_data == "GoToHome done":
        break
  print("XY-Plotter is ready!")
  # init values
  x = 0
  x_prev = 0
  y = 0
  y_prev = 0
  z = 0
  z_prev = 0
  db["numberOfSessionsPlayed"] += 1 # increase the session
  distanceTravelledInASession = 0
  # Log preparation
  if not os.path.isdir(PATH + 'logs'):
    os.makedirs(PATH + 'logs')
  log_path = os.path.join(PATH + 'logs', 'log%s.csv' % db["numberOfSessionsPlayed"])
  header = "date time x y z distanceTravelledInASession overallDistanceTravelledInMm"
  with open(log_path, 'a') as log_file:
    log_file.write(header + "\n")
  print("New Logfile for the new session is created!")
  print("New Session can start!")
  
# MQTT funcs
def on_connect(client, userdata, flags, rc):
  print("Connection to tingg.io established!")
  
def sendAllDataToTingg():
  client.publish(topic = 'x', payload = x)
  client.publish(topic = 'y', payload = y)
  client.publish(topic = 'z', payload = z)
  client.publish(topic = 'distance_travelled_mm', payload = db["overallDistanceTravelledInMm"])
  client.publish(topic = 'number_of_sessions_played', payload = db["numberOfSessionsPlayed"])  
  client.publish(topic = 'distance_travelled_in_session', payload = distanceTravelledInASession)
  print("Sended initial data to tingg.io!")


###############
# INITIALIZATIONS

# Variables
close = False #Loop until the user clicks the close button
x = 0
x_prev = 0
y = 0
y_prev = 0
z = 0
z_prev = 0
distanceTravelledInASession = 0

# init the database for storing long term values
print("Restore values from database-file (like travelled distance so far e.g.)")
try:
  with open(PATH + 'db.json', 'r') as f:
    db = json.load(f)
    print("Values restored are:")
    print(db)
    print("")
except:
    db = {"overallDistanceTravelledInMm": 0, "numberOfSessionsPlayed": -1}
    with open(PATH + 'db.json', 'w') as f:
      json.dump(db, f)

# init the joystick/buttons
try:
  GPIO.setmode(GPIO.BCM)
  
  GPIO.setup(GPIO_JOYSTICK_UP, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  cb_jskUP = ButtonHandler(GPIO_JOYSTICK_UP, cb_debounced_jskUP, edge='both', bouncetime=GPIO_BOUNCETIME_MS)
  cb_jskUP.start()
  GPIO.add_event_detect(GPIO_JOYSTICK_UP, GPIO.BOTH, callback=cb_jskUP)

  GPIO.setup(GPIO_JOYSTICK_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  cb_jskRIGHT = ButtonHandler(GPIO_JOYSTICK_RIGHT, cb_debounced_jskRIGHT, edge='both', bouncetime=GPIO_BOUNCETIME_MS)
  cb_jskRIGHT.start()
  GPIO.add_event_detect(GPIO_JOYSTICK_RIGHT, GPIO.BOTH, callback=cb_jskRIGHT)
  
  GPIO.setup(GPIO_JOYSTICK_DOWN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  cb_jskDOWN = ButtonHandler(GPIO_JOYSTICK_DOWN, cb_debounced_jskDOWN, edge='both', bouncetime=GPIO_BOUNCETIME_MS)
  cb_jskDOWN.start()
  GPIO.add_event_detect(GPIO_JOYSTICK_DOWN, GPIO.BOTH, callback=cb_jskDOWN)
  
  GPIO.setup(GPIO_JOYSTICK_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  cb_jskLEFT = ButtonHandler(GPIO_JOYSTICK_LEFT, cb_debounced_jskLEFT, edge='both', bouncetime=GPIO_BOUNCETIME_MS)
  cb_jskLEFT.start()
  GPIO.add_event_detect(GPIO_JOYSTICK_LEFT, GPIO.BOTH, callback=cb_jskLEFT)
  
  GPIO.setup(GPIO_BUTTON_UP, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  cb_btnUP = ButtonHandler(GPIO_BUTTON_UP, cb_debounced_btnUP, edge='both', bouncetime=GPIO_BOUNCETIME_MS)
  cb_btnUP.start()
  GPIO.add_event_detect(GPIO_BUTTON_UP, GPIO.BOTH, callback=cb_btnUP)

  GPIO.setup(GPIO_BUTTON_DOWN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  cb_btnDOWN = ButtonHandler(GPIO_BUTTON_DOWN, cb_debounced_btnDOWN, edge='both', bouncetime=GPIO_BOUNCETIME_MS)
  cb_btnDOWN.start()
  GPIO.add_event_detect(GPIO_BUTTON_DOWN, GPIO.BOTH, callback=cb_btnDOWN) 

  GPIO.setup(GPIO_BUTTON_OPEN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  cb_btnOPEN = ButtonHandler(GPIO_BUTTON_OPEN, cb_debounced_btnOPEN, edge='both', bouncetime=GPIO_BOUNCETIME_MS)
  cb_btnOPEN.start()
  GPIO.add_event_detect(GPIO_BUTTON_OPEN, GPIO.BOTH, callback=cb_btnOPEN) 

  GPIO.setup(GPIO_BUTTON_CLOSE, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  cb_btnCLOSE = ButtonHandler(GPIO_BUTTON_CLOSE, cb_debounced_btnCLOSE, edge='both', bouncetime=GPIO_BOUNCETIME_MS)
  cb_btnCLOSE.start()
  GPIO.add_event_detect(GPIO_BUTTON_CLOSE, GPIO.BOTH, callback=cb_btnCLOSE) 

  GPIO.setup(GPIO_BUTTON_STOP, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  cb_btnSTOP = ButtonHandler(GPIO_BUTTON_STOP, cb_debounced_btnSTOP, edge='both', bouncetime=GPIO_BOUNCETIME_MS)
  cb_btnSTOP.start()
  GPIO.add_event_detect(GPIO_BUTTON_STOP, GPIO.BOTH, callback=cb_btnSTOP) 

  GPIO.setup(GPIO_BUTTON_RESET, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  cb_btnRESET = ButtonHandler(GPIO_BUTTON_RESET, cb_debounced_btnRESET, edge='both', bouncetime=GPIO_BOUNCETIME_MS)
  cb_btnRESET.start()
  GPIO.add_event_detect(GPIO_BUTTON_RESET, GPIO.BOTH, callback=cb_btnRESET)  
  
except Exception as e:
  print("Error open joystick/buttons: " + str(e))
  print("")
  raise
  exit()
print("Init joystick/buttons is done!")


# Tingg.io connection
try:
  print("Try to connect to tingg.io ...")
  client = mqtt.Client(client_id = TINGG_ID)
  client.username_pw_set("thing", TINGG_KEY)
  client.on_connect = on_connect
  client.connect(host = "mqtt.tingg.io", port = 1883)
  client.loop_start()
except Exception as e:
  print("Error connecting to tingg.io: " + str(e))
  print("")
  exit()
  
# open the serial port to the XY-plotter arduino
ser = serial.Serial()
ser.port = "/dev/ttyUSB0"
ser.baudrate = 115200
ser.bytesize = serial.EIGHTBITS  # number of bits per bytes
ser.parity = serial.PARITY_NONE  # set parity check: no parity
ser.stopbits = serial.STOPBITS_TWO # number of stop bits
ser.timeout = 0.5          # timeout in seconds when no block is read. other options: None
ser.xonxoff = False        # disable software flow control
ser.rtscts = False         # disable hardware (RTS/CTS) flow control
ser.dsrdtr = False         # disable hardware (DSR/DTR) flow control
try:
  ser.open()
  print("Serial successfully opened!")
except Exception as e:
  print("Error open serial port: " + str(e))
  print("")
  exit()
  

####################
####################
# MAIN

# -------- Main Program Loop -----------
try:  
  print("")
  # init xy-plotter, drive head to Home (0,0,0)
  #ser.write('H\r\n'.encode('utf-8')) # ToDo: currently when serial inits, the arduino starts from the beginning 
  observeGoToHomeProcessTillEndAndInitData()
  sendAllDataToTingg()
  
  print("####################")
  print("")
  
  # MAIN-Loop
  while close == False:
    read_data = ser.readline().rstrip('\r\n'.encode('utf-8')).decode('utf-8')
    while read_data != "":
      # current positions
      if read_data[0] == "x":
        semicolonPos = read_data.find(';')
        x = int(read_data[1:semicolonPos])
        rest = read_data[semicolonPos+1:]
        semicolonPos = rest.find(';')
        y = int(rest[1:semicolonPos])
        z = int(rest[semicolonPos+2:])
        
        # in millimeter
        x = int(x * 256 / 21.168 / 16)
        y = int(y * 256 / 21.168 / 16)
        z = int(z * 256 / 13.5 / 16)
        
        # just send and calculate distance when there was a move
        if x != x_prev or y != y_prev or z != z_prev:                    
          # send to tingg.io
          client.publish(topic = 'x', payload = x)
          client.publish(topic = 'y', payload = y)
          client.publish(topic = 'z', payload = z)
          
          # calc droved way
          x_diff = abs(x - x_prev)
          y_diff = abs(y - y_prev)
          z_diff = abs(z - z_prev)
          
          tmpDistanceTravelledInMm = int(math.sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff))
          
          db["overallDistanceTravelledInMm"] += tmpDistanceTravelledInMm
          client.publish(topic = 'distance_travelled_mm', payload = db["overallDistanceTravelledInMm"])
          
          distanceTravelledInASession += tmpDistanceTravelledInMm
          client.publish(topic = 'distance_travelled_in_session', payload = distanceTravelledInASession)
          
          x_prev = x
          y_prev = y
          z_prev = z
          
          # log
          timestamp = str(datetime.datetime.now()).split('.')[0]
          log_path = os.path.join(PATH + 'logs', 'log%s.csv' % db["numberOfSessionsPlayed"])
          logData = timestamp + ' ' + str(x) + ' ' + str(y) + ' ' + str(z) + ' ' + str(distanceTravelledInASession) + ' ' + str(db["overallDistanceTravelledInMm"])
          with open(log_path, 'a') as log_file:
            log_file.write(logData + "\n")
      elif read_data == "GoToHome":
        observeGoToHomeProcessTillEndAndInitData()
        sendAllDataToTingg()
        print("####################")
        print("")
      else:
        print(read_data)
      read_data = ser.readline().rstrip('\r\n'.encode('utf-8')).decode('utf-8')

    time.sleep(interval)
    
# CTRL+C
except KeyboardInterrupt:
  print("CTRL+C pressed => close app!")
except Exception as e:
  print("Error: " + str(e))
  print("")
  raise
# run in all cases at end
finally:
  print("")
  print("!!!!!!!!!!!!!!!!!!!!!!!!!!!")
  print("EXIT")
  print("Write current values back to database-file (like travel distance e.g.)!")
  with open(PATH + 'db.json', 'w') as f:
    json.dump(db, f)
  print("GPIO cleanup!")
  GPIO.cleanup()
  print("Disconnect from tingg.io!")
  client.loop_stop()
  client.disconnect()
  print("!!!!!!!!!!!!!!!!!!!!!!!!!!!")
  print("")
  
  