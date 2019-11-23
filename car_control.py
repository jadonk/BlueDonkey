#!/usr/bin/env python3

# This car_control routine was originally part of the OpenMV project.
# Copyright (c) 2013-2017 Ibrahim Abdelkader <iabdalkader@openmv.io> & Kwabena W. Agyeman <kwagyeman@openmv.io>
# This work is licensed under the MIT license, see the file LICENSE for details.

print("Loading Python modules for car_control. Please be patient.")
import rcpy, datetime, time, math, socket, json, numpy, cv2
#import pygame
# from rcpy.servo import servo1
# from rcpy.servo import servo3
#from rcpy.button import modeAI, pauseAI
from rcpy.button import mode, pause
from rcpy import button
from rcpy.led import red
from rcpy.led import green
print("Done importing modules for now!")


class car_control:
    
    # This function loads variables from josn in the provided filename.
    # Variables from car_control.py and line_follower.py have been moved here
    def load(self, filename):
        # Load json into memory
        with open(filename, 'r') as filehandle:
            self.jsondata = json.load(filehandle)
        
        # Perform math operations
        self.threshold = self.jsondata["COLOR_THRESHOLD_MAX"]
        
        self.jsondata["MIXING_RATE"] = max(min(self.jsondata["MIXING_RATE"], 1.0), 0.0)
        self.jsondata["THROTTLE_CUT_OFF_ANGLE"] = max(min(self.jsondata["THROTTLE_CUT_OFF_ANGLE"], 89.99), 0)
        self.jsondata["THROTTLE_CUT_OFF_RATE"] = max(min(self.jsondata["THROTTLE_CUT_OFF_RATE"], 1.0), 0.01)
        self.jsondata["THROTTLE_OFFSET"] = max(min(self.jsondata["THROTTLE_OFFSET"], 100), 0)
        self.jsondata["STEERING_OFFSET"] = max(min(self.jsondata["STEERING_OFFSET"], 180), 0)
        # Handle if these were reversed...
        self.tmp = max(self.jsondata["THROTTLE_SERVO_MIN"], self.jsondata["THROTTLE_SERVO_MAX"])
        self.jsondata["THROTTLE_SERVO_MIN"] = min(self.jsondata["THROTTLE_SERVO_MIN"], self.jsondata["THROTTLE_SERVO_MAX"])
        self.jsondata["THROTTLE_SERVO_MAX"] = self.tmp
        # Handle if these were reversed...
        self.tmp = max(self.jsondata["STEERING_SERVO_MIN"], self.jsondata["STEERING_SERVO_MAX"])
        self.jsondata["STEERING_SERVO_MIN"] = min(self.jsondata["STEERING_SERVO_MIN"], self.jsondata["STEERING_SERVO_MAX"])
        self.jsondata["STEERING_SERVO_MAX"] = self.tmp
        
        self.old_cx_normal = None
        self.throttle_old_result = None
        self.throttle_i_output = 0
        self.throttle_output = 0
        self.steering_old_result = None
        self.steering_i_output = 0
        
        # Array of region of interest masks in the order they should be searched
        # Furthest away first
        self.roi_masks = numpy.array([
                # Focus on the center
                # 8/20ths in from the self.left
                # 10/20ths down from the self.top
                # 1/20ths tall
                # 4x1 pixel count
                [int(8*self.jsondata["FRAME_WIDTH"]/20), int(10*self.jsondata["FRAME_HEIGHT"]/20), int(1*self.jsondata["FRAME_HEIGHT"]/20), int((4*self.jsondata["FRAME_WIDTH"]/20)*(1*self.jsondata["FRAME_HEIGHT"]/20)/100)],
                # Then look wider
                # 6/20ths in from the sides
                # 11/20ths down from the self.top
                # 1/20ths tall
                # 8x1 pixel count
                [int(6*self.jsondata["FRAME_WIDTH"]/20), int(11*self.jsondata["FRAME_HEIGHT"]/20), int(1*self.jsondata["FRAME_HEIGHT"]/20), int((8*self.jsondata["FRAME_WIDTH"]/20)*(1*self.jsondata["FRAME_HEIGHT"]/20)/100)],
                # Then look wider
                # 4/20ths in from the sides
                # 12/20ths down from the self.top
                # 1/20ths tall
                # 12x1 pixel count
                [int(4*self.jsondata["FRAME_WIDTH"]/20), int(12*self.jsondata["FRAME_HEIGHT"]/20), int(1*self.jsondata["FRAME_HEIGHT"]/20), int((12*self.jsondata["FRAME_WIDTH"]/20)*(1*self.jsondata["FRAME_HEIGHT"]/20)/100)],
                # Then really wide and taller
                # 0/20ths in from the sides
                # 13/20ths down from the self.top
                # 1/20ths tall
                # 20x1 pixel count
                [int(0*self.jsondata["FRAME_WIDTH"]/10), int(13*self.jsondata["FRAME_HEIGHT"]/20), int(1*self.jsondata["FRAME_HEIGHT"]/20), int((20*self.jsondata["FRAME_WIDTH"]/20)*(1*self.jsondata["FRAME_HEIGHT"]/20)/100)],
            ], dtype=numpy.int32)
        
    # This method outputs to the 3006 socket, filehandle vari_file
    def jsonreadout(self):
        # Create list
        self.tabledata = []
        # Throw all of the dicts into the list
        for key in self.jsondata:
            self.tabledata.append({ "Name":key, "Value":self.jsondata[key] })
        # Print the list -- This will cause a Node-RED error that can be ignored
        print(json.dumps(self.tabledata, sort_keys=True), file=self.vari_file, flush=True)
    
    def figure_out_my_steering(self, line):
        [vx,vy,x,y] = line
        cx_middle = x - (self.jsondata["FRAME_WIDTH"] / 2)
        cx_normal = cx_middle / (self.jsondata["FRAME_WIDTH"] / 2)
    
        if self.old_cx_normal != None:
            self.old_cx_normal = (cx_normal * self.jsondata["MIXING_RATE"]) + \
                (self.old_cx_normal * (1.0 - self.jsondata["MIXING_RATE"]))
        else: 
            self.old_cx_normal = cx_normal
        return self.old_cx_normal

    def figure_out_my_throttle(self, steering): # steering -> [0:180]
        # Solve: self.jsondata["THROTTLE_CUT_OFF_RATE"] = pow(sin(90 +/- self.jsondata["THROTTLE_CUT_OFF_ANGLE"]), x) for x...
        #        -> sin(90 +/- self.jsondata["THROTTLE_CUT_OFF_ANGLE"]) = cos(self.jsondata["THROTTLE_CUT_OFF_ANGLE"])
        t_power = math.log(self.jsondata["THROTTLE_CUT_OFF_RATE"]) / math.log(math.cos(math.radians(self.jsondata["THROTTLE_CUT_OFF_ANGLE"])))
    
        # pow(sin()) of the steering angle is only non-zero when driving straight... e.g. steering ~= 90
        t_result = math.pow(math.sin(math.radians(max(min(steering, 179.99), 0.0))), t_power)
        return (t_result * self.jsondata["THROTTLE_GAIN"]) + self.jsondata["THROTTLE_OFFSET"]
    
    #
    # Servo Control Code
    #
    
    # throttle [0:100] (101 values) -> [self.jsondata["THROTTLE_SERVO_MIN"], self.jsondata["THROTTLE_SERVO_MAX"]]
    # steering [0:180] (181 values) -> [self.jsondata["STEERING_SERVO_MIN"], self.jsondata["STEERING_SERVO_MAX"]]
    # def set_servos(self, throttle, steering):
        # throttle = self.jsondata["THROTTLE_SERVO_MIN"] + ((throttle/100) * (self.jsondata["THROTTLE_SERVO_MAX"] - self.jsondata["THROTTLE_SERVO_MIN"]))
        # steering = self.jsondata["STEERING_SERVO_MIN"] + ((steering/180) * (self.jsondata["STEERING_SERVO_MAX"] - self.jsondata["STEERING_SERVO_MIN"]))
        # servo3.set(throttle)
        # servo1.set(steering)
    
    # Enable PWM/servo outputs
    def enable_steering_and_throttle(self):
        # rcpy.servo.enable()
        
        # Enable steering servo
        # servo1.set(0)
        # servo1clk = rcpy.clock.Clock(servo1, 0.02)
        # servo1clk.start()
        
        # Enable throttle
        # servo3.set(0)
        # servo3clk = rcpy.clock.Clock(servo3, 0.02)
        # servo3clk.start()
        time.sleep(1)
        print("Arming throttle")
        # servo3.set(-0.1)
        time.sleep(3)
        # servo3.set(0)

    def tick(self):
        self.fps.tick()

    def pauseToggle(self):
        self.paused.toggle()

    def update(self, line):
        self.fps.stamp()
        if line:
            self.fps.update()
    
            #
            # Figure out steering and do steering PID
            #
    
            steering_new_result = self.figure_out_my_steering(line)
            steering_delta_result = (steering_new_result - self.steering_old_result) if (self.steering_old_result != None) else 0
            self.steering_old_result = steering_new_result
    
            steering_p_output = steering_new_result # Standard PID Stuff here... nothing particularly interesting :)
            self.steering_i_output = max(min(self.steering_i_output + steering_new_result, self.jsondata["STEERING_I_MAX"]), self.jsondata["STEERING_I_MIN"])
            steering_d_output = ((steering_delta_result * 1000) / self.fps.delta()) if self.fps.delta() else 0
            steering_pid_output = (self.jsondata["STEERING_P_GAIN"] * steering_p_output) + \
                                  (self.jsondata["STEERING_I_GAIN"] * self.steering_i_output) + \
                                  (self.jsondata["STEERING_D_GAIN"] * steering_d_output)
    
            # Steering goes from [-90,90] but we need to output [0,180] for the servos.
            self.steering_output = (self.jsondata["STEERING_OFFSET"] + steering_pid_output) % 180
    
            #
            # Figure out throttle and do throttle PID
            #
    
            throttle_new_result = self.figure_out_my_throttle(self.steering_output)
            throttle_delta_result = (throttle_new_result - self.throttle_old_result) if (self.throttle_old_result != None) else 0
            self.throttle_old_result = throttle_new_result
    
            throttle_p_output = throttle_new_result # Standard PID Stuff here... nothing particularly interesting :)
            self.throttle_i_output = max(min(self.throttle_i_output + throttle_new_result, self.jsondata["THROTTLE_I_MAX"]), self.jsondata["THROTTLE_I_MIN"])
            throttle_d_output = ((throttle_delta_result * 1000) / self.fps.delta()) if self.fps.delta() else 0
            throttle_pid_output = (self.jsondata["THROTTLE_P_GAIN"] * throttle_p_output) + \
                                  (self.jsondata["THROTTLE_I_GAIN"] * self.throttle_i_output) + \
                                  (self.jsondata["THROTTLE_D_GAIN"] * throttle_d_output)
    
            # Throttle goes from 0% to 100%.
            self.throttle_output = max(min(throttle_pid_output, 100), 0)
        else:
            self.throttle_output = self.throttle_output * 0.99
            self.steering_output = self.jsondata["STEERING_OFFSET"]
            
        # Call jsonreadout() method to update the variables on the dashboard
        self.jsonreadout()
        
        # If not paused
        if not self.paused.state:
            # Try to accept connection from client.py
            try:
                client, addr = self.listener.accept()
                
            # The accept timed out, move on
            # Timout is set in car_control initialization
            except socket.timeout:
                pass
            
            # Something went really wrong
            except:
                raise
            
            # Successfully accepted connection
            else:
                # Receive data from client.py
                data = client.recv(1024).decode()
                # Since we are paused, the only check is for 'PAUSE'
                # If the client sent pause command
                if data == 'PAUSE':
                    print('PAUSE triggered by client', file=self.log_file,
                                                                flush=True)
                   # Set state to paused and send confirmation fo client (debug)
                    self.paused.state = not self.paused.state
                    client.send(data.encode())
        
        # If paused
        else:
            # Set movement to 0
            self.throttle_output = 0
            self.steering_output = self.jsondata["STEERING_OFFSET"]
            
            # Try to accept connection from client.py
            try:
                client, addr = self.listener.accept()
                
            # The accept timed out, move on
            # Timout is set in car_control initialization
            except socket.timeout:
                pass
            
            # Something went really wrong
            except:
                raise
            
            # Successfully accepted connection
            else:
                # Receive data from client.py
                data = client.recv(1024).decode()
                
                # Did client.py send 'UNPAUSE'? If so then unpause
                if data == 'UNPAUSE':
                    print('UNPAUSE triggered by client', file=self.log_file, flush=True)
                    self.paused.state = not self.paused.state
                    client.send(data.encode())
                    
                # Did client.py send 'LOAD'? If so then call load
                # TODO profiles
                elif data == 'LOAD':
                    print('Loading values from default_values.json', file=self.log_file, flush=True)
                    self.load('profiles/default_values.json')
                
                # Did client.py send 'LOADNODE'? If so then call load
                # on the node json file
                elif data == 'LOADNODE':
                    print("Loading values from node_values.json", file=self.log_file, flush=True)
                    self.load('profiles/node_values.json')
                    
                # Did client send 'LOADCUSTOM'? If so then call load
                # on the json file path provided
                elif 'LOADCUSTOM' in data:
                    tmp, file = data.split()
                    print("Loading values from " + file, file=self.log_file, flush=True)
                    self.load(file)

        # self.set_servos(self.throttle_output, self.steering_output)
        return(self.paused.state, self.throttle_output, self.steering_output, self.fps.get())

    def __init__(self):
        # This file is the initially loaded file
        self.filename = './profiles/default_values.json'
        # Call load() to initialize variables
        self.load(self.filename)
        print("car_control load complete")
        # Start up the pause button handler
        self.paused = PauseButtonEvent()
        self.paused.start()
        self.enable_steering_and_throttle()
        self.fps = track_fps()
        
        # Input socket to receive from client.py
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(('127.0.0.1', 3002))
        # This sets the timeout for the socket accept() calls
        # defaulted to 0.001 as that is the time that was previously being 
        # time.sleep ed for in the update method
        self.listener.settimeout(0.001)
        print('listening on 127.0.0.1:3002')
        # Based on my understanding of the documentation
        # This sets the number of client.py connect() calls to cache
        self.listener.listen(5)
        
        # Output socket for tracking signals received
        self.log_port = 3005
        self.log_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.log_out.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.log_out.connect(("", self.log_port))
        # This filehandle sends the data from the socket broadcast
        self.log_file = self.log_out.makefile('w', buffering=None)
        
        # Output socket for viewing current variables
        self.vari_port = 3006
        self.vari_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vari_out.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.vari_out.connect(("", self.vari_port))
        # This filehandle sends the data from the socket broadcast
        self.vari_file = self.vari_out.makefile('w', buffering=None)
        
        
        

class PauseButtonEvent(button.ButtonEvent):
    state = True
    def __init__(self):
        # Start-up in 'paused' mode and handle button presses to exit paused mode
        red.on()
        green.off()
        #from rcpy.button import modeAI, pauseAI
        button.ButtonEvent.__init__(self, pause, button.ButtonEvent.PRESSED)
        #button.ButtonEvent.__init__(self, pauseAI, button.ButtonEvent.PRESSED)
        #self.start()
    def action(self, event):
        self.toggle()
    def toggle(self):
        self.state = not self.state
        if self.state:
            red.on()
            green.off()
        else:
            green.on()
            red.off()
    def state(self):
        return self.state

class track_fps:
    delta_time = 0
    stamp_time = 0
    old_time = 0
    def __init__(self):
        #self.clock = pygame.time.Clock()
        self.old_time = datetime.datetime.now()
    def tick(self):
        #self.clock.tick()
        return
    def get(self):
        #return self.clock.get_fps()
        return 0
    def stamp(self):
        self.stamp_time = datetime.datetime.now()
        msec_stamp = int((self.stamp_time.second * 1000) + (self.stamp_time.microsecond / 1000))
        return msec_stamp
    def update(self):
        self.delta_time = (self.stamp_time - self.old_time).microseconds / 1000
        self.old_time = self.stamp_time
    def delta(self):
        return self.delta_time
