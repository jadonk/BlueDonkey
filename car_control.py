#!/usr/bin/env python3
print("Loading Python modules for car_control. Please be patient.")
import rcpy, datetime, time, math, socket, json
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
    
    def load(self, filename):
        with open(filename, 'r') as filehandle:
            jsonhandle = json.load(filehandle)
        self.FRAME_WIDTH = float(jsonhandle["FRAME_WIDTH"])
        self.MIXING_RATE = float(jsonhandle["MIXING_RATE"])                            # Percentage of a new line detection to mix into current steering.
        self.THROTTLE_CUT_OFF_ANGLE = float(jsonhandle["THROTTLE_CUT_OFF_ANGLE"])      # Maximum angular distance from 90 before we cut speed [0.0-90.0).
        self.THROTTLE_CUT_OFF_RATE = float(jsonhandle["THROTTLE_CUT_OFF_RATE"])        # How much to cut our speed boost (below) once the above is passed (0.0-1.0].
        self.THROTTLE_GAIN = float(jsonhandle["THROTTLE_GAIN"])                        # e.g. how much to speed up on a straight away
        self.THROTTLE_OFFSET = float(jsonhandle["THROTTLE_OFFSET"])                    # e.g. default speed (0 to 100)
        self.THROTTLE_P_GAIN = float(jsonhandle["THROTTLE_P_GAIN"])
        self.THROTTLE_I_GAIN = float(jsonhandle["THROTTLE_I_GAIN"])
        self.THROTTLE_I_MIN = float(jsonhandle["THROTTLE_I_MIN"])
        self.THROTTLE_I_MAX = float(jsonhandle["THROTTLE_I_MAX"])
        self.THROTTLE_D_GAIN = float(jsonhandle["THROTTLE_D_GAIN"])
        self.STEERING_OFFSET = float(jsonhandle["STEERING_OFFSET"])                    # Change this if you need to fix an imbalance in your car (0 to 180).
        self.STEERING_P_GAIN = float(jsonhandle["STEERING_P_GAIN"])                    # Make this smaller as you increase your speed and vice versa.
        self.STEERING_I_GAIN = float(jsonhandle["STEERING_I_GAIN"])
        self.STEERING_I_MIN = float(jsonhandle["STEERING_I_MIN"])
        self.STEERING_I_MAX = float(jsonhandle["STEERING_I_MAX"])
        self.STEERING_D_GAIN = float(jsonhandle["STEERING_D_GAIN"])                    # Make this larger as you increase your speed and vice versa.
        self.THROTTLE_SERVO_MIN = float(jsonhandle["THROTTLE_SERVO_MIN"])
        self.THROTTLE_SERVO_MAX = float(jsonhandle["THROTTLE_SERVO_MAX"])
        self.STEERING_SERVO_MIN = float(jsonhandle["STEERING_SERVO_MIN"])
        self.STEERING_SERVO_MAX = float(jsonhandle["STEERING_SERVO_MAX"])
        
        self.MIXING_RATE = max(min(self.MIXING_RATE, 1.0), 0.0)
        self.THROTTLE_CUT_OFF_ANGLE = max(min(self.THROTTLE_CUT_OFF_ANGLE, 89.99), 0)
        self.THROTTLE_CUT_OFF_RATE = max(min(self.THROTTLE_CUT_OFF_RATE, 1.0), 0.01)
        self.THROTTLE_OFFSET = max(min(self.THROTTLE_OFFSET, 100), 0)
        self.STEERING_OFFSET = max(min(self.STEERING_OFFSET, 180), 0)
        # Handle if these were reversed...
        self.tmp = max(self.THROTTLE_SERVO_MIN, self.THROTTLE_SERVO_MAX)
        self.THROTTLE_SERVO_MIN = min(self.THROTTLE_SERVO_MIN, self.THROTTLE_SERVO_MAX)
        self.THROTTLE_SERVO_MAX = self.tmp
        # Handle if these were reversed...
        self.tmp = max(self.STEERING_SERVO_MIN, self.STEERING_SERVO_MAX)
        self.STEERING_SERVO_MIN = min(self.STEERING_SERVO_MIN, self.STEERING_SERVO_MAX)
        self.STEERING_SERVO_MAX = self.tmp
        
        self.old_cx_normal = None
        self.throttle_old_result = None
        self.throttle_i_output = 0
        self.throttle_output = 0
        self.steering_old_result = None
        self.steering_i_output = 0
        
        
    def jsonreadout(self):
        print("Frame Width: ",self.FRAME_WIDTH,"\nMixing Rate: ",self.MIXING_RATE,"\nThrottle_cut_off_angle: ",self.THROTTLE_CUT_OFF_ANGLE,
        "\nThrottle_cut_off_rate: ",self.THROTTLE_CUT_OFF_RATE,"\nThrottle_Gain: ",self.THROTTLE_GAIN,"\nThrottle_Offset: ",self.THROTTLE_OFFSET,
        "\nThrottle P Gain: ",self.THROTTLE_P_GAIN,"\nThrottle I Gain: ",self.THROTTLE_I_GAIN,"\nThrottle I Min: ",self.THROTTLE_I_MIN,
        "\nThrottle I Max: ",self.THROTTLE_I_MAX,"\nThrottle D Gain: ",self.THROTTLE_D_GAIN,"\nSteering Offset: ",self.STEERING_OFFSET,
        "\nSteering P Gain: ",self.STEERING_P_GAIN,"\nSteering I Gain: ",self.STEERING_I_GAIN,"\nSteering I Min: ",self.STEERING_I_MIN,
        "\nSteering I Max: ",self.STEERING_I_MAX,"\nSteering D Gain: ",self.STEERING_D_GAIN,"\nThrottle Servo Min: ",self.THROTTLE_SERVO_MIN,
        "\nThrottle Servo Max: ",self.THROTTLE_SERVO_MAX,"\nSteering Servo Min: ",self.STEERING_SERVO_MIN,"\nSteering Servo Max: ",self.STEERING_SERVO_MAX, file=self.vari_file, flush=True)
    
    def figure_out_my_steering(self, line):
        [vx,vy,x,y] = line
        cx_middle = x - (self.FRAME_WIDTH / 2)
        cx_normal = cx_middle / (self.FRAME_WIDTH / 2)
    
        if self.old_cx_normal != None:
            self.old_cx_normal = (cx_normal * self.MIXING_RATE) + \
                (self.old_cx_normal * (1.0 - self.MIXING_RATE))
        else: 
            self.old_cx_normal = cx_normal
        return self.old_cx_normal

    def figure_out_my_throttle(self, steering): # steering -> [0:180]
        # Solve: self.THROTTLE_CUT_OFF_RATE = pow(sin(90 +/- self.THROTTLE_CUT_OFF_ANGLE), x) for x...
        #        -> sin(90 +/- self.THROTTLE_CUT_OFF_ANGLE) = cos(self.THROTTLE_CUT_OFF_ANGLE)
        t_power = math.log(self.THROTTLE_CUT_OFF_RATE) / math.log(math.cos(math.radians(self.THROTTLE_CUT_OFF_ANGLE)))
    
        # pow(sin()) of the steering angle is only non-zero when driving straight... e.g. steering ~= 90
        t_result = math.pow(math.sin(math.radians(max(min(steering, 179.99), 0.0))), t_power)
        return (t_result * self.THROTTLE_GAIN) + self.THROTTLE_OFFSET
    
    #
    # Servo Control Code
    #
    
    # throttle [0:100] (101 values) -> [self.THROTTLE_SERVO_MIN, self.THROTTLE_SERVO_MAX]
    # steering [0:180] (181 values) -> [self.STEERING_SERVO_MIN, self.STEERING_SERVO_MAX]
    # def set_servos(self, throttle, steering):
        # throttle = self.THROTTLE_SERVO_MIN + ((throttle/100) * (self.THROTTLE_SERVO_MAX - self.THROTTLE_SERVO_MIN))
        # steering = self.STEERING_SERVO_MIN + ((steering/180) * (self.STEERING_SERVO_MAX - self.STEERING_SERVO_MIN))
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
            self.steering_i_output = max(min(self.steering_i_output + steering_new_result, self.STEERING_I_MAX), self.STEERING_I_MIN)
            steering_d_output = ((steering_delta_result * 1000) / self.fps.delta()) if self.fps.delta() else 0
            steering_pid_output = (self.STEERING_P_GAIN * steering_p_output) + \
                                  (self.STEERING_I_GAIN * self.steering_i_output) + \
                                  (self.STEERING_D_GAIN * steering_d_output)
    
            # Steering goes from [-90,90] but we need to output [0,180] for the servos.
            self.steering_output = (self.STEERING_OFFSET + steering_pid_output) % 180
    
            #
            # Figure out throttle and do throttle PID
            #
    
            throttle_new_result = self.figure_out_my_throttle(self.steering_output)
            throttle_delta_result = (throttle_new_result - self.throttle_old_result) if (self.throttle_old_result != None) else 0
            self.throttle_old_result = throttle_new_result
    
            throttle_p_output = throttle_new_result # Standard PID Stuff here... nothing particularly interesting :)
            self.throttle_i_output = max(min(self.throttle_i_output + throttle_new_result, self.THROTTLE_I_MAX), self.THROTTLE_I_MIN)
            throttle_d_output = ((throttle_delta_result * 1000) / self.fps.delta()) if self.fps.delta() else 0
            throttle_pid_output = (self.THROTTLE_P_GAIN * throttle_p_output) + \
                                  (self.THROTTLE_I_GAIN * self.throttle_i_output) + \
                                  (self.THROTTLE_D_GAIN * throttle_d_output)
    
            # Throttle goes from 0% to 100%.
            self.throttle_output = max(min(throttle_pid_output, 100), 0)
        else:
            self.throttle_output = self.throttle_output * 0.99
            self.steering_output = self.STEERING_OFFSET
            
        self.jsonreadout()
        
        if not self.paused.state:
            try:
                client, addr = self.listener.accept()
            except socket.timeout:
                pass
            except:
                raise
            else:
                data = client.recv(1024).decode()
                if data == 'PAUSE':
                    print('PAUSE triggered by client', file=self.log_file, flush=True)
                    self.paused.state = not self.paused.state
                    client.send(data.encode())
        
        if self.paused.state:
            self.throttle_output = 0
            self.steering_output = self.STEERING_OFFSET
            
            try:
                client, addr = self.listener.accept()
            except socket.timeout:
                pass
            except:
                raise
            else:
                data = client.recv(1024).decode()
                if data == 'UNPAUSE':
                    print('UNPAUSE triggered by client', file=self.log_file, flush=True)
                    self.paused.state = not self.paused.state
                    client.send(data.encode())
                elif data == 'LOAD':
                    print('LOAD triggered by client', file=self.log_file, flush=True)
                    self.load('testjson.json')
                    

        # self.set_servos(self.throttle_output, self.steering_output)
        return(self.paused.state, self.throttle_output, self.steering_output, self.fps.get())

    def __init__(self):
        # Start up the pause button handler
        self.load('testjson.json')
        print("LOAD COMPLETE MWUAHAHA")
        self.paused = PauseButtonEvent()
        self.paused.start()
        self.enable_steering_and_throttle()
        self.fps = track_fps()
        
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(('127.0.0.1', 3002))
        self.listener.settimeout(0.001)
        print('listening on 127.0.0.1:3002')
        self.listener.listen(5)
        
        # Output for tracking signals received
        self.log_port = 3005
        self.log_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.log_out.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.log_out.connect(("", self.log_port))
        self.log_file = self.log_out.makefile('w', buffering=None)
        
        # Output for viewing current variables
        self.vari_port = 3006
        self.vari_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vari_out.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.vari_out.connect(("", self.vari_port))
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
