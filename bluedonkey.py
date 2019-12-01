#!/usr/bin/env python3
import os, sys, subprocess, socket
#import cgroups

def start_mjpg_streamer():
    print("Starting up mjpg_streamer.")
    # TODO: Add notification if either mjpg-streamer or
    #       cvfilter_py.so aren't installed
    # TODO: Detect any error if process exits, 
    #       such as the uvcvideo crash I'm seeing
    subprocess.run(["mjpg_streamer", "-i",
        "input_opencv.so -r 640x480 --filter /usr/lib/mjpg-streamer/cvfilter_py.so --fargs " + os.path.realpath(__file__),
        "-o",
        "output_http.so -p 8090 -w /usr/share/mjpg-streamer/www"],
        stdin=subprocess.PIPE
        #, stdout=subprocess.PIPE   #Commented to allow visibility of
        #, stderr=subprocess.PIPE   #responses from the system on commandline
        )

if __name__ == "__main__":
    start_mjpg_streamer()

# This method is called by the mjpg_streamer command run above. 
# This is what calls and executes the running code
def init_filter():
    ##  Socket streams that were here previously are 
    ##  now moved to multiple sockets where they are used.
    import line_follower
    dc = dummy_car_control()
    f = line_follower.mjs_filter(dc)
    print("Returning process")
    return f.process

# This class houses the car_control class
class dummy_car_control():
    def __init__(self):
        ## Commented per jkridner's advice
        import car_control
        self.c = car_control.car_control()
        
        #Output for the status in update method below
        self.status_port = 3004
        self.status_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status_out.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.status_out.connect(("", self.status_port))
        # This filehandle sends the data to the socket broadcast
        self.status_file = self.status_out.makefile('w', buffering=None)

    def tick(self):
        self.c.tick()
        return

    def update(self, line, threshold):
        (self.paused, self.throttle, self.steering, self.fps) = \
            self.c.update(line)
        
        # Code has been reworked to output to a separate filehandle pointing 
        # to the socket 3004, output to the dashboard under 'Status'
        # Replaced the Status output below to be a JSON string
        stri = "{"
        if self.paused:
            stri += '"Status":"Paused"'
        else:
            stri += '"Status":"Unpaused"'
        if line:
            stri += ', "Line_X":' + str(line[2]) + ', "Line_Y":' + str(line[3])
        else:
            stri += ', "Line_X":"No Line", "Line_Y":"No Line"'
        stri += ',"Throttle":' + str(self.throttle) + ',"Steering":' + \
            str(self.steering) 
        stri += ',"FPS":' + str(self.fps) + ',"Min_Threshold":' + \
            str(threshold) + '}'
        print(stri, "\r", end="", flush=True, file=self.status_file)
        return ""
