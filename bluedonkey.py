#!/usr/bin/env python3.7
import os, sys, subprocess

def start_mjpg_streamer():
    print("Starting up mjpg_streamer.")
    devnull = open(os.devnull, 'w')
    subprocess.run(["mjpg_streamer", "-i",
        "input_opencv.so -r 640x480 --filter /usr/local/lib/mjpg-streamer/cvfilter_py.so --fargs " + os.path.realpath(__file__),
        "-o",
        "output_http.so -p 8090 -w /usr/share/mjpg-streamer/www"], 
        stdout=subprocess.PIPE,
        stderr=devnull)

if __name__ == "__main__":
    start_mjpg_streamer()
    sys.exit()

import line_follower
    
def init_filter():
    f = line_follower.mjs_filter()
    return f.process
