#!/bin/sh
mjpg_streamer -i "input_opencv.so --filter /usr/local/lib/mjpg-streamer/cvfilter_py.so --fargs /var/lib/cloud9/BlueDonkey/mjs_filter.py" -o "output_http.so -p 8090 -w /usr/share/mjpg-streamer/www"
