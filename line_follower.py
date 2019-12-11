#!/usr/bin/env python3.7
print("Loading Python modules for line_follower. Please be patient.")
import os, sys
import cv2, numpy
print("Done importing modules for now!")

# line follower routine implemented as an mjpg_streamer filter

class mjs_filter:
    
    def __init__(self, dummy_car_control):
        self.c = dummy_car_control

    ###########
    # Loop
    ###########

    def process(self, img):
        self.c.tick()
        
        # Variable initialization in line_follower.py was moved to 
        # car_control.py load() method and references changed to support that. 
        # Unfortunately it gets kind of ugly.
        # This was necessary to allow the variables to be updated via json,
        # as car_control.py couldn't easily set variables in line_follower.py.
        # Thus 'self.c' refers to the 'dummy_car_control' object and
        # 'self.c.c' refers to the 'car_control' object in 'dummy_car_control'.

        # Image comes in at 640x480, so turn it into 160x120
        self.frame = img[::4,::4].copy()
        
        try:
            self.line = False
            self.pixel_cnt = 0
            self.pixel_cnt_min = int(
                self.c.c.jsondata["PERCENT_THRESHOLD_MIN"]* \
                self.c.c.roi_masks[2][3])
            self.pixel_cnt_max = int(
                self.c.c.jsondata["PERCENT_THRESHOLD_MAX"]* \
                self.c.c.roi_masks[2][3])
            for roi_mask in self.c.c.roi_masks:
                # roi_mask[0] pixels in from the sides
                # roi_mask[1] pixels down from the top
                # roi_mask[2] pixels high
                # roi_mask[3] number of pixels / 100
                if (not self.line) or (self.pixel_cnt < self.pixel_cnt_min):
                    # Extract self.blue only in ROI
                    self.top = roi_mask[1]
                    self.bottom = roi_mask[1] + roi_mask[2] - 1
                    self.left = roi_mask[0]
                    self.right = self.c.c.jsondata["FRAME_WIDTH"]-roi_mask[0]-1
                    self.blue = self.frame[ 
                        self.top : self.bottom , self.left : self.right , 0 ]
                    # Zero out pixels below self.threshold
                    self.thresh_mask = cv2.inRange(
                        self.blue, self.c.c.threshold, 255)
                    # Get array of pixel locations that are non-zero
                    self.pixelpoints = cv2.findNonZero(self.thresh_mask)
                    if self.pixelpoints is not None:
                        self.pixel_cnt = self.pixelpoints.size
                        self.pixel_cnt_min = int(
                            self.c.c.jsondata["PERCENT_THRESHOLD_MIN"]* \
                            roi_mask[3])
                        self.pixel_cnt_max = int(
                            self.c.c.jsondata["PERCENT_THRESHOLD_MAX"]* \
                            roi_mask[3])
                        self.vx = 0
                        self.vy = 1
                        self.y = int((self.top+self.bottom) / 2)
                        self.x = int(
                            self.pixelpoints[:,:,0].mean()) + roi_mask[0]
                        self.line = [self.vx,self.vy,self.x,self.y]
                        if self.c.c.jsondata["BINARY_VIEW"]:
                            self.thresh_color = cv2.cvtColor(
                                self.thresh_mask, cv2.COLOR_GRAY2BGR)
                            self.frame[ 
                                self.top : self.bottom , 
                                self.left : self.right ] = self.thresh_color

            self.status = self.c.update(self.line, self.c.c.threshold)
            if self.c.c.jsondata["BINARY_VIEW"]:
                #cv2.putText(self.frame, self.status, 
                #    (10,self.FRAME_HEIGHT-(int(self.FRAME_HEIGHT/4))), 
                #    self.FONT, 0.3, (150,150,255))
                if self.line:
                    self.frame = cv2.line(
                        self.frame, (self.x,0), (self.x,self.y), (0,255,0), 2)

            # Adjust self.threshold if finding too few or too many pixels
            if self.pixel_cnt > self.pixel_cnt_max:
                self.c.c.threshold += self.c.c.jsondata["COLOR_THRESHOLD_DELTA"]
                if self.c.c.threshold > self.c.c.jsondata["COLOR_THRESHOLD_MAX"]:
                    self.c.c.threshold = self.c.c.jsondata["COLOR_THRESHOLD_MAX"]
            if self.pixel_cnt < self.pixel_cnt_min:
                self.c.c.threshold -= self.c.c.jsondata["COLOR_THRESHOLD_DELTA"]
                if self.c.c.threshold < self.c.c.jsondata["COLOR_THRESHOLD_MIN"]:
                    self.c.c.threshold = self.c.c.jsondata["COLOR_THRESHOLD_MIN"]

        except Exception as e:
            print(e)
            self.c.update(False, self.c.c.threshold)

        return self.frame
