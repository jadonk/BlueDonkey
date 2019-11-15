#!/usr/bin/env python3.7
print("Loading Python modules for self.line_follower. Please be patient.")
import os, sys
import cv2, numpy
print("Done importing modules for now!")

# self.line follower routine implemented as an mjpg_streamer filter

    
class mjs_filter:
    
    def __init__(self, car_control):
        self.c = car_control
        self.load()

    ###########
    # Loop
    ###########
    
    def load(self):
        self.BINARY_VIEW = True # Include informational updates to image
        self.COLOR_THRESHOLD_MIN = 235
        self.COLOR_THRESHOLD_MAX = 254
        self.COLOR_THRESHOLD_DELTA = 1
        self.PERCENT_THRESHOLD_MIN = 3
        self.PERCENT_THRESHOLD_MAX = 20
        self.FRAME_WIDTH = 160
        self.FRAME_HEIGHT = 120
        self.FONT = cv2.FONT_HERSHEY_SIMPLEX
        self.frame_cnt = 0
        self.threshold = self.COLOR_THRESHOLD_MAX
        
        # Array of region of interest masks in the order they should be searched
        # Furthest away first
        self.roi_masks = numpy.array([
                # Focus on the center
                # 8/20ths in from the self.left
                # 10/20ths down from the self.top
                # 1/20ths tall
                # 4x1 pixel count
                [int(8*self.FRAME_WIDTH/20), int(10*self.FRAME_HEIGHT/20), int(1*self.FRAME_HEIGHT/20), int((4*self.FRAME_WIDTH/20)*(1*self.FRAME_HEIGHT/20)/100)],
                # Then look wider
                # 6/20ths in from the sides
                # 11/20ths down from the self.top
                # 1/20ths tall
                # 8x1 pixel count
                [int(6*self.FRAME_WIDTH/20), int(11*self.FRAME_HEIGHT/20), int(1*self.FRAME_HEIGHT/20), int((8*self.FRAME_WIDTH/20)*(1*self.FRAME_HEIGHT/20)/100)],
                # Then look wider
                # 4/20ths in from the sides
                # 12/20ths down from the self.top
                # 1/20ths tall
                # 12x1 pixel count
                [int(4*self.FRAME_WIDTH/20), int(12*self.FRAME_HEIGHT/20), int(1*self.FRAME_HEIGHT/20), int((12*self.FRAME_WIDTH/20)*(1*self.FRAME_HEIGHT/20)/100)],
                # Then really wide and taller
                # 0/20ths in from the sides
                # 13/20ths down from the self.top
                # 1/20ths tall
                # 20x1 pixel count
                [int(0*self.FRAME_WIDTH/10), int(13*self.FRAME_HEIGHT/20), int(1*self.FRAME_HEIGHT/20), int((20*self.FRAME_WIDTH/20)*(1*self.FRAME_HEIGHT/20)/100)],
            ], dtype=numpy.int32)

    def process(self, img):
        self.c.tick()

        # Image comes in at 640x480, so turn it into 160x120
        self.frame = img[::4,::4].copy()
        
        try:
            self.line = False
            self.pixel_cnt = 0
            self.pixel_cnt_min = int(self.PERCENT_THRESHOLD_MIN*self.roi_masks[2][3])
            self.pixel_cnt_max = int(self.PERCENT_THRESHOLD_MAX*self.roi_masks[2][3])
            for roi_mask in self.roi_masks:
                # roi_mask[0] pixels in from the sides
                # roi_mask[1] pixels down from the top
                # roi_mask[2] pixels high
                # roi_mask[3] number of pixels / 100
                if (not self.line) or (self.pixel_cnt < self.pixel_cnt_min):
                    # Extract self.blue only in ROI
                    self.top = roi_mask[1]
                    self.bottom = roi_mask[1] + roi_mask[2] - 1
                    self.left = roi_mask[0]
                    self.right = self.FRAME_WIDTH-roi_mask[0]-1
                    self.blue = self.frame[ self.top : self.bottom , self.left : self.right , 0 ]
                    # Zero out pixels below self.threshold
                    self.thresh_mask = cv2.inRange(self.blue, self.threshold, 255)
                    # Get array of pixel locations that are non-zero
                    self.pixelpoints = cv2.findNonZero(self.thresh_mask)
                    if self.pixelpoints is not None:
                        self.pixel_cnt = self.pixelpoints.size
                        self.pixel_cnt_min = int(self.PERCENT_THRESHOLD_MIN*roi_mask[3])
                        self.pixel_cnt_max = int(self.PERCENT_THRESHOLD_MAX*roi_mask[3])
                        self.vx = 0
                        self.vy = 1
                        self.y = int((self.top+self.bottom) / 2)
                        self.x = int(self.pixelpoints[:,:,0].mean()) + roi_mask[0]
                        self.line = [self.vx,self.vy,self.x,self.y]
                        if self.BINARY_VIEW:
                            self.thresh_color = cv2.cvtColor(self.thresh_mask, cv2.COLOR_GRAY2BGR)
                            self.frame[ self.top : self.bottom , self.left : self.right ] = self.thresh_color

            self.status = self.c.update(self.line, self.threshold)
            if self.BINARY_VIEW:
                #cv2.putText(self.frame, self.status, (10,self.FRAME_HEIGHT-(int(self.FRAME_HEIGHT/4))), self.FONT, 0.3, (150,150,255))
                if self.line:
                    self.frame = cv2.line(self.frame, (self.x,0), (self.x,self.y), (0,255,0), 2)

            # Adjust self.threshold if finding too few or too many pixels
            if self.pixel_cnt > self.pixel_cnt_max:
                self.threshold += self.COLOR_THRESHOLD_DELTA
                if self.threshold > self.COLOR_THRESHOLD_MAX:
                    self.threshold = self.COLOR_THRESHOLD_MAX
            if self.pixel_cnt < self.pixel_cnt_min:
                self.threshold -= self.COLOR_THRESHOLD_DELTA
                if self.threshold < self.COLOR_THRESHOLD_MIN:
                    self.threshold = self.COLOR_THRESHOLD_MIN

        except Exception as e:
            print(e)
            self.c.update(False, self.threshold)

        return self.frame
