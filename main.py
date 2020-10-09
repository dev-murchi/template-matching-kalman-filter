import os
import cv2
import numpy as np
import sys
import time

class TemplateMatch:
    def __init__(self, match_method = cv2.TM_CCORR_NORMED):
        ''' This function initializes the Template Matching Class '''
        self.source_video = ""
        self.cam = cv2.VideoCapture()
        self.with_kalman_filter = kf
        self.threshold = threshold
        self.match_method = match_method
        self.track_mode = "manual"
    def detect(self, frame, templ, w, h):
        ''' This function detects the template on source frame'''
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        match_result = cv2.matchTemplate(gray_frame, templ, self.match_method)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(match_result)
        if self.match_method == cv2.TM_SQDIFF or self.match_method == cv2.TM_SQDIFF_NORMED:
            top_left = min_loc
            match_val = min_val
        else:
            top_left = max_loc
            match_val = max_val
        return (match_val, top_left[0], top_left[1])
    def main(self):
        ok = self.cam.open(self.source_video)
        if not ok:
            print(". Error - could not open video file")
            return -1
        numOfFrames = int(self.cam.get(cv2.CAP_PROP_FRAME_COUNT))
        if 0 >= numOfFrames:
            print(". Error - could not read video file")
            return -1
        ok, frame = self.cam.read()
        if not ok:
            print(". Error - video frame could not read")
            return -1
        myRoi = cv2.selectROI("Tracking", frame, True, True)
        imCrop = frame[int(myRoi[1]) : int(myRoi[1]+myRoi[3]) , int(myRoi[0]) : int(myRoi[0]+myRoi[2])]
        template_gray = cv2.cvtColor(imCrop, cv2.COLOR_BGR2GRAY)
        template = (template_gray, template_gray.shape[::-1])
        frame_count = 0
        self.cam.set(cv2.CAP_PROP_POS_FRAMES, frame_count)
        while True:
            ok, frame = self.cam.read()
            if not ok:
                print(". Error - video frame could not read")
                break
            tw = int(template[1][0])
            th = int(template[1][1])
            matchedValue, differenceX, differenceY = self.detect(frame, template[0], *template[1])
            cv2.rectangle(frame, (int(differenceX), int(differenceY)), (int(differenceX+tw), int(differenceY+th)), (255,0,0), 0)
            cv2.imshow("Tracking", frame)
            print("frame num: ", frame_count, " match: ", matchedValue)
            frame_count += 1
            if (cv2.waitKey(1) & 0xFF) == 27:
                break
        cv2.destroyAllWindows()
if __name__ == "__main__":
    app = TemplateMatch()
    app.source_video = "../template_matching_v1/videos/fish5_trial1_055hz_01cm.mov"
    app.main()
    print("=========== DONE =============")