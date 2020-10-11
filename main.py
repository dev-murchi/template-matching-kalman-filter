import os
import cv2
import numpy as np
import sys
import time
import csv
class KalmanFilter:
    kf = cv2.KalmanFilter(4,2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    def Estimate(self, coordX, coordY):
        ''' This function estimates the position of the object '''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        return predicted
class TemplateMatch:
    def __init__(self,kalmanFilterEnable=False, match_method = cv2.TM_CCOEFF_NORMED, threshold=0.7):
        ''' This function initializes the Template Matching Class '''
        self.source_video = ""
        self.cam = cv2.VideoCapture()
        self.kalmanFilterEnable = kalmanFilterEnable
        self.threshold = threshold
        self.match_method = match_method
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
    def main(self, csvPath):
        ok = self.cam.open(self.source_video)
        if not ok:
            print("Error - could not open video file")
            return -1
        numOfFrames = int(self.cam.get(cv2.CAP_PROP_FRAME_COUNT))
        if 0 >= numOfFrames:
            print("Error - could not read video file")
            return -1
        ok, frame = self.cam.read()
        if not ok:
            print("Error - video frame could not read")
            return -1
        kalmanFilter = KalmanFilter()
        predictedResults = np.zeros((2,1), np.float32)
        camWidth= int(self.cam.get(3))
        camHeight = int(self.cam.get(4))
        searchBound = [0, 0, camWidth, camHeight]
        templateLocation = [0, 0, np.float32(camWidth), np.float32(camHeight)]
        frameCount = 0
        fishTracker = np.zeros((numOfFrames, 4), np.float32)
        keyInput = 0
        predictedCoordX = 0
        predictedCoordy = 0
        trackingStoped = True
        breakTracking = True
        if os.path.exists(csvPath):
            with open(csvPath, 'rt', newline = '') as csvFile:
                reader = csv.reader(csvFile)
                for row in reader:
                    fishTracker[int(float(row[0]))] = [float(row[0]), float(row[1]), float(row[2]), float(row[3])]
        else:
            with open(csvPath,'w') as fp:
                pass
        print("Press SPACE to select, ESC to exit, n to next, p to prev.")
        while True:
            self.cam.set(cv2.CAP_PROP_POS_FRAMES, frameCount)
            ok, frame = self.cam.read()
            if not ok:
                print("Error - video frame could not read")
                break
            frame_cpy = frame.copy()
            if not trackingStoped:
                tw = template[1][0]
                th = template[1][1]
                if (searchBound[0]+tw) <= searchBound[2] and (searchBound[1]+th) <= searchBound[3]:
                    areaToScan = frame[searchBound[1]:searchBound[3], searchBound[0]:searchBound[2]]
                    matchedValue, differenceX, differenceY = self.detect(areaToScan, template[0], *template[1])
                    templateLocation[0] = differenceX + searchBound[0]
                    templateLocation[1] = differenceY + searchBound[1]
                    templateLocation[2] = differenceX + searchBound[0] + tw
                    templateLocation[3] = differenceY + searchBound[1] + th
                    if matchedValue >= self.threshold:
                        fishTracker[frameCount] = [int(frameCount), 1, templateLocation[0]+tw*0.5, templateLocation[1]+th*0.5]
                        if self.kalmanFilterEnable:
                            templateLocationCpy = templateLocation
                            predictedResults = kalmanFilter.Estimate(templateLocation[0], templateLocation[1])
                            predictedCoordX, predictedCoordY, cov_x, cov_y = predictedResults[:,0]
                            if round(abs(cov_x)) < 10 and 0 < round(abs(cov_x)):
                                templateLocationCpy[0] = predictedCoordX
                                templateLocationCpy[2] = predictedCoordX + tw
                                x_margin = np.sqrt((np.square(matchedValue*cov_x)+np.square((1-matchedValue)*differenceX))/ 2)+5
                            else:
                                x_margin = np.sqrt((np.square((1-matchedValue)*(predictedCoordX-templateLocation[0]))+np.square(matchedValue*differenceX))/2) + 10
                            if round(abs(cov_y)) < 10 and 0 < round(abs(cov_y)):
                                templateLocationCpy[1] = predictedCoordY
                                templateLocationCpy[3] = predictedCoordY + th
                                y_margin = np.sqrt((np.square(matchedValue*cov_y)+np.square((1-matchedValue)*differenceY))/ 2)+5
                            else:
                                y_margin = np.sqrt((np.square((1-matchedValue)*(predictedCoordY-templateLocation[1]))+np.square(matchedValue*differenceY))/2) + 10 
                            searchBound[0] = int(abs(templateLocationCpy[0] - x_margin))
                            searchBound[1] = int(abs(templateLocationCpy[1] - y_margin))
                            searchBound[2] = int(abs(templateLocationCpy[2] + x_margin))
                            searchBound[3] = int(abs(templateLocationCpy[3] + y_margin))
                            if searchBound[0] < 0:
                                searchBound[0] = 0
                            if searchBound[1] <= 0:
                                searchBound[1] = 0
                            if searchBound[2]>= camWidth:
                                searchBound[2] = camWidth
                            if searchBound[3] >= camHeight:
                                searchBound[3] = camHeight
                        breakTracking = False
                        cv2.rectangle(frame_cpy, (int(templateLocation[0]), int(templateLocation[1])), (int(templateLocation[0]+tw), int(templateLocation[1]+th)), (255,0,0), 1)
                        cv2.putText(frame_cpy, '{}'.format(matchedValue), (50,120), cv2.FONT_HERSHEY_SIMPLEX , 1, (255,0,0), 2, cv2.LINE_AA) 
                    else:
                        cv2.putText(frame_cpy, '{}'.format(matchedValue), (50,120), cv2.FONT_HERSHEY_SIMPLEX , 1, (255,0,0), 2, cv2.LINE_AA) 
                        trackingStoped = True
                else:
                    trackingStoped = True
            else:
                if fishTracker[frameCount][1] == 1:
                    cv2.rectangle(frame_cpy, (int(fishTracker[frameCount][2]-4), int(fishTracker[frameCount][3]-4)), (int(fishTracker[frameCount][2]+4), int(fishTracker[frameCount][3]+4)), (0,255,0), 2)
            cv2.putText(frame_cpy, '{}'.format(frameCount), (50,50), cv2.FONT_HERSHEY_SIMPLEX , 1, (255,0,0), 2, cv2.LINE_AA) 
            cv2.namedWindow("Tracking", cv2.WINDOW_NORMAL)
            cv2.imshow("Tracking", frame_cpy)
            if trackingStoped:
                if not breakTracking:
                    print("Tracking failed, press SPACE to re-select, ESC to exit, n to next, p to prev.")
                keyInput = (cv2.waitKey(0) & 0xFF)
                if keyInput == 27:
                    break
                elif keyInput ==  32 : # ' '
                    myRoi = cv2.selectROI("Tracking", frame_cpy, True, True)
                    imCrop = frame[int(myRoi[1]) : int(myRoi[1]+myRoi[3]) , int(myRoi[0]) : int(myRoi[0]+myRoi[2])]
                    templateGray = cv2.cvtColor(imCrop, cv2.COLOR_BGR2GRAY)
                    template = (templateGray, templateGray.shape[::-1])
                    fishTracker[frameCount] = [frameCount, 1,myRoi[0] + myRoi[2]*0.5, myRoi[1] + myRoi[3]*0.5]
                    searchBound = [0, 0, camWidth, camHeight]
                    trackingStoped = False
                    breakTracking = False
                    print("Press ESC to exit, b to break track")
                elif keyInput == 110 : # 'n'
                    if frameCount >= numOfFrames-1:
                        frameCount = numOfFrames-2
                    breakTracking = True
                elif keyInput == 112 : # 'p'
                    frameCount = frameCount - 2
                    if frameCount < -1:
                        frameCount = -1
                    breakTracking =True
                else:
                    frameCount -= 1
            frameCount += 1
            keyInput = (cv2.waitKey(1) & 0xFF)
            if keyInput == 27:
                break
            elif keyInput == 98:
                trackingStoped = True
            
        cv2.destroyAllWindows()
        with open(csvPath, 'w', encoding='utf-8', newline='') as csvFile:
            writer = csv.writer(csvFile)
            for track in fishTracker:
                writer.writerow(track)
if __name__ == "__main__":
    if len(sys.argv) == 5:
        app = TemplateMatch()
        app.source_video = str(sys.argv[1])
        csvPath = str(sys.argv[2])
        app.threshold = float(sys.argv[3])
        app.kalmanFilterEnable = sys.argv[4]
        app.main(csvPath)
    print("=========== DONE =============")
