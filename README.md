# template-matching-kalman-filter
Object detection and tracking using template matching and Kalman filter.
This was the continuation of my graduation project. 
I worked with my project supervisor Dr. İsmail UYANIK. Thanks to him for all his support.

# Requirements
* cv2
* numpy

# Usage
```
python main.py [sample_video_path] [tracking_output_csv_path] [threshold] [kalman_filter_enable_or_not]
```
# Example
```
python main.py videos/video1.mp4 csvFiles/video1_tracking.csv 0.7 True
```

# Notes:
When you run the code you will see some instructions on the terminal. 
These instructions give some information to you about choosing a template, move between frames, etc.
