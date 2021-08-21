# mmWave_radar_tracking
object tracking based on millimeter wave radar data

\* Author: [Zhihong Zhang](https://github.com/dawnlh)

## Introduction

Object tracking based on millimeter wave radar data with Kalman Filter algorithm. 

### Flow chart for the whole project

<img src="_asset/flow_chart.png" alt="single object tracking" style="zoom: 50%;" />



### Flow chart for the object tracking module

<img src="_asset/tracking_flow.png" alt="single object tracking" style="zoom: 50%;" />

## Demo

<img src="_asset/SOT.jpg" alt="single object tracking" style="zoom: 50%;" />

<img src="_asset/MOT.jpg" alt="single object tracking" style="zoom: 50%;" />

## Note

- In multiple object tracking, when two objects are overlapping, mistakes may occur. This problem can perhaps be solved by using a more robust `detectionToTrackAssignment.m` function. Specifically, we can take the doppler speed information and other statistic features into account when calculate the cost in `detectionToTrackAssignment.m` function. Currently, only distance between the locations of the detected objects and the tracks' predicting locations are considerer.
- The performance of the implemented algorithm is very dependent on parameter tuning, especially the parameters of DBSCAN and the tracking module (like parameters in `detectionToTrackAssignment.m` and `updateTrackStates.m`).



## Reference

- P. Zhao *et al.*, “mID: Tracking and Identifying People with Millimeter Wave Radar,” in *2019 15th International Conference on Distributed Computing in Sensor Systems (DCOSS)*, Santorini Island, Greece, May 2019, pp. 33–40. doi: [10.1109/DCOSS.2019.00028](https://doi.org/10.1109/DCOSS.2019.00028).
- [MATLAB: Motion-Based Multiple Object Tracking](https://ww2.mathworks.cn/help/vision/ug/motion-based-multiple-object-tracking.html)
- [MATLAB: Use Kalman Filter for Object Tracking](https://ww2.mathworks.cn/help/vision/ug/using-kalman-filter-for-object-tracking.html)
