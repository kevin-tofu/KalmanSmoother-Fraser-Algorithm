# Kalman Smoother
 A high accurate Kalman smoother implementation.  
Only "fixed-interval smoothing" is implemented now.  
You can correct estimated information which is comming from Kalman Filter.  


## Features
 1. It works stably even for float16 matrices.  
    Because this implementation uses Matrix transformation model.  
 2. Fixed-interval smoothing is implemented. you can correct information in past.  


## Introduce to environment.
```
git clone --recursive [repository]
```


## Theory
### Overveiw
Kalman smoother is going to correct predicted random variables which is computed by Kalman filter process. Although Kalman filter is online process, Kalman smoother corrects and updates information which is gotten in the past.  
There are 3 types of Kalman smoother.  
(1) fixed-interval smoothing  
(2) fixed-point smoothing  
(3) fixed-lag smoothing  
 I introduce and implement (1) fixed-interval smoothing here.  
In fixed-interval smoothing, Kalman smoother needs random variables of Kalman filter on ALL time steps.
After getting predicted random variables on ALL time steps [0-T], 
Kalman smoother back-propagates these variables and corrects them.
It is called "fixed-interval smoothing".  

### introction of Normal fixed-lag Kalman smoothing 
 There are 2 steps to use fixed-lag Kalman smoothing.  
(1) Estimate state variables using Kalman Filter on each time-step [0-T].  
(2) Start correcting from estimated state variable on the last time-step [T],  
    and back-propagate information to the backward, and Finally arrives first time-step [0].

Normal fixed-lag Kalman smoothing is given as follows.  


### Kalman Smoother ( using matrix transoformation )


## Functions and explanations.

### Functions
|Function name|Explanations|Arguments|
|:---|:---|:---|
|fKalmanSmoothing_New|dynamically allocate structure data on memory for computing kalmansmoother process, and initialize thoes values.|int, int : dimention of state and measurement|
|fKalmanSmoothing_Initialize|initialize values on data.|_stKalmanSmoothing*|
|fKalmanSmoothing_Delete| release allocated memories.|_stKalmanSmoothing*|
|fKalmanSmoothing_Run| Execute smoothing process at the time [t]. <br> if you are going to get all prediction, need to run this process repeatedly. |_stKalmanSmoothing*|

