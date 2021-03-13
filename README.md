# Kalman Smoother

## Features
This is a Kalman Smoother implementation.  
Therefore this implementation uses the Matrix transformation model,  
estimation results are stable and accurate even in an environment that single-precision single-precision floating-point matrices are used.  
And what's more, 
It is possible to deal with operations even on a weak CPU, like an embedded system environment.  


## Introduce to environment.
```
git clone --recursive [repository]
```


## Teory
Kalman filter is going to correct predicted random variables which is computed by Kalman filter process. 
So, Kalman smoother needs random variables of Kalman filter on ALL time steps.
After getting predicted random variables on ALL time steps [0-T], 
Kalman smoother back-propagates these variables and corrects them.


### Kalman Smoother ( normal implementation )


### Kalman Smoother ( using matrix transoformation )



## Functions and explanations.

### Functions
|Function name|Explanations|Arguments|
|:---|:---|:---|
|fKalmanSmoothing_New|dynamically allocate structure data on memory for computing kalmansmoother process, and initialize thoes values.|int, int : dimention of state and measurement|
|fKalmanSmoothing_Initialize|initialize values on data.|_stKalmanSmoothing*|
|fKalmanSmoothing_Delete| release allocated memories.|_stKalmanSmoothing*|
|fKalmanSmoothing_Run| Execute smoothing process at the time [t]. <br> if you are going to get all prediction, need to run this process repeatedly. |_stKalmanSmoothing*|

