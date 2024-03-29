# Kalman Smoother using Fraser Algorithm
 A high accurate Kalman smoother implementation.  
Only "fixed-interval smoothing" is implemented on this repository.  
You can correct estimated information which is comming from Kalman Filter.  
Because Fraser Algorithm on this code, it is quite accurate.  

## Features

 1. Fixed-interval smoothing is implemented. you can correct information in past.  
 2. It works stably even for float16 matrices.  
    Because this implementation uses Fraser Algorithm.  

## Introduce to environment.

```bash
git clone --recursive [repository]
```

## Theory

### Overveiw

Kalman smoother is going to correct predicted random variables which is computed by Kalman filter process. Although Kalman filter is online process, Kalman smoother corrects and updates information which is gotten in the past.  
There are 3 types of Kalman smoother.  

 1. fixed-interval smoothing
 2. fixed-point smoothing  
 3. fixed-lag smoothing  
 I introduce and implement (1) fixed-interval smoothing here.  
In fixed-interval smoothing, Kalman smoother needs random variables of Kalman filter on ALL time steps.
After getting predicted random variables on ALL time steps [0-T],  
Kalman smoother back-propagates these variables and corrects them.
It is called "fixed-interval smoothing".  

###  Introction of Normal fixed-lag Kalman smoothing 
 There are 2 steps to use fixed-lag Kalman smoothing.  
(1) Estimate state variables using Kalman Filter on each time-step [0-T].  
(2) Start correcting from estimated state variable on the last time-step [T],  
    and back-propagate information to the backward, and Finally arrives first time-step [0].

Normal fixed-lag Kalman smoothing is given as follows.  
ALL estimated state variables can be computed in order by using this formula.  

* Estimated variables by smoother.

```math
\begin{align}
    \hat{s}_{t/N} = \hat{s}_{t/t} + C_t [\hat{s}_{t+1/N} - \hat{s}_{t+1/t}] \quad t=N-1,,,0 \\
\end{align}
```

* Gain

```math
\begin{align}
    C_t = P_{t/t}F_t^T(P_{t+1/t})^{-1}
\end{align}
```

* Estimated error covariance matrix
<!-- <img src="https://github.com/kevin-tofu/KalmanSmoother_C/blob/master/imgs/eq3.jpg" alt="eq3" title="formulation3"> -->
```math
\begin{align}
    P_{t/N} = P_{t/t} + C_t [P_{t+1/N}-P_{t+1/t}] C_{t}^{T}
\end{align}
```

 This formulation is easy to understand, but sometimes it can be unstable when  

 ```math
\begin{align} 
    [\hat{s}_{t+1/N} - \hat{s}_{t+1/t}] \\
    [P_{t+1/N}-P_{t+1/t}] 
\end{align}
 ```

 takes small values, or  

```math
$ \begin{align} 
    (P_{t+1/t})^{-1} 
\end{align}
```

 takes huge value. So, Lost of digits can be happend on computers, and become numerically unstable.  
Fraser Algorithm was introduced to avoid this point.

### Kalman Smoother using Fraser Algorithm

Fraser Algorithm is defined as follows.  
The basic definition of characters that are used in this formula is as same as  in (https://github.com/kevin-tofu/KalmanFilter_in_C).  

```math
\begin{align}
    \hat{s}_{t/T} = \hat{s}_{t/t} + P_{t/t}F_t^{T} \lambda_{t+1} \\
    {\lambda}_{t} = \hat{F}^{T} {\lambda}_{t+1} \hat{H}_t^{T} [H_tP_{t/t-1}H_t^{T}+R_t]^{-1} {\nu}_t \\
    {\lambda}_{N+1} = 0 \\
    \hat{F}^{T} = F_t ( I - K_t H_t) \\
    {\nu}_t = [m_t - H_t \hat{s}_{t/t-1} ] \\
    P_{t/N} = (I - C_t F_t) P_{t/t} (I - C_t F_t)^{T} + C_t (P_{t+1/N}+G_tQ_tG_t^{T})C_t^{T}
\end{align}
```

## Functions and explanations

### Functions

|Function name|Explanations|Arguments|
|:---|:---|:---|
|fKalmanSmoothing_New|dynamically allocate structure data on memory for computing kalmansmoother process, and initialize thoes values.|int, int : dimention of state and measurement|
|fKalmanSmoothing_Initialize|initialize values on data.|_stKalmanSmoothing*|
|fKalmanSmoothing_Delete| release allocated memories.|_stKalmanSmoothing*|
|fKalmanSmoothing_Run| Execute smoothing process at the time [t]. if you are going to get all prediction, need to run this process repeatedly. |_stKalmanSmoothing*|
