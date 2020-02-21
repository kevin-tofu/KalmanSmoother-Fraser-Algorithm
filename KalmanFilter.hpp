#pragma once

#include "matrix.hpp"

typedef struct stKalmanFilter 
{
    int vMeasureLengh;
    int vStateLength;
    _stMatrix* oMeasure;
    _stMatrix* oH;
    _stMatrix* oF;
    _stMatrix* oState_pre;
    _stMatrix* oErrCov_pre;
    _stMatrix* oCovQ;
    _stMatrix* oG;
    _stMatrix* oKalmanGain_cor;
    _stMatrix* oState_cor;
    _stMatrix* oErrCov_cor;
    _stMatrix* oCovR;
    
}_stKalmanFilter;



extern _stKalmanFilter* fKalmanFilter_New(int measure, int state);
extern int fKalmanFilter_Initialize(_stKalmanFilter* This);
extern int fKalmanFilter_Delete(_stKalmanFilter* This);
extern int fKalman_KalmanFilter(_stKalmanFilter* This);
extern int fKalmanFilter_MeasurementUpdate(_stKalmanFilter* This);
extern int fKalmanFilter_TimeUpdate(_stKalmanFilter* This);
extern int fKalmanFilter_PriorEstimate(_stKalmanFilter* This);
