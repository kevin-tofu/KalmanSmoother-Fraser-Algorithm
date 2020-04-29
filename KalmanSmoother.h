
#pragma once

#include "matrix.h"

typedef struct stKalmanSmoothing 
{
    int vMeasureLengh;
    int vStateLength;
    _stMatrix* oLambda;
    _stMatrix* oMeasure;
    _stMatrix* oH;
    _stMatrix* oF;
    _stMatrix* oState_pre;
    _stMatrix* oErrCov_pre;
    _stMatrix* oKalmanGain_cor;
    _stMatrix* oState_cor;
    _stMatrix* oErrCov_cor;
    _stMatrix* oCovR;
    
    _stMatrix* oState_Smoothing;
    
}_stKalmanSmoothing;



///
///�֐���`_KalmanSmoothing
///
extern _stKalmanSmoothing* fKalmanSmoothing_New(int measure, int state);
extern int fKalmanSmoothing_Initialize(_stKalmanSmoothing* This);
extern int fKalmanSmoothing_Delete(_stKalmanSmoothing* This);
extern int fKalmanSmoothing_Run(_stKalmanSmoothing* This);
