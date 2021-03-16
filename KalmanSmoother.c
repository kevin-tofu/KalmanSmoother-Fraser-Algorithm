//Copyright © 2011- Kohei-tofu. All rights reserved.
//koheitech001 [at] gmail.com

#include <stdio.h>
#include <stdlib.h>
#include "KalmanSmoother.h"

///
///�֐���`_KalmanSmoothing
///
_stKalmanSmoothing* fKalmanSmoothing_New(int measure, int state);
int fKalmanSmoothing_Initialize(_stKalmanSmoothing* This);
int fKalmanSmoothing_Delete(_stKalmanSmoothing* This);
int fKalmanSmoothing_Run(_stKalmanSmoothing* This);



/*!
KalmanSmoothing�̈�m��
@param    
@return   
@note     
*/
_stKalmanSmoothing* fKalmanSmoothing_New(int measure, int state)
{
	//
	_stKalmanSmoothing* retAddress = (_stKalmanSmoothing*)malloc(sizeof(_stKalmanSmoothing));

	retAddress->vMeasureLengh = measure;
	retAddress->vStateLength = state;

	retAddress->oMeasure = fMat_New(measure,1);//_m1

	retAddress->oH = fMat_New(measure,state);//_ms
	retAddress->oF = fMat_New(state,state);//_ss
	retAddress->oState_pre = fMat_New(state,1);//_s1
	retAddress->oErrCov_pre = fMat_New(state,state);//_ss

	retAddress->oKalmanGain_cor = fMat_New(state,measure);//_sm
	retAddress->oState_cor = fMat_New(state,1);//_s1
	retAddress->oErrCov_cor = fMat_New(state,state);//_ss
	retAddress->oCovR = fMat_New(measure,measure);//_mm

	retAddress->oLambda = fMat_New(state,1);//oState_Smoothing
	retAddress->oState_Smoothing = fMat_New(state,1);

	//NULL check���ׂ�
	fKalmanSmoothing_Initialize(retAddress);

	return retAddress;
}

/*!
KalmanSmoothing������
@param    
@return   
@note     
*/
int fKalmanSmoothing_Initialize(_stKalmanSmoothing* This)
{
	int lcounter = 0;

	lcounter += fMat_Zero(This->oLambda);

	return lcounter;
}

/*!
KalmanSmoothing�������̈�폜
@param
@return
@note
*/
int fKalmanSmoothing_Delete(_stKalmanSmoothing* This)
{
	fMat_Delete(This->oLambda);
	fMat_Delete(This->oMeasure);
	fMat_Delete(This->oH);
	fMat_Delete(This->oF);
	fMat_Delete(This->oState_pre);
	fMat_Delete(This->oErrCov_pre);
	fMat_Delete(This->oKalmanGain_cor);
	fMat_Delete(This->oState_cor);
	fMat_Delete(This->oErrCov_cor);
	fMat_Delete(This->oCovR);

	//fMat_Delete(This->vMeasureErr);
	//fMat_Delete(This->vSystemErr);

	fMat_Delete(This->oState_Smoothing);

	free(This);

	return 0;
}

/*!
KalmanSmoothing Algolithm
@param    _stKalmanSmoothing* This)
@return   Error��
@note     
@note     
@note     �g�������炵�āA���͉͂󂵂�����Ă����̂����B
*/
int fKalmanSmoothing_Run(_stKalmanSmoothing* This)
{
	int lcounter = 0;
	int measure = This->vMeasureLengh;
	int state = This->vStateLength;

	//�����Ă��̏ꍇ�A���̊֐����̌v�Z�͌J��Ԃ��A�����čs���̂�
	//�����̗̈�͊֐����Ŋm�ہA�J�����Ȃ��ق����悢����
	_stMatrix* lMat_ss = fMat_New(state,state);
	_stMatrix* lMat_s1 = fMat_New(state,1);
	_stMatrix* lMat_m1 = fMat_New(measure,1);
	_stMatrix* lMat_mm = fMat_New(measure, measure);
	_stMatrix* lMat_sm = fMat_New(state, measure);
	_stMatrix* lMat_ms = fMat_New(measure, state);
	_stMatrix* lF_Tilde_ss = fMat_New(state,state);

	///��������l�̍X�V
	///x(t/N)^ = x(t/t)^ + P(t/t)Ft��(t+1)
	///

	//P(t/t)Ft
	lcounter += fMat_MltTrans(lMat_ss, This->oErrCov_cor, This->oF);
	//P(t/t)FT��(t+1)
	lcounter += fMat_Mlt(This->oState_Smoothing, lMat_ss, This->oLambda);
	//x(t/N)^ = x(t/t)^ + P(t/t)Ft��(t+1)
	lcounter += fMat_Add2(This->oState_Smoothing, This->oState_cor);

	//fMat_UnitMatrix
	//fMat_Zero
	
	///�␳���X�V
	///��t = �@+�A
	///

	//
	//F~ = Ft(I - KH)
	//

	//set I
	lcounter += fMat_UnitMatrix(lMat_ss);

	//set KH
	lcounter += fMat_Mlt(lF_Tilde_ss, This->oKalmanGain_cor, This->oH);

	//set (I - KH)
	lcounter += fMat_Sub2(lMat_ss, lF_Tilde_ss);

	//set F~ = F(I - KH)
	lcounter += fMat_Mlt(lF_Tilde_ss, This->oF, lMat_ss);

	//��(t) = (F~)t ��(t+1)
	lcounter += fMat_TransMlt(lMat_s1, lF_Tilde_ss, This->oLambda);

	//��(t) = (F~)t ��(t+1)
	lcounter += fMat_Copy(This->oLambda, lMat_s1);

	//
	//Ht[HPHt + R]^-1[y-Hx]
	//

	//H = I�̏ꍇ��ǉ����ׂ���
	//HP ms ss = ms
	lcounter += fMat_Mlt(lMat_ms, This->oH, This->oErrCov_pre);

	//(HP)Ht ms sm = mm
	lcounter += fMat_MltTrans(lMat_mm, lMat_ms, This->oH);

	//HPHt + R
	lcounter += fMat_Add2(lMat_mm, This->oCovR);

	//(HPHt + R)^-1
	lcounter += fMat_InverseMatrix_Gauss2(lMat_mm);//!< Dynamically allocated

	//Ht[HPHt + R]^-1  (ms)t mm = sm mm = sm
	lcounter += fMat_TransMlt(lMat_sm, This->oH, lMat_mm);
	
	//Hx(t/t-1) ms s1 = m1
	lcounter += fMat_Mlt(lMat_m1, This->oH, This->oState_pre);
	
	//y-Hx (Innovation) 
	lcounter += fMat_Sub2(This->oMeasure, lMat_m1);//!< Measure���󂵂Ă���B�g���̂ĂȂ̂œ��ɖ��Ȃ�

	//Ht[HPHt + R]^-1[y-Hx]  sm m1 = s1
	lcounter += fMat_Mlt(lMat_s1, lMat_sm, This->oMeasure);

	//��(t)
	lcounter += fMat_Add2(This->oLambda, lMat_s1);


	fMat_Delete(lMat_ss);
	fMat_Delete(lMat_s1);
	fMat_Delete(lMat_m1);
	fMat_Delete(lMat_mm);
	fMat_Delete(lMat_sm);
	fMat_Delete(lMat_ms);
	fMat_Delete(lF_Tilde_ss);

	/*
	_stMatrix* lMat_ss = fMat_New(state,state);
	_stMatrix* lMat_s1 = fMat_New(state,1);
	_stMatrix* lMat_m1 = fMat_New(measure,1);
	_stMatrix* lMat_mm = fMat_New(measure, measure);
	_stMatrix* lMat_sm = fMat_New(state, measure);
	_stMatrix* lMat_ms = fMat_New(measure, state);
	_stMatrix* lF_Tilde_ss = fMat_New(state,state);
	*/

	return lcounter;
}
