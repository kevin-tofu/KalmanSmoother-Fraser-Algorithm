/// <summary>
/// class : matrix
/// name  : �n� �_��
/// Date  : 2010/12/14
/// 
/// ����
/// 
///
/// </summary>

//#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>

//#include "cal.h"
#include "matrix.hpp"


//���͂̍s�񐔂����������`�F�b�N����
#define def_InputCheck

//row    : �s
//column : �� 

//@note      �vcheck
//�͓��얢�m�F

///
///Dynamically allocated
///
//new del
_stMatrix *fMat_New(int input_row, int input_col);
int fMat_Delete(_stMatrix* Mat);
_stMatrix* fMat_NewCopy(_stMatrix* This);
_stMatrix* fMat_NewCopy_rowVector(_stMatrix* This, int input_row);
_stMatrix* fMat_NewCopy_colVector(_stMatrix* This, int input_col);
matrix new_matrix(int nrow, int ncol);
void free_matrix(matrix a);
int fMat_Transpose2(_stMatrix* This);
int fMat_Mlt2(_stMatrix* A, _stMatrix* B);
int fMat_MltTrans2(_stMatrix* A, _stMatrix* B);
int fMat_InverseMatrix_Gauss2(_stMatrix* Input);
int fMat_TransMlt2(_stMatrix* A, _stMatrix* B);

//property
int fMat_Set(_stMatrix* This, int input_row, int input_col, def_ElementType_mat input);
def_ElementType_mat fMat_Get(_stMatrix* This, int input_row, int input_col);
def_ElementType_mat fMat_GetDiag(_stMatrix* This, int input_row);

//cal
int fMat_SetConst(_stMatrix* This, def_ElementType_mat input);
int fMat_Zero(_stMatrix* This);
int fMat_UnitMatrix(_stMatrix* This);
int fMat_Add(_stMatrix* C,_stMatrix* A,_stMatrix* B);
int fMat_Add2(_stMatrix* A,_stMatrix* B);
int fMat_Mlt(_stMatrix* C,_stMatrix* A,_stMatrix* B);
int fMat_MltTrans(_stMatrix* C,_stMatrix* A,_stMatrix* B);
int fMat_TransMlt(_stMatrix* C, _stMatrix* A, _stMatrix* B);
int fMat_Sub(_stMatrix* C,_stMatrix* A,_stMatrix* B);
int fMat_Sub2(_stMatrix* A,_stMatrix* B);
int fMat_UD_Degradation(_stMatrix* U, _stMatrix* D, _stMatrix* input);
int fMat_Multiplier(_stMatrix* output, _stMatrix* input, def_ElementType_mat Multiplier);
int fMat_Multiplier2(_stMatrix* This, def_ElementType_mat Multiplier);
int fMat_Multiplier_rowVector(_stMatrix* output, int output_row, _stMatrix* input, int input_row, def_ElementType_mat Multiplier);
int fMat_Multiplier_colVector(_stMatrix* output, int output_col, _stMatrix* input, int input_col, def_ElementType_mat Multiplier);
int fMat_Multiplier2_rowVector(_stMatrix* This, int output_row, def_ElementType_mat Multiplier);
int fMat_Multiplier2_colVector(_stMatrix* This, int output_col, def_ElementType_mat Multiplier);
int fMat_Copy(_stMatrix* output, _stMatrix* input);
int fMat_Copy_rowVector(_stMatrix* output, int output_row, _stMatrix* input, int input_row);
int fMat_Copy_colVector(_stMatrix* output, int output_col, _stMatrix* input, int input_col);
int fMat_Copy_colVector_TO_row(_stMatrix* output, int output_row, _stMatrix* input, int input_col);
int fMat_Copy_rowVector_TO_column(_stMatrix* output, int output_col, _stMatrix* input, int input_row);
int fMat_Add2_rowVector(_stMatrix* output, int output_row, _stMatrix* input, int input_row);
int fMat_Add2_colVector(_stMatrix* output, int output_col, _stMatrix* input, int input_col);
int fMat_InverseMatrix_Gauss(_stMatrix* Output, _stMatrix* Input);
int fMat_CholeskyDecomposition(_stMatrix* Output, _stMatrix* Input);
int fMat_Transpose(_stMatrix* output, _stMatrix* input);
int fMat_Check_SymmetricMatrix(_stMatrix* A);
def_ElementType_mat fMat_InnerProduct(_stMatrix* A, _stMatrix* B);
int fMat_InnerMatrix(_stMatrix* C, _stMatrix* A, _stMatrix* B);
int fMat_GetMaxElement_NotDiag(_stMatrix* input, int* p, int* q, def_ElementType_mat* max);

//typedef def_ElementType_mat *vector, **matrix;



/*!
@param    
@param    
@return  
@note     �s��̈�̔z�񕔕��m��
@note     	//nrow:�s�Ancol:��
*/
matrix new_matrix(int nrow, int ncol)
{
	matrix ret;

#ifdef def_ShortMemory
	ret = (matrix)malloc(sizeof(def_ElementType_mat) * nrow * ncol);
#else
	int i;

	ret = (matrix)malloc((nrow + 1) * sizeof(void *));//
	
	if (ret == NULL) return NULL;  /* �L���̈�s�� */
	
	for (i = 0; i < nrow; i++)
	{
		ret[i] = (vector)malloc(sizeof(def_ElementType_mat) * ncol);
		
		if (ret[i] == NULL)
		{
			while (--i >= 0) free(ret[i]);

			free(ret);  return NULL;  /* �L���̈�s�� */
		}
	}

	ret[nrow] = NULL;  /* �s�̐����������f���邽�߂̍H�v */
#endif
	return ret;
}


/*!

@param    
@param    
@return  
@note     �s��̈�J��
*/
void free_matrix(matrix a)
{
	
#ifdef def_ShortMemory
	free(a);
#else
	matrix b;

	b = a;
	
	while (*b != NULL) free(*b++);

	free(a);
#endif
}


/*!

@param    
@param    
@return  
@note     �s��̈�m��
*/
_stMatrix* fMat_New(int input_row, int input_col)
{
	
	_stMatrix* retAddress =  (_stMatrix*)malloc(sizeof(_stMatrix));
	//arg set
	retAddress->row = input_row;
	retAddress->column = input_col;

	//
	matrix Mataddress = new_matrix(input_row, input_col);
	retAddress->Mat = Mataddress;
	//

	fMat_Zero(retAddress);

	return retAddress;
}


/*!

@param    
@param    
@return  
@note     �s��̈�폜
*/
int fMat_Delete(_stMatrix* Mat)
{

	free_matrix(Mat->Mat);

	free(Mat);

	return 0;
}




/*!

@param    
@param    
@return  
@note     �s��̈抄�蓖��
@note     ���g�p�B�o�O���邩��
*/
//int fMat___dd(_stMatrix* Mat, double** point, double* valArg, int row, int col)
//{
//	int row_loop = 0;
//
//	Mat->column = row;
//	Mat->row = col;
//
//	Mat->Mat = point;
//
//	for(row_loop = 0; row_loop < row; row_loop++)
//	{
//		*(*point + row_loop) = valArg[row];
//	}
//
//
//	return 0;
//}

/*!

@param    
@param    
@return  
@note     �s��R�s�[
@note     �s��𕡐��i�̈�m�ہj���A�l���R�s�[����B
*/
_stMatrix* fMat_NewCopy(_stMatrix* This)
{
	_stMatrix* retAddress;
	int row_len = This->row;
	int col_len = This->column;
	int row_loop = 0;
	int col_loop = 0;

	retAddress = fMat_New(row_len, col_len);

	for(row_loop = 0; row_loop < row_len; row_loop++)
	{
		for(col_loop = 0; col_loop < col_len; col_loop++)
		{
			//copy value
			//retAddress, row_loop][col_loop] = This, row_loop][col_loop];
			fMat(retAddress, row_loop, col_loop) = fMat(This, row_loop, col_loop);
		}
	}

	return retAddress;
}


/*!

@param    _stMatrix*        : �R�s�[���̃A�h���X
@param    int input_row    : �R�s�[����s�ԍ�
@return  
@note     �s��R�s�[
@note     �V���ȍs����쐬�i�̈�m�ہj���A�w�肵���s�̂݃R�s�[����B
*/
_stMatrix* fMat_NewCopy_rowVector(_stMatrix* This, int input_row)
{
	_stMatrix* retAddress;
	int row_len = This->row;
	int col_len = This->column;
	//int row_loop = 0;
	int col_loop = 0;

	//retAddress = fMat_New(row, col);
	retAddress = fMat_New(1, col_len);

	//for(row_loop = 0; row_loop < row; row_loop++)
	{
		for(col_loop = 0; col_loop < col_len; col_loop++)
		{
			//retAddress, 0][col_loop] = This, input_row][col_loop];
			fMat(retAddress, 0, col_loop) = fMat(This, input_row, col_loop);
		}
	}

	return retAddress;
}

/*!

@param    _stMatrix*        : �R�s�[���̃A�h���X
@param    int input_col    : �R�s�[�����ԍ�
@return  
@note     �s��R�s�[
@note     �V���ȍs����쐬�i�̈�m�ہj���A�w�肵����̂݃R�s�[����B
*/
_stMatrix* fMat_NewCopy_colVector(_stMatrix* This, int input_col)
{
	_stMatrix* retAddress;
	int row_len = This->row;
	int col_len = This->column;
	int row_loop = 0;
	//int col_loop = 0;

	//retAddress = fMat_New(row, col);
	retAddress = fMat_New(row_len, 1);

	for(row_loop = 0; row_loop < row_len; row_loop++)
	{
		//for(col_loop = 0; col_loop < col_len; col_loop++)
		{
			//retAddress, row_loop][0] = This, row_loop][input_col];
			fMat(retAddress, row_loop, 0) = fMat(This, row_loop, input_col);
		}
	}

	return retAddress;
}

/*!

@param    _stMatrix* output
@param    _stMatrix* input
@return  
@note     �s��R�s�[
@note     output = input �s��l�𕡐�����B�s���A�񐔂������łȂ���΂Ȃ�Ȃ�
*/
int fMat_Copy(_stMatrix* output, _stMatrix* input)
{
	int lErrorCounter = 0;

	int row_len = input->row;
	int col_len = input->column;
	int row_loop = 0;
	int col_loop = 0;

#ifdef def_InputCheck
	if((row_len == output->row)
	 &&(col_len == output->column)
	 )
#else
	if(true)
#endif
	{

		for(row_loop = 0; row_loop < row_len; row_loop++)
		{
			for(col_loop = 0; col_loop < col_len; col_loop++)
			{
				//output, row_loop][col_loop] = input, row_loop][col_loop];
				fMat(output, row_loop, col_loop) = fMat(input, row_loop, col_loop);
			}
		}
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}

/*!

@param    _stMatrix* output
@param    int output_row
@param    _stMatrix* input
@param    int input_row
@return  
@note     �s��R�s�[�B�w�肵���s�̒l���w�肵���s�ɏo�͂���B�񐔁i�s�̗v�f���j�������K�v������B
@note     �w�肵���s�ԍ����o�͐�̍s�ԍ���菬����
*/
int fMat_Copy_rowVector(_stMatrix* output, int output_row, _stMatrix* input, int input_row)
{
	int lErrorCounter = 0;
	//_stMatrix* retAddress;
	int row_len = input->row;
	int col_len = input->column;
	//int row_loop = 0;
	int col_loop = 0;

	//retAddress = fMat_New(row, col);
	//retAddress = fMat_New(1, col);

	//for(row_loop = 0; row_loop < row; row_loop++)
#ifdef def_InputCheck
	if((col_len == output->column)
		&&(input_row < row_len)//�w�肵���s�ԍ����o�͐�̍s�ԍ���菬����
		&&(output_row < output->row)
		)
#else
	if(true)
#endif
	{
		for(col_loop = 0; col_loop < col_len; col_loop++)
		{
			//output, output_row][col_loop] = input, input_row][col_loop];
			fMat(output, output_row, col_loop) = fMat(input, input_row, col_loop);
		}
	}
	else
	{
		lErrorCounter++;
	}


	return lErrorCounter;
}

/*!

@param    _stMatrix* output
@param    int output_row
@param    _stMatrix* input
@param    int input_row
@return  
@note     �w�肵���s�x�N�g�����w�肵���s��Add2
@note     �񐔁i�s�̗v�f���j�������K�v������B
@note     �w�肵���s�ԍ����o�͐�̍s�ԍ���菬����
*/
int fMat_Add2_rowVector(_stMatrix* output, int output_row, _stMatrix* input, int input_row)
{
	int lErrorCounter = 0;
	//_stMatrix* retAddress;
	int row = input->row;
	int col = input->column;
	//int row_loop = 0;
	int col_loop = 0;

	//retAddress = fMat_New(row, col);
	//retAddress = fMat_New(1, col);

	//for(row_loop = 0; row_loop < row; row_loop++)
#ifdef def_InputCheck
	if((col == output->column)
		&&(input_row < row)
		&&(output_row < output->row)
		)
#else
	if(true)
#endif
	{
		for(col_loop = 0; col_loop < col; col_loop++)
		{
			fMat(output, output_row, col_loop) += fMat(input, input_row, col_loop);
		}
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}

/*!

@param    _stMatrix* output
@param    int output_row
@param    _stMatrix* input
@param    int input_row
@return  
@note     �s��R�s�[�B�w�肵����̒l���w�肵����ɏo�͂���B�s���i��̗v�f���j�������K�v������B
@note     �w�肵����ԍ����o�͐�̗�ԍ���菬����  
*/
int fMat_Copy_colVector(_stMatrix* output, int output_col, _stMatrix* input, int input_col)
{
	int lErrorCounter = 0;
	//_stMatrix* retAddress;
	int row_len = input->row;
	int col_len = input->column;
	int row_loop = 0;
	//int col_loop = 0;

	//retAddress = fMat_New(row, col);
	//retAddress = fMat_New(row, 1);

#ifdef def_InputCheck
	if((row_len == output->row)
		&&(input_col < col_len)
		&&(output_col < output->column)
		)
#else
	if(true)
#endif
	for(row_loop = 0; row_loop < row_len; row_loop++)
	{
		//for(col_loop = 0; col_loop < col; col_loop++)
		{
			fMat(output, row_loop, output_col) = fMat(input, row_loop, input_col);
		}
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}


/*!

@param    _stMatrix* output
@param    int output_row
@param    _stMatrix* input
@param    int input_col
@return  
@note     �s��R�s�[�B�w�肵������w�肵���s�ɏo�͂���B��̗v�f���iinput�j�ƍs�̗v�f���ioutput�j�������K�v������B
@note     �w�肵����ԍ����o�͐�̗�ԍ���菬����
@note     �vcheck
*/
int fMat_Copy_colVector_TO_row(_stMatrix* output, int output_row, _stMatrix* input, int input_col)
{
	int lErrorCounter = 0;
	//_stMatrix* retAddress;
	int row_len = output->row;
	int col_len = output->column;
	//int row_loop = 0;
	int col_loop = 0;

	//retAddress = fMat_New(row, col);
	//retAddress = fMat_New(row, 1);

#ifdef def_InputCheck
	if((input->row == output->column)
		&&(input_col < input->column)
		&&(output_row < output->row)
		)
#else
	if(true)
#endif
	//for(row_loop = 0; row_loop < row_len; row_loop++)
	{
		for(col_loop = 0; col_loop < col_len; col_loop++)
		{
			fMat(output, output_row, col_loop) = fMat(input, col_loop, input_col);
		}
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}

/*!

@param    _stMatrix* output
@param    int output_row
@param    _stMatrix* input
@param    int input_col
@return  
@note     �s��R�s�[�B�w�肵������w�肵���s�ɏo�͂���B��̗v�f���iinput�j�ƍs�̗v�f���ioutput�j�������K�v������B
@note     �w�肵����ԍ����o�͐�̗�ԍ���菬����
@note     �vcheck
*/
int fMat_Copy_rowVector_TO_column(_stMatrix* output, int output_col, _stMatrix* input, int input_row)
{
	int lErrorCounter = 0;
	//_stMatrix* retAddress;
	int row_len = output->row;
	int col_len = output->column;
	int row_loop = 0;
	//int col_loop = 0;

	//retAddress = fMat_New(row, col);
	//retAddress = fMat_New(row, 1);

#ifdef def_InputCheck
	if((input->column == output->row)
		&&(input_row < input->row)
		&&(output_col < output->column)
		)
#else
	if(true)
#endif
	for(row_loop = 0; row_loop < row_len; row_loop++)
	{
		//for(col_loop = 0; col_loop < col_len; col_loop++)
		{
			fMat(output, row_loop, output_col) = fMat(input, input_row, row_loop);
		}
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}

/*!

@param    _stMatrix* output
@param    int output_row
@param    _stMatrix* input
@param    int input_row
@return  
@note     �w�肵����x�N�g�����w�肵�����Add2
@note     �s���i��̗v�f���j�������K�v������B
@note     �w�肵����ԍ����o�͐�̗�ԍ���菬����
*/
int fMat_Add2_colVector(_stMatrix* output, int output_col, _stMatrix* input, int input_col)
{
	int lErrorCounter = 0;
	//_stMatrix* retAddress;
	int row = input->row;
	int col = input->column;
	int row_loop = 0;
	//int col_loop = 0;

	//retAddress = fMat_New(row, col);
	//retAddress = fMat_New(row, 1);
#ifdef def_InputCheck
	if((row == output->row)
		&&(input_col < col)
		&&(output_col < output->column)
		)
#else
	if(true)
#endif
	for(row_loop = 0; row_loop < row; row_loop++)
	{
		//for(col_loop = 0; col_loop < col; col_loop++)
		{
			fMat(output, row_loop, output_col) += fMat(input, row_loop, input_col);
		}
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}

/*!

@param    _stMatrix* output
@param    _stMatrix* input
@param    def_ElementType_mat Multiplier
@return  
@note     input�ɏ搔Multiplier���������l��output�ɏo�͂���
@note     A = kB
*/
int fMat_Multiplier(_stMatrix* output, _stMatrix* input, def_ElementType_mat Multiplier)
{
	int lErrorCounter = 0;

	int rowLoop,columnLoop;
	int rowLoopMax = output->row;
	int colLoopMax = output->column;

#ifdef def_InputCheck
	if((input->row == output->row)
		&&(input->column == output->column)
		)
#else
	if(true)
#endif
	for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
	{
		for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
		{
			fMat(output, rowLoop, columnLoop) = fMat(input, rowLoop, columnLoop) * Multiplier;
		}
	}
	else
	{
		lErrorCounter++;
	}


	return lErrorCounter;
}

/*!

@param    _stMatrix* This
@param    def_ElementType_mat Multiplier
@return  
@note     This�ɏ搔Multiplier���������l��This�ɏo�͂���
@note     A = kA
*/
int fMat_Multiplier2(_stMatrix* This, def_ElementType_mat Multiplier)
{
	int lErrorCounter = 0;

	int rowLoop,columnLoop;
	int rowLoopMax = This->row;
	int colLoopMax = This->column;
#ifdef def_InputCheck
	if(true)
#else
	if(true)
#endif
	for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
	{
		for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
		{
			fMat(This, rowLoop, columnLoop) *= Multiplier;
		}
	}
	else
	{
		lErrorCounter++;
	}


	return lErrorCounter;
}


/*!

@param    _stMatrix* output
@param    int output_row
@param    _stMatrix* input
@param    int input_row
@param    def_ElementType_mat Multiplier
@return  
@note     input�̎w�肵���s�x�N�g���ɏ搔Multiplier���������l��output�̎w�肵���s�x�N�g���ɏo�͂���
@note     A = kA
*/
int fMat_Multiplier_rowVector(_stMatrix* output, int output_row, _stMatrix* input, int input_row, def_ElementType_mat Multiplier)
{
	int lErrorCounter = 0;

	int columnLoop;//rowLoop,
	int rowLoopMax = output->row;
	int colLoopMax = output->column;

	//for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
#ifdef def_InputCheck
	if((input->column == output->column)
		&&(output_row < output->row)
		&&(input_row < input->row)
		)
#else
	if(true)
#endif
	{
		for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
		{
			fMat(output, output_row, columnLoop) = fMat(input, input_row, columnLoop) * Multiplier;
		}
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}


/*!

@param    _stMatrix* This
@param    def_ElementType_mat Multiplier
@return  
@note     This�̎w�肵���s�x�N�g���ɏ搔Multiplier���������l��This�ɏo�͂���
@note     A = kA
*/
int fMat_Multiplier2_rowVector(_stMatrix* This, int output_row, def_ElementType_mat Multiplier)
{
	int lErrorCounter = 0;

	int columnLoop;//rowLoop,
	int rowLoopMax = This->row;
	int colLoopMax = This->column;

	//for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
#ifdef def_InputCheck
	if((output_row < This->row)
		)
#else
	if(true)
#endif
	{
		for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
		{
			fMat(This, output_row, columnLoop) *= Multiplier;
		}
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}



/*!

@param    _stMatrix* output
@param    int output_row
@param    _stMatrix* input
@param    int input_row
@param    def_ElementType_mat Multiplier
@return  
@note     input�̎w�肵����x�N�g���ɏ搔Multiplier���������l��output�̎w�肵����x�N�g���ɏo�͂���
@note     A = kA
*/
int fMat_Multiplier_colVector(_stMatrix* output, int output_col, _stMatrix* input, int input_col, def_ElementType_mat Multiplier)
{
	int lErrorCounter = 0;

	int rowLoop;//,columnLoop;
	int rowLoopMax = output->row;
	//int colLoopMax = output->column;

#ifdef def_InputCheck
	if((input->row == output->row)
		&&(output_col < output->column)
		&&(input_col < input->column)
		)
#else
	if(true)
#endif
	for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
	{
		//for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
		{
			fMat(output, rowLoop, output_col) = fMat(input, rowLoop, input_col) * Multiplier;
		}
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}


/*!

@param    _stMatrix* This
@param    def_ElementType_mat Multiplier
@return  
@note     This�̎w�肵����x�N�g���ɏ搔Multiplier���������l��This�ɏo�͂���
@note     A = kA
*/
int fMat_Multiplier2_colVector(_stMatrix* This, int output_col, def_ElementType_mat Multiplier)
{
	int lErrorCounter = 0;

	int rowLoop;//,columnLoop;
	int rowLoopMax = This->row;
	//int colLoopMax = output->column;

#ifdef def_InputCheck
	if((output_col < This->column)
		)
#else
	if(true)
#endif
	for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
	{
		//for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
		{
			fMat(This, rowLoop, output_col) *= Multiplier;
		}
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}


/*!

@param    _stMatrix* This
@return  
@note     �w�肵���s���0�s��ɂ���
@note     A = 0
*/
int fMat_Zero(_stMatrix* This)
{
	int lErrorCounter = 0;

	int rowLoop,columnLoop;
	int rowLoopMax = This->row;
	int colLoopMax = This->column;

	for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
	{
		for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
		{
			fMat(This, rowLoop, columnLoop) = 0;
		}
	}

	return lErrorCounter;
}
/*!

@param    _stMatrix* This
@return  
@note     �w�肵���s��̗v�f�S�Ăɒ萔����������
@note     A = I
*/
int fMat_SetConst(_stMatrix* This, def_ElementType_mat input)
{
	int lErrorCounter = 0;

	int rowLoop,columnLoop;
	int rowLoopMax = This->row;
	int colLoopMax = This->column;

	for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
	{
		for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
		{
			fMat(This, rowLoop, columnLoop) = input;
		}
	}

	return lErrorCounter;
}

/*!

@param    _stMatrix* This
@return  
@note     �w�肵���s���P�ʍs��ɂ���
@note     A = I
@note     �����s��ł���K�v����������B�i���̐����͂Ȃ��Ă��悢��������Ȃ��j
*/
int fMat_UnitMatrix(_stMatrix* This)
{
	int lErrorCounter = 0;

	int rowLoop,columnLoop;
	int rowLoopMax = This->row;
	int colLoopMax = This->column;

#ifdef def_InputCheck
	if((rowLoopMax == colLoopMax)
		)
#else
	if(true)
#endif
	
	{
		for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
		{
			for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
			{
				if(rowLoop == columnLoop)
					fMat(This, rowLoop, columnLoop) = 1;
				else
					fMat(This, rowLoop, columnLoop) = 0;
			}
		}
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}

//�̈�̓���ւ����s��˂΂Ȃ�Ȃ��̂Ŗʓ|�����B
/*
int fMat_Trans(_stMatrix* This)
{


	return 1;
}
*/
/*
void fMat_Add_rou(matrix c,matrix a, matrix b,int rowLoopMax,int colLoopMax)
{
	int rowLoop,columnLoop;

	for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
	{
		for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
		{
			c[rowLoop][columnLoop] = a[rowLoop][columnLoop] + b[rowLoop][columnLoop];
		}
	}
}
*/


/*!

@param    _stMatrix* A
@param    _stMatrix* B
@param    _stMatrix* C
@return  
@note     �s��̘a���o��
@note     Add C = A + B
@note     ab  ab = ab
*/
int fMat_Add(_stMatrix* C,_stMatrix* A,_stMatrix* B)
{
	int lErrorCounter = 0;

	int rowLoop,columnLoop;

	int rowLoopMax = A->row;
	int colLoopMax = A->column;

#ifdef  def_InputCheck
	if((A->row == B->row)
		&&(A->row== C->row)
		&&(A->column == B->column)
		&&(A->column== C->column))
#else
	if(true)
#endif
	{
		//fMat_Add_rou(C->Mat,A->Mat,B->Mat,rowLoopMax,colLoopMax);
		for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
		{
			for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
			{
				fMat(C, rowLoop, columnLoop) = fMat(A, rowLoop, columnLoop) + fMat(B, rowLoop, columnLoop);
			}
		}
	}
	else
	{
		lErrorCounter++;
	}


	return lErrorCounter;
}



/*!

@param    _stMatrix* A
@param    _stMatrix* B
@return  
@note     �s��̘a���o��
@note     Add A += B
*/
int fMat_Add2(_stMatrix* A,_stMatrix* B)
{
	int rowLoop,columnLoop;
	int lErrorCounter = 0;

	int rowLoopMax = A->row;
	int colLoopMax = A->column;

	
#ifdef  def_InputCheck
	if((A->row == B->row)
		&&(A->column == B->column))
#else
	if(true)
#endif		
	{
		for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
		{
			for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
			{
				fMat(A, rowLoop, columnLoop) += fMat(B, rowLoop, columnLoop);
			}
		}
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}



/*!

@param    _stMatrix* A
@param    _stMatrix* B
@param    _stMatrix* C
@return  
@note     �s��̍����o��
@note     Add C = A - B
@note     ab  ab = ab

*/
int fMat_Sub(_stMatrix* C,_stMatrix* A,_stMatrix* B)
{
	int rowLoop,columnLoop;
	int lErrorCounter = 0;

	int rowLoopMax = A->row;
	int colLoopMax = A->column;

#ifdef  def_InputCheck
	if((A->row == B->row)
		&&(A->row== C->row)
		&&(A->column == B->column)
		&&(A->column== C->column))
#else
	if(true)
#endif
	{
		for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
		{
			for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
			{
				fMat(C, rowLoop, columnLoop) = fMat(A, rowLoop, columnLoop) - fMat(B, rowLoop, columnLoop);
			}
		}
	}
	else
	{
		lErrorCounter++;
	}
	return lErrorCounter;
}




/*!

@param    _stMatrix* A
@param    _stMatrix* B
@return  
@note     �s��̍����o��
@note     Add A -= B
*/
int fMat_Sub2(_stMatrix* A,_stMatrix* B)
{
	int rowLoop,columnLoop;
	int lErrorCounter = 0;

	int rowLoopMax = A->row;
	int colLoopMax = A->column;

#ifdef  def_InputCheck
	if((A->row == B->row)
		&&(A->column == B->column))
#else
	if(true)
#endif
	{
		for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
		{
			for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
			{
				fMat(A, rowLoop, columnLoop) -= fMat(B, rowLoop, columnLoop);
			}
		}
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}

/*!
Check_Symmetric Matrix
@param    _stMatrix* A
@return   error counter
@note     �Ώ̍s�񂩂ǂ���check����B
@note     �Ώ̍s��łȂ��ꍇ�A�����͐����s��łȂ��ꍇ�ɂ̓G���[S
*/
int fMat_Check_SymmetricMatrix(_stMatrix* A)
{
	int rowLoop,columnLoop;
	int lErrorCounter = 0;

	int rowLoopMax = A->row;
	int colLoopMax = A->column;
/*
	double out[4][4];

	for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
	{
		for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
		{
			out[rowLoop][columnLoop] = A, rowLoop][columnLoop];
		}
	}
*/

#ifdef  def_InputCheck

	if((A->column == A->row))	
#else
	if(true)
#endif
	{
		for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
		{
			for(columnLoop = 0; columnLoop < rowLoop + 1; columnLoop++)
			{
				if(fMat(A, rowLoop, columnLoop) != fMat(A, columnLoop, rowLoop))
				{
					lErrorCounter++;
				}
			}
		}
	}
	else
	{
		lErrorCounter++;
	}


	return lErrorCounter;
}

/*!

@param    _stMatrix* A
@param    _stMatrix* B
@param    _stMatrix* C
@return  
@note     �s��̊|���Z
@note     Mlt C = AB
@note     am * mb = ab
*/
int fMat_Mlt(_stMatrix* C,_stMatrix* A,_stMatrix* B)
{
	int rowLoop,columnLoop,multiLoop;
	int lErrorCounter = 0;

	int rowLoopMax = A->row;
	int colLoopMax = B->column;
	int mltLoopMax = A->column;

#ifdef  def_InputCheck

	if((A->column == B->row)
		&&(C->row == A->row)
		&&(C->column == B->column))	
#else
	if(true)
#endif
	{
		for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
		{
			for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
			{
				fMat(C, rowLoop, columnLoop) = 0;

				for(multiLoop = 0; multiLoop < mltLoopMax; multiLoop++)
				{
					fMat(C, rowLoop, columnLoop) += fMat(A, rowLoop, multiLoop) * fMat(B, multiLoop, columnLoop);
				}
			}
		}
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}


/*!

@param    _stMatrix* A
@param    _stMatrix* B
@param    _stMatrix* C
@return  
@note     �]�u�s��̊|���Z
@note     Mlt C = ABt
@note     am * bm = ab
*/
int fMat_MltTrans(_stMatrix* C, _stMatrix* A, _stMatrix* B)
{
	int rowLoop,columnLoop,multiLoop;
	int lErrorCounter = 0;

	int rowLoopMax = A->row;//a
	int colLoopMax = B->row;//b
	int mltLoopMax = A->column;

#ifdef  def_InputCheck
	if((A->column == B->column)
		&&(C->row == A->row)
		&&(C->column == B->row))
#else
	if(true)
#endif
	{
		for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
		{
			for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
			{
				//C, rowLoop][columnLoop] = 0;
				fMat(C, rowLoop, columnLoop) = 0;

				for(multiLoop = 0; multiLoop < mltLoopMax; multiLoop++)
				{
					//C, rowLoop][columnLoop] += A, rowLoop][multiLoop] * B, columnLoop][multiLoop];
					fMat(C, rowLoop, columnLoop) += fMat(A, rowLoop, multiLoop) * fMat(B, columnLoop, multiLoop);
				}
			}
		}
	}
	else
	{
		lErrorCounter++;
	}
	

	return lErrorCounter;
}

/*!

@param    _stMatrix* A
@param    _stMatrix* B
@param    _stMatrix* C
@return  
@note     �]�u�s��̊|���Z
@note     Mlt C = AtB
@note     ma * mb = ab
@note     �vcheck
*/
int fMat_TransMlt(_stMatrix* C, _stMatrix* A, _stMatrix* B)
{
	int rowLoop,columnLoop,multiLoop;
	int lErrorCounter = 0;

	int rowLoopMax = A->column;//a
	int colLoopMax = B->column;//b
	int mltLoopMax = A->row;

#ifdef  def_InputCheck
	if((A->row == B->row)
		&&(C->row == A->column)
		&&(C->column == B->column))
#else
	if(true)
#endif
	{
		for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
		{
			for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
			{
				//C, rowLoop][columnLoop] = 0;
				fMat(C, rowLoop, columnLoop) = 0;

				for(multiLoop = 0; multiLoop < mltLoopMax; multiLoop++)
				{
					//C, rowLoop][columnLoop] += A, multiLoop][rowLoop] * B, multiLoop][columnLoop];
					fMat(C, rowLoop, columnLoop) += fMat(A, multiLoop, rowLoop) * fMat(B, multiLoop, columnLoop);
				}
			}
		}
	}
	else
	{
		lErrorCounter++;
	}
	

	return lErrorCounter;
}

/*!

@param     _stMatrix* U
@param     _stMatrix* D
@param     _stMatrix* input
@return  
@note      UD���� input �� UD
@note      ���p�J���}���t�B���^ p171
@note      �Ίp�s��쐬��
*/
int fMat_UD_Degradation(_stMatrix* U, _stMatrix* D, _stMatrix* input)
{
	//D�͂���Ȃ�����
	//2version�쐬����H

	int lErrorCounter = 0;

	int row_input = input->row;
	int col_input = input->column;


	int i_loop = 0;
	int j_loop = 0;
	int k_loop = 0;

#ifdef  def_InputCheck	
	if((row_input == col_input)
		&&(row_input == U->row)
		&&(row_input == D->row)
		&&(col_input == U->column)
		&&(col_input == D->column)
		)
#else
	if(true)
#endif
	{
		//UD����
		for(i_loop = row_input - 1; i_loop >= 0; i_loop--)
		{
			fMat(D, i_loop, i_loop) = fMat(input, i_loop, i_loop);

			fMat(U, i_loop, i_loop) = 1;

			for(j_loop = 0; j_loop < i_loop; j_loop++)
			{
				//U, j_loop][i_loop] = input, j_loop][i_loop] / D, i_loop][i_loop];
				fMat(U, j_loop, i_loop) = fMat(input, j_loop, i_loop) / fMat(D, i_loop, i_loop);

				for(k_loop = 0; k_loop <= j_loop; k_loop++)
				{
					//input, k_loop][j_loop] -= U, k_loop][i_loop] * D, i_loop][i_loop] * U, j_loop][i_loop];
					fMat(input, k_loop, j_loop) -= fMat(U, k_loop, i_loop) *fMat(D, i_loop, i_loop) * fMat(U, j_loop, i_loop);
				}
			}
		}

		//if(k == 0)
		//U, 0][0] = 1;
		//D, 0][0] = input, 0][0];
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}

/*!

@param    _stMatrix* Output   :  A^-1
@param    _stMatrix* Input    :  A
@return   error��
@note     �|���o���@�ɂ��t�s����v�Z����B
@note     �����s��ł���Ƃ���������t��������ׂ�����
*/
int fMat_InverseMatrix_Gauss(_stMatrix* Output, _stMatrix* Input)
{
	int lErrorCounter = 0;
	int row_input = Input->row;
	int col_input = Input->column;

	int i_loop = 0;
	int j_loop = 0;
	int k_loop = 0;

	double lCalBuf = 0;

#ifdef  def_InputCheck	
	if((row_input == col_input)
		&&(row_input == Output->row)
		&&(row_input == Output->column)
		)
#else
	if(true)
#endif
	{
		//�P�ʍs��̍쐬
		fMat_UnitMatrix(Output);

		//
		for(i_loop = 0;i_loop < col_input;i_loop++)
		{
			//lCalBuf = 1 / Input, i_loop][i_loop];
			lCalBuf = 1 / fMat(Input, i_loop, i_loop);

			for(j_loop = 0;j_loop < col_input; j_loop++)
			{
				//Input, i_loop][j_loop] *= lCalBuf;
				fMat(Input, i_loop, j_loop) *= lCalBuf;

				//Output, i_loop][j_loop] *= lCalBuf;
				fMat(Output, i_loop, j_loop) *= lCalBuf;
			}

			for(j_loop = 0;j_loop < col_input;j_loop++)
			{
				if(i_loop != j_loop)
				{
					//lCalBuf = Input, j_loop][i_loop];
					lCalBuf = fMat(Input, j_loop, i_loop);

					for(k_loop = 0;k_loop < col_input;k_loop++)
					{
						//Input, j_loop][k_loop] -= Input, i_loop][k_loop] * lCalBuf;
						fMat(Input, j_loop, k_loop) -= fMat(Input, i_loop, k_loop) * lCalBuf;

						//Output, j_loop][k_loop] -= Output, i_loop][k_loop] * lCalBuf;
						fMat(Output, j_loop, k_loop) -= fMat(Output, i_loop, k_loop) * lCalBuf;
					}
				}
			}
		}

	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}

/*!

@param    _stMatrix* Output  : A^-1
@param    _stMatrix* Input   : A
@return   error��
@note     input�̃R���X�L�[������output�ɏo�͂���
@note	  �֐����œ��͍s��input���ω�����̂Œ���
@note	  �֐����ŕ�������p����iC�̏ꍇ��Math.h�AC++/CLI��namespace System���K�v
*/
int fMat_CholeskyDecomposition(_stMatrix* Output, _stMatrix* Input)
{
	int lErrorCounter = 0;
	int row_input = Input->row;
	int col_input = Input->column;

	int i_loop = 0;
	int j_loop = 0;
	int k_loop = 0;

	double lCalBuf = 0;

#ifdef  def_InputCheck	
	if((row_input == col_input)
		&&(row_input == Output->row)
		&&(row_input == Output->column)
		)
#else
	if(true)
#endif
	{
		fMat_Zero(Output);

		for(k_loop = (col_input - 1);k_loop > 0;k_loop--)
		{
			//(8.13���j
			//Output, k_loop][k_loop] = fCal_Sqrt(Input, k_loop][k_loop]);
			fMat(Output, k_loop, k_loop) = fCal_Sqrt(fMat(Input, k_loop, k_loop));

			//j_loop�̎g�����ɒ���(k_loop�Ŋ��ɂƂ肤��l�����߂Ă���
			for(j_loop = 0;j_loop < (k_loop - 0); j_loop++)
			{
				//(8.14���j
				//Output, j_loop][k_loop] = Input, j_loop][k_loop] / Output, k_loop][k_loop];
				fMat(Output, j_loop, k_loop) = fMat(Input, j_loop, k_loop) / fMat(Output, k_loop, k_loop);

				//i_loop�̎g�����ɒ���
				for(i_loop = 0;i_loop <= j_loop;i_loop++)
				{
					//(8.15���j
					//Input, i_loop][j_loop] = Input, i_loop][j_loop] - Output, i_loop][k_loop] * Output, j_loop][k_loop];
					fMat(Input, i_loop, j_loop) = fMat(Input, i_loop, j_loop) - fMat(Output, i_loop, k_loop) * fMat(Output, j_loop, k_loop);
				}
			}
		}
		//(8.16���j
		fMat(Output, 0, 0) = fCal_Sqrt(fMat(Input, 0, 0));
		//Output, 0][0] = sqrt(Input, 0][0]);

	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}



/*!

@param    def_ElementType_mat input
@param    int input_row
@param    int input_col
@param    _stMatrix* This
@return  
@note     �w�肵���s��ɒl���i�[����B
*/
int fMat_Set(_stMatrix* This, int input_row, int input_col, def_ElementType_mat input)
{
	int lErrorCounter = 0;

#ifdef  def_InputCheck	
	if((input_row < This->row)
	 &&(input_col < This->column)
		)
#else
	if(true)
#endif
	{
		fMat(This, input_row, input_col) = input;
	}
	else
	{
		lErrorCounter++;
	}

	return lErrorCounter;
}

/*!

@param    int input_row
@param    int input_col
@param    _stMatrix* This
@return  
@note     �w�肵���s��̗v�f���擾����
*/
def_ElementType_mat fMat_Get(_stMatrix* This, int input_row, int input_col)
{
	def_ElementType_mat ret = 0;

#ifdef  def_InputCheck	
	if((input_row < This->row)
	 &&(input_col < This->column)
		)
#else
	if(true)
#endif
	{
		ret = fMat(This, input_row, input_col);
	}
	else
	{
		ret = -1;
	}

	return ret;
}

/*!

@param    int input_row
@param    _stMatrix* This
@return  
@note     �w�肵���s��̑Ίp�v�f���擾����
*/
def_ElementType_mat fMat_GetDiag(_stMatrix* This, int input_row)
{
	def_ElementType_mat ret = 0;

#ifdef  def_InputCheck	
	if((input_row < This->row)
	 &&(input_row < This->column)
		)
#else
	if(true)
#endif
	{
		ret = fMat(This, input_row, input_row);
	}
	else
	{
		ret = -1;
	}


	return ret;
}
/*!
@param     _stMatrix* output
@param     _stMatrix* input
@return  
@note      �]�u�s��
@note      A = Bt
@note      �vcheck
*/
int fMat_Transpose(_stMatrix* output, _stMatrix* input)
{
	int rowLoop = 0;
	int columnLoop = 0;
	int lErrorCounter = 0;

	int row = output->row;
	int col = output->column;

	//_stMatrix* trans = fMat_New(col,row);//fMat_New(row,col);

#ifdef  def_InputCheck
	if((output->column == input->row)
		&&(output->column == input->row))	
#else
	if(true)
#endif
	{
		for(rowLoop = 0; rowLoop < row; rowLoop++)
		{
			for(columnLoop = 0; columnLoop < col; columnLoop++)
			{
				//fMat_Set(This, rowLoop][columnLoop],columnLoop,rowLoop,trans);
				fMat(output, rowLoop, columnLoop) = fMat(input, columnLoop, rowLoop);
			}
		}
	}
	else
	{
		lErrorCounter++;
	}
	
	return lErrorCounter;
}


/*!
@param     _stMatrix* input
@param     int* p
@param     int* q
@return  
@note      �Ώ̍s���Ίp�v�f�̍ő�v�f�i��Βl�j��T��
@note      jacobi�@�Ɏg�p����
@note      (p,q)�ɗv�f�ԍ����i�[
@note      �Ώ̍s��ł���΍H�v�����ق���������
*/
int fMat_GetMaxElement_NotDiag(_stMatrix* input, int* p, int* q, def_ElementType_mat* max)
{
	int rowLoop = 0;
	int columnLoop = 0;
	int lErrorCounter = 0;

	int row = input->row;
	int col = input->column;

	int lp,lq;
	def_ElementType_mat lMax = 0;


#ifdef  def_InputCheck
	if((row == col)
	   //(row > 1)
		)	
#else
	if(true)
#endif
	{
		//���O�p�s��̃��[�v
		for(rowLoop = 1; rowLoop < row; rowLoop++)
		{
			for(columnLoop = 0; columnLoop < rowLoop; columnLoop++)
			{
				//
				if(fCal_Abs(fMat(input, rowLoop, columnLoop)) > lMax)
				{
					lMax = fCal_Abs(fMat(input, rowLoop, columnLoop));
					lp = rowLoop;
					lq = columnLoop;
				}
			}
		}

		*max = lMax;
		*p = lp;
		*q = lq;
	}
	else
	{
		lErrorCounter++;

		*max = lMax;
		*p = 0;
		*q = 0;
	}

	
	
	return lErrorCounter;
}


/*!
@param     _stMatrix* A
@param     _stMatrix* B
@return  
@note     ���όv�Z
@note     (n,1)T (n,1)�s��̓���
*/
def_ElementType_mat fMat_InnerProduct(_stMatrix* A, _stMatrix* B)
{
	def_ElementType_mat ret = (def_ElementType_mat)0.0;

	int rowLoop = 0;
	//int columnLoop = 0;
	int lErrorCounter = 0;

	int row = A->row;
	//int col = A->column;

	//_stMatrix* trans = fMat_New(col,row);//fMat_New(row,col);

#ifdef  def_InputCheck
	if((A->row == B->row)
     &&(A->column == 1)
	 &&(B->column == 1)
	 )
#else
	if(true)
#endif
	{
		for(rowLoop = 0; rowLoop < row; rowLoop++)
		{
			ret += fMat(A, rowLoop, 0) * fMat(B, rowLoop, 0);
		}
	}
	else
	{
		lErrorCounter++;
	}

	return ret;
}

/*!
@param     _stMatrix* C
@param     _stMatrix* A
@param     _stMatrix* B
@return  
@note     ���ύs��̌v�Z
*/
int fMat_InnerMatrix(_stMatrix* C, _stMatrix* A, _stMatrix* B)
{
	int lErrorCounter = 0;

	lErrorCounter++;

	return lErrorCounter;
}

///*------------------------------------------------------------------*///
///���ӁF�֐����ŗ̈�m�ہA�J�����s���Ă���
///*------------------------------------------------------------------*///

/*!

@param    _stMatrix* A
@param    _stMatrix* B
@return  
@note     �s��̊|���Z���o��
@note     MLT2 A = AB (am mb = ab)
@note     �vcheck
*/
int fMat_Mlt2(_stMatrix* A, _stMatrix* B)
{
	int rowLoop = 0;
	int columnLoop = 0;
	int multiLoop = 0;
	int lErrorCounter = 0;

	int row = A->row;
	int col = B->column;
	int mltLoopMax = A->column;

	//_stMatrix* trans = fMat_New(col,row);//fMat_New(row,col);

#ifdef  def_InputCheck
	if((A->column == B->row)
		//&&(B->column == B->row)
		//&&(A->column == B->column)
		)	
#else
	if(true)
#endif
	{
		//�̈�m��
		_stMatrix* Mataddress = fMat_New(row, col);
		matrix lTemp_Mat;
	
		for(rowLoop = 0; rowLoop < row; rowLoop++)
		{
			for(columnLoop = 0; columnLoop < col; columnLoop++)
			{
				fMat(Mataddress, rowLoop, columnLoop) = 0;
	
				for(multiLoop = 0; multiLoop < mltLoopMax; multiLoop++)
				{
					fMat(Mataddress, rowLoop, columnLoop) += fMat(A, rowLoop, multiLoop) * fMat(B, multiLoop, columnLoop);
				}
			}
		}
		//���u��
		lTemp_Mat = A->Mat;

		//��������
		A->Mat = Mataddress->Mat;
		A->row = row;
		A->column = col;

		//����ւ�
		Mataddress->Mat = lTemp_Mat;

		//�J��
		fMat_Delete(Mataddress);

		
	}
	else
	{
		lErrorCounter++;
	}
	
	return lErrorCounter;
}

/*!

@param    _stMatrix* A
@param    _stMatrix* B
@return   
@note     �vcheck �o�O����?
@note     A = A * Bt (am bm = ab)
*/
int fMat_MltTrans2(_stMatrix* A, _stMatrix* B)
{
	int rowLoop = 0;
	int columnLoop = 0;
	int multiLoop = 0;
	int lErrorCounter = 0;

	int row = A->row;
	int col = B->row;
	int mltLoopMax = A->column;

	//_stMatrix* trans = fMat_New(col,row);//fMat_New(row,col);

#ifdef  def_InputCheck
	if((A->column == B->column)
		//&&(B->column == B->row)
		//&&(A->column == B->column)
		)	
#else
	if(true)
#endif
	{
		//�̈�m��
		_stMatrix* Mataddress = fMat_New(row, col);
		matrix lTemp_Mat;
	
		for(rowLoop = 0; rowLoop < row; rowLoop++)
		{
			for(columnLoop = 0; columnLoop < col; columnLoop++)
			{
				fMat(Mataddress, rowLoop, columnLoop) = 0;
	
				for(multiLoop = 0; multiLoop < mltLoopMax; multiLoop++)
				{
					fMat(Mataddress, rowLoop, columnLoop) += fMat(A, rowLoop, multiLoop) * fMat(B, columnLoop, multiLoop);
				}
			}
		}

		//���u��
		lTemp_Mat = A->Mat;

		//��������
		A->Mat = Mataddress->Mat;
		A->row = row;
		A->column = col;

		//����ւ�
		Mataddress->Mat = lTemp_Mat;

		//�J��
		fMat_Delete(Mataddress);
	}
	else
	{
		lErrorCounter++;
	}
	
	return lErrorCounter;
}




/*!

@param     _stMatrix* This
@param    
@return  
@note      �]�u�s��
@note      �V���ȗ̈���쐬���A���̃|�C���^����J���A���̌�|�C���^�����ւ���
@note      A = At
@note      �vcheck 
*/
int fMat_Transpose2(_stMatrix* This)
{
	int rowLoop = 0;
	int columnLoop = 0;
	int lErrorCounter = 0;

	int row = This->column;
	int col = This->row;

	//
	_stMatrix* Mataddress = fMat_New(row, col);
	matrix lTemp_Mat;
	
	for(rowLoop = 0; rowLoop < row; rowLoop++)
	{
		for(columnLoop = 0; columnLoop < col; columnLoop++)
		{
			//fMat_Set(This, rowLoop][columnLoop],columnLoop,rowLoop,trans);
			fMat(Mataddress, rowLoop, columnLoop) = fMat(This, columnLoop, rowLoop);
		}
	}

	//���u��
	lTemp_Mat = This->Mat;

	//�u������
	This->Mat = Mataddress->Mat;
	This->row = row;
	This->column = col;

	//����ւ�
	Mataddress->Mat = lTemp_Mat;

	//�J��
	fMat_Delete(Mataddress);
	
	return lErrorCounter;
}

/*!

@param    _stMatrix* Input   :  A^-1
@return   error��
@note     �K�E�X�̏����@�ɂ��t�s����v�Z����B
@note     ���I�m�ۂ��s��
@note     A = A^-1
@note     �����s��ł���Ƃ���������t��������ׂ�����
*/
int fMat_InverseMatrix_Gauss2(_stMatrix* Input)
{
	int lErrorCounter = 0;
	int row_input = Input->row;
	int col_input = Input->column;

	int i_loop = 0;
	int j_loop = 0;
	int k_loop = 0;

	double lCalBuf = 0;

	

#ifdef  def_InputCheck	
	if((row_input == col_input)
		//&&(row_input == Output->row)
		//&&(row_input == Output->column)
		)
#else
	if(true)
#endif
	{
		//�m��
		_stMatrix* Mataddress = fMat_New(row_input, row_input);
		matrix lTemp_Mat;
		

		//�P�ʍs��̍쐬
		for(i_loop = 0;i_loop < row_input; i_loop++)
		{
			for(j_loop = 0;j_loop < row_input; j_loop++)
			{
				if(i_loop == j_loop)
				{
					fMat(Mataddress, i_loop, j_loop) = 1;
				}
				else
				{
					fMat(Mataddress, i_loop, j_loop) = 0;
				}
			}
		}
		//
		for(i_loop = 0;i_loop < col_input;i_loop++)
		{
			lCalBuf = 1 / fMat(Input, i_loop, i_loop);

			for(j_loop = 0;j_loop < col_input; j_loop++)
			{
				fMat(Input, i_loop, j_loop) *= lCalBuf;

				fMat(Mataddress, i_loop, j_loop) *= lCalBuf;
			}

			for(j_loop = 0;j_loop < col_input;j_loop++)
			{
				if(i_loop != j_loop)
				{
					lCalBuf = fMat(Input, j_loop, i_loop);

					for(k_loop = 0;k_loop < col_input;k_loop++)
					{
						fMat(Input, j_loop, k_loop) -= fMat(Input, i_loop, k_loop) * lCalBuf;

						fMat(Mataddress, j_loop, k_loop) -= fMat(Mataddress, i_loop, k_loop) * lCalBuf;
					}
				}
			}
		}
		//���u��
		lTemp_Mat = Input->Mat;

		//�|�C���^������������
		Input->Mat = Mataddress->Mat;

		//����ւ�
		Mataddress->Mat = lTemp_Mat;

		//�J��
		fMat_Delete(Mataddress);
	}
	else
	{
		lErrorCounter++;
	}

	//fMat_Delete(Mataddress);

	return lErrorCounter;
}

/*!

@param    _stMatrix* A
@param    _stMatrix* B
@return  
@note     �]�u�s��̊|���Z
@note     Mlt A = AtB
@note     ma * mb = ab
@note     �vcheck
*/
int fMat_TransMlt2(_stMatrix* A, _stMatrix* B)
{
	int rowLoop,columnLoop,multiLoop;
	int lErrorCounter = 0;

	int rowLoopMax = A->column;//a
	int colLoopMax = B->column;//b
	int mltLoopMax = A->row;

#ifdef  def_InputCheck
	if((A->row == B->row)
		//&&(C->row == A->column)
		//&&(C->column == B->column)
		)
#else
	if(true)
#endif
	{
		//�̈�m��
		_stMatrix* Mataddress = fMat_New(rowLoopMax, colLoopMax);
		matrix lTemp_Mat;

		for(rowLoop = 0; rowLoop < rowLoopMax; rowLoop++)
		{
			for(columnLoop = 0; columnLoop < colLoopMax; columnLoop++)
			{
				fMat(Mataddress, rowLoop, columnLoop) = 0;

				for(multiLoop = 0; multiLoop < mltLoopMax; multiLoop++)
				{
					fMat(Mataddress, rowLoop, columnLoop) += fMat(A, multiLoop, rowLoop) * fMat(B, multiLoop, columnLoop);
				}
			}
		}

		//���u��
		lTemp_Mat = A->Mat;

		//��������
		A->Mat = Mataddress->Mat;
		A->row = rowLoopMax;
		A->column = colLoopMax;

		//����ւ�
		Mataddress->Mat = lTemp_Mat;

		//�J��
		fMat_Delete(Mataddress);
	}
	else
	{
		lErrorCounter++;
	}
	return lErrorCounter;
}


/*!

@param    _stMatrix* A
@param    _stMatrix* B
@param    _stMatrix* C
@return  
@note     �s��̊|���Z
@note     Mlt A = ABAt
@note     ab * bb * ba = aa
@note     �쐬�\��
*/