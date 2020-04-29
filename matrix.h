#pragma once

#include <math.h>

//#define def_ElementType_mat float
#define def_ElementType_mat double

//
//
//

//#define def_ShortMemory
//



#ifdef def_ShortMemory

#define matrix def_ElementType_mat*
typedef struct stMatrix 
{
    int row;
    int column;
    matrix Mat;
    
}_stMatrix;

#define fMat(a, b, c) (a->Mat[a->row*b + c])

//inline def_ElementType_mat fMat(Mat r, c) {return }

#else

#define vector def_ElementType_mat*
#define matrix vector*

typedef struct stMatrix 
{
    int row;
    int column;
    matrix Mat;

}_stMatrix;


#define fMat(a, b, c) (a->Mat[b][c])

#endif


#define fCal_Abs(a) (abs(a))
#define fCal_Sqrt(a) (sqrt(a))
//#define fCal_Abs(a, b, c) (abs(fMat(a, b, c)))
//#define fCal_Sqrt(a, b, c) (sqrt(fMat(a, b, c)))
//#define fCal_Sqrt(a, b, c) (sqrt(fMat(a, b, c)))



extern _stMatrix *fMat_New(int input_row, int input_col);
extern int fMat_Delete(_stMatrix* Mat);
extern _stMatrix* fMat_NewCopy(_stMatrix* This);
extern _stMatrix* fMat_NewCopy_rowVector(_stMatrix* This, int input_row);
extern _stMatrix* fMat_NewCopy_colVector(_stMatrix* This, int input_col);
extern matrix new_matrix(int nrow, int ncol);
extern void free_matrix(matrix a);
extern int fMat_Transpose2(_stMatrix* This);
extern int fMat_Mlt2(_stMatrix* A, _stMatrix* B);
extern int fMat_MltTrans2(_stMatrix* A, _stMatrix* B);
extern int fMat_InverseMatrix_Gauss2(_stMatrix* Input);
extern int fMat_TransMlt2(_stMatrix* A, _stMatrix* B);

//property
extern int fMat_Set(_stMatrix* This, int input_row, int input_col, def_ElementType_mat input);
extern def_ElementType_mat fMat_Get(_stMatrix* This, int input_row, int input_col);
extern def_ElementType_mat fMat_GetDiag(_stMatrix* This, int input_row);

//cal
extern int fMat_SetConst(_stMatrix* This, def_ElementType_mat input);
extern int fMat_Zero(_stMatrix* This);
extern int fMat_UnitMatrix(_stMatrix* This);
extern int fMat_Add(_stMatrix* C,_stMatrix* A,_stMatrix* B);
extern int fMat_Add2(_stMatrix* A,_stMatrix* B);
extern int fMat_Mlt(_stMatrix* C,_stMatrix* A,_stMatrix* B);
extern int fMat_MltTrans(_stMatrix* C,_stMatrix* A,_stMatrix* B);
extern int fMat_TransMlt(_stMatrix* C, _stMatrix* A, _stMatrix* B);
extern int fMat_Sub(_stMatrix* C,_stMatrix* A,_stMatrix* B);
extern int fMat_Sub2(_stMatrix* A,_stMatrix* B);
extern int fMat_UD_Degradation(_stMatrix* U, _stMatrix* D, _stMatrix* input);
extern int fMat_Multiplier(_stMatrix* output, _stMatrix* input, def_ElementType_mat Multiplier);
extern int fMat_Multiplier2(_stMatrix* This, def_ElementType_mat Multiplier);
extern int fMat_Multiplier_rowVector(_stMatrix* output, int output_row, _stMatrix* input, int input_row, def_ElementType_mat Multiplier);
extern int fMat_Multiplier_colVector(_stMatrix* output, int output_col, _stMatrix* input, int input_col, def_ElementType_mat Multiplier);
extern int fMat_Multiplier2_rowVector(_stMatrix* This, int output_row, def_ElementType_mat Multiplier);
extern int fMat_Multiplier2_colVector(_stMatrix* This, int output_col, def_ElementType_mat Multiplier);
extern int fMat_Copy(_stMatrix* output, _stMatrix* input);
extern int fMat_Copy_rowVector(_stMatrix* output, int output_row, _stMatrix* input, int input_row);
extern int fMat_Copy_colVector(_stMatrix* output, int output_col, _stMatrix* input, int input_col);
extern int fMat_Copy_colVector_TO_row(_stMatrix* output, int output_row, _stMatrix* input, int input_col);
extern int fMat_Copy_rowVector_TO_column(_stMatrix* output, int output_col, _stMatrix* input, int input_row);
extern int fMat_Add2_rowVector(_stMatrix* output, int output_row, _stMatrix* input, int input_row);
extern int fMat_Add2_colVector(_stMatrix* output, int output_col, _stMatrix* input, int input_col);
extern int fMat_InverseMatrix_Gauss(_stMatrix* Output, _stMatrix* Input);
extern int fMat_CholeskyDecomposition(_stMatrix* Output, _stMatrix* Input);
extern int fMat_Transpose(_stMatrix* output, _stMatrix* input);
extern int fMat_Check_SymmetricMatrix(_stMatrix* A);
extern def_ElementType_mat fMat_InnerProduct(_stMatrix* A, _stMatrix* B);
extern int fMat_InnerMatrix(_stMatrix* C, _stMatrix* A, _stMatrix* B);
extern int fMat_GetMaxElement_NotDiag(_stMatrix* input, int* p, int* q, def_ElementType_mat* max);
