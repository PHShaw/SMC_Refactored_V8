/*
 * matrixManipulation.h
 *
 *  Created on: 11 May 2012
 *      Author: icub
 */


#ifndef MATRIX_MANIPULATION_PHS
#define MATRIX_MANIPULATION_PHS

#include <cstdio>


struct Matrix_smc{

	Matrix_smc()
	{
		rows = 4;
		columns = 4;
		mat[rows][columns];
		for(int i=0; i<rows; i++)
		{
			for(int j=0; j<columns; j++)
			{
				mat[i][j]=0;
			}
		}
	}

	Matrix_smc(int pRows, int pColumns)
	{
		rows = pRows;
		columns = pColumns;
		mat[pRows][pColumns];
		for(int i=0; i<rows; i++)
		{
			for(int j=0; j<columns; j++)
			{
				mat[i][j]=0;
			}
		}
	}


	void set(int i, int j, float f)
	{
		mat[i][j] = f;
	}

	float get(int i, int j)
	{
		return mat[i][j];
	}

	void transpose()
	{
		float matPrime[columns][rows];
		for(int i=0; i<rows; i++)
		{
			for(int j=0; j<columns; j++)
			{
				matPrime[j][i] = mat[i][j];
			}
		}

		mat[columns][rows];
		int cols = columns;
		columns = rows;
		rows = cols;
		for(int i=0; i<rows; i++)
		{
			for(int j=0; j<columns; j++)
			{
				mat[i][j] = matPrime[i][j];
			}
		}
	}

	/**
	 * Designed to deal with square matrices, ideally.
	 */
	void multiply(Matrix_smc m2)
	{
		if(m2.rows!= columns && m2.columns!=rows)
		{
			printf("Invalid size: [%i,%i]\n",m2.rows,m2.columns);
			return;
		}
		float matPrime[rows][m2.columns];
		float sum;
		for(int i=0; i<rows; i++)
		{
			for(int j=0; j<m2.columns; j++)
			{
				sum=0;
				for(int k=0; k<m2.rows; k++)
				{
					sum += mat[i][k] * m2.get(k,j);
				}
				matPrime[i][j] = sum;
			}
		}

		columns = m2.columns;
		mat[rows][columns];

		for(int i=0; i<rows; i++)
		{
			for(int j=0; j<columns; j++)
			{
				mat[i][j] = matPrime[i][j];
			}
		}
	}

	void add(Matrix_smc m2)
	{
		if(m2.rows!=rows && m2.columns!=columns)
		{
			printf("Matrices are different sizes\n");
			return;
		}
		for(int i=0; i<rows; i++)
		{
			for(int j=0; j<columns; j++)
			{
				mat[i][j] += m2.get(i,j);
			}
		}
	}

	void subtract(Matrix_smc m2)
	{
		if(m2.rows!=rows && m2.columns!=columns)
		{
			printf("Matrices are different sizes\n");
			return;
		}
		for(int i=0; i<rows; i++)
		{
			for(int j=0; j<columns; j++)
			{
				mat[i][j] -= m2.get(i,j);
			}
		}
	}


	void print()
	{
		for(int i=0; i<rows; i++)
		{
			for(int j=0; j<columns; j++)
			{
				printf("[%.2f]",mat[i][j]);
			}
			printf("\n");
		}
	}

	int rows;
	int columns;
	float mat[4][4];
};




#endif
