#pragma once

#include <stdio.h>

class Matrix
{
    private:
        double** matrix;
        unsigned int num_cols;
        unsigned int num_rows;

    public:

        // matrix nxm
        Matrix (unsigned int _num_rows, unsigned int _num_cols, double _matrix[])
            : num_cols{_num_cols}, num_rows{_num_rows}
        {
            matrix = new double*[num_rows];
            for(unsigned short i=0; i<num_rows; i++){
                matrix[i] = new double[num_cols];
                for(unsigned short j=0; j<num_cols; j++)
                {
                    matrix[i][j] = _matrix[j + num_cols*i];
                }
            }
        }

        Matrix (unsigned int order)
            : num_cols{order}, num_rows{order}
        {
            matrix = new double*[num_rows];
            for(unsigned short i=0; i<order; i++){
                matrix[i] = new double[num_cols];
                for(unsigned short j=0; j<order; j++)
                {
                    if(i == j){
                        matrix[i][j] = 1.0;
                        continue;
                    }
                    matrix[i][j] = 0.0;
                }
            }
        }

        void print ()
        {
            for(unsigned short i=0; i<num_rows; i++)
            {
                for(unsigned short j=0; j<num_cols; j++)
                {
                    printf("%.3f ", matrix[i][j]);
                }
                printf("\n");
            }
        }

        void transpose ()
        {
            unsigned short temp_col = this->num_cols;
            this->num_cols = this->num_rows;
            this->num_rows = temp_col;

            double** temp_matrix = new double*[num_rows];
            for(unsigned short i=0; i<num_rows; i++)
            {
                temp_matrix[i] = new double[num_cols];
                for(unsigned short j=0; j<num_cols; j++)
                {
                    temp_matrix[i][j] = this->matrix[j][i];
                }
            }
            this->matrix = temp_matrix;
        }

        double** getMatrix () const
        {
            return this->matrix;
        }

        unsigned int cols () const
        {
            return this->num_cols;
        }

        unsigned int rows () const
        {
            return this->num_rows;
        }

        Matrix operator*(const Matrix& m)
        {
            if(this->num_cols != m.rows()){
                printf("Not possible to multiply!");
                return Matrix(0);
            }

            double** m_matrix = m.getMatrix();
            unsigned int _num_rows = this->num_rows;
            unsigned int _num_cols = m.cols();

            double temp_m[_num_cols * _num_rows];
            double temp_res = 0.0;
            unsigned short index = 0;

            for(unsigned short i=0; i<this->num_rows; i++)
            {
                for(unsigned short j=0; j<m.cols(); j++)
                {
                    for(unsigned short k=0; k<this->num_cols; k++)
                    {
                        temp_res += this->matrix[i][k] * m_matrix[k][j];
                    }
                    temp_m[index] = temp_res;
                    index++;
                    temp_res = 0;
                }
                
            }

            return Matrix(_num_rows, _num_cols, temp_m);
        }

        Matrix operator*(double val)
        {
            unsigned int _num_rows = this->num_rows;
            unsigned int _num_cols = this->num_cols;
            double temp_m[_num_cols * _num_rows];
            unsigned short index = 0;

            for(unsigned short i=0; i<this->num_rows; i++)
            {
                for(unsigned short j=0; j<this->num_cols; j++)
                {
                    temp_m[index] = this->matrix[i][j] * val;
                    index++;
                }
            }

            return Matrix(_num_rows, _num_cols, temp_m);
        }


};