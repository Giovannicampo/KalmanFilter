#pragma once

#include <stdio.h>
#include <math.h>

class Matrix
{
    private:
        double** matrix;
        unsigned int num_cols;
        unsigned int num_rows;

    public:

        //--------------------------------------------------------------------------------
        // ATTENZIONE -> determinante implementato solo per matrici con ordine minore <= 3
        //--------------------------------------------------------------------------------
        void determinante (double* det)
        {
            if(this->num_cols != this->num_rows){
                printf("la matrice non è quadrata, non è possibile calcolarne il determinante\n");
                return;
            }

            if(num_cols > 3){
                printf("not implemented yet! sorry\n");
            }

            if(num_cols == 1){
                *det = matrix[0][0];
                return;
            }

            if(num_cols == 2){
                *det = (matrix[0][0] * matrix[1][1]) -
                       (matrix[1][0] * matrix[0][1]);
                return;
            }
if(num_cols > 3){
                printf("not implemented yet! sorry\n");
            }
            *det = (matrix[0][0] * matrix[1][1] * matrix[2][2]) +
                   (matrix[0][1] * matrix[1][2] * matrix[2][0]) +
                   (matrix[0][2] * matrix[1][0] * matrix[2][1]) -
                   (matrix[2][0] * matrix[1][1] * matrix[0][2]) -
                   (matrix[2][1] * matrix[1][2] * matrix[0][0]) -
                   (matrix[2][2] * matrix[1][0] * matrix[0][1]);
        }

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

        Matrix(const Matrix& m)
            : num_cols{m.cols()}, num_rows{m.rows()}, matrix{m.getMatrix()}
        {}

        void print ()
        {
            for(unsigned short i=0; i<num_rows; i++)
            {
                for(unsigned short j=0; j<num_cols; j++)
                {
                    printf("%.5f ", matrix[i][j]);
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

        double cofattore3x3(unsigned int ix, unsigned int iy)
        {
            double temp_m[4];
            unsigned int index = 0;

            for(unsigned short i=0; i<this->num_rows; i++)
            {
                for(unsigned short j=0; j<this->num_cols; j++)
                {
                    if(i == ix || j == iy){
                        continue;
                    }
                    temp_m[index] = matrix[i][j];
                    index++;
                }
            }

            Matrix M(2,2,temp_m);
            double det;
            M.determinante(&det);

            // printf("cof (%d,%d): %f\n", ix, iy, (double) det * pow((-1),(ix+iy)));

            return (double) det * pow((-1),(ix+iy));
        }

        //--------------------------------------------------------
        // ATTENZIONE -> Inverti implementata solo per matrici 3x3
        //--------------------------------------------------------
        void invert3x3 ()
        {
            // STEP 0 -> controllare se la matrice è quadrata
            if(this->num_cols != this->num_rows){
                printf("la matrice non è quadrata, quindi non è invertibile\n");
                return;
            }

            if(num_cols > 3){
                printf("not implemented yet! sorry\n");
                return;
            }

            // STEP 1 -> controllare se il determinante della matrice è nullo
            double det;
            this->determinante(&det);
            if(det == 0){
                printf("il determinante è nullo, quindi non è invertibile\n");
                return;
            }

            // STEP 2 -> se non è nullo, la matrice è invertibile
            double inverted_matrix[this->num_cols * this->num_rows];
            unsigned int index = 0;

            for(unsigned short i=0; i<num_rows; i++)
            {
                for(unsigned short j=0; j<num_cols; j++)
                {
                    inverted_matrix[index] = this->cofattore3x3(i,j);
                    index++;
                }
            }

            for(unsigned short i=0; i<num_rows; i++){
                for(unsigned short j=0; j<num_cols; j++)
                {
                    matrix[i][j] = inverted_matrix[j + num_cols*i];
                }
            }
            this->transpose();
            *(this) = (*this)*(1/det);
        }

};