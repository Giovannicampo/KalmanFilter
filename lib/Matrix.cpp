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
        
        // Matrix n x m
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

        // Identity matrix
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
                    printf("%.1f ", matrix[i][j]);
                }
                printf("\n");
            }
        }

        Matrix transpose ()
        {
            double* temp_matrix = new double[num_rows*num_cols];
            unsigned int index = 0;
            for(unsigned short i=0; i<num_rows; i++)
            {
                for(unsigned short j=0; j<num_cols; j++)
                {
                    temp_matrix[index] = this->matrix[j][i];
                    index++;
                }
            }
            return Matrix(num_cols,num_rows,temp_matrix);
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

        Matrix operator+(const Matrix& m)
        {
            if(this->num_rows != m.rows() || this->num_cols != m.cols()){
                printf("Not possible to sum!");
                return Matrix(0);
            }

            unsigned int _num_rows = this->num_rows;
            unsigned int _num_cols = this->num_cols;
            double** m_matrix = m.getMatrix();
            double temp_m[_num_cols * _num_rows];
            unsigned short index = 0;

            for(unsigned short i=0; i<this->num_rows; i++)
            {
                for(unsigned short j=0; j<this->num_cols; j++)
                {
                    temp_m[index] = this->matrix[i][j] + m_matrix[i][j];
                    index++;
                }
            }

            return Matrix(_num_rows, _num_cols, temp_m);
        }

        Matrix operator-(const Matrix& m)
        {
            if(this->num_rows != m.rows() || this->num_cols != m.cols()){
                printf("Not possible to sum!");
                return Matrix(0);
            }

            unsigned int _num_rows = this->num_rows;
            unsigned int _num_cols = this->num_cols;
            double** m_matrix = m.getMatrix();
            double temp_m[_num_cols * _num_rows];
            unsigned short index = 0;

            for(unsigned short i=0; i<this->num_rows; i++)
            {
                for(unsigned short j=0; j<this->num_cols; j++)
                {
                    temp_m[index] = this->matrix[i][j] - m_matrix[i][j];
                    index++;
                }
            }

            return Matrix(_num_rows, _num_cols, temp_m);
        }

        void determinante (double* det)
        {
            if(this->num_cols != this->num_rows){
                printf("la matrice non è quadrata, non è possibile calcolarne il determinante\n");
                return;
            }

            if(num_cols < 0){
                printf("not possible to calculate\n");
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

            if(num_cols == 3){
                *det = (matrix[0][0] * matrix[1][1] * matrix[2][2]) +
                   (matrix[0][1] * matrix[1][2] * matrix[2][0]) +
                   (matrix[0][2] * matrix[1][0] * matrix[2][1]) -
                   (matrix[2][0] * matrix[1][1] * matrix[0][2]) -
                   (matrix[2][1] * matrix[1][2] * matrix[0][0]) -
                   (matrix[2][2] * matrix[1][0] * matrix[0][1]);
                   return;
            }


            printf("\nMatrice %dx%d\n", num_cols, num_cols);
            this->print();
            printf("\n");
            for(unsigned short j=0; j<num_cols; j++)
            {
                printf("Calcolo il cofattore di [%d,%d] della matrice %dx%d: \n",j,0,num_cols, num_cols);
                double val = this->cofattore(j,0) * this->matrix[j][0];
                *det += val;
                printf("determinante locale: %f\n", val);

            }
            printf("\n");
        }

        double cofattore(unsigned int ix, unsigned int iy)
        {
            unsigned int dim = (this->num_rows - 1) * (this->num_cols - 1);
            double temp_m[dim];
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

            Matrix M((this->num_rows - 1),(this->num_cols - 1),temp_m);
            double det;
            M.determinante(&det);

            printf("cof (%d,%d): %f\n", ix, iy, (double) det * pow((-1),(ix+iy)));

            return (double) det * pow((-1),(ix+iy));
        }

        Matrix invert ()
        {
            // STEP 0 -> controllare se la matrice è quadrata
            if(this->num_cols != this->num_rows){
                printf("la matrice non è quadrata, quindi non è invertibile\n");
                return Matrix(0);
            }

            if(num_cols > 3){
                printf("not implemented yet! sorry\n");
                return Matrix(0);
            }

            // STEP 1 -> controllare se il determinante della matrice è nullo
            double det;
            this->determinante(&det);
            if(det == 0){
                printf("il determinante è nullo, quindi non è invertibile\n");
                return Matrix(0);
            }

            // STEP 2 -> se non è nullo, la matrice è invertibile
            double inverted_matrix[this->num_cols * this->num_rows];
            unsigned int index = 0;

            for(unsigned short i=0; i<num_rows; i++)
            {
                for(unsigned short j=0; j<num_cols; j++)
                {
                    inverted_matrix[index] = this->cofattore(i,j);
                    index++;
                }
            }

            Matrix M(3,3,inverted_matrix);
            M = M.transpose();
            return M * (1/det);
        }

};