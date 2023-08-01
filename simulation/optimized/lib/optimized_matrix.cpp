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

        ~Matrix()
        {
            // printf("Deallocating\n");
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

        void set (unsigned int _num_rows, unsigned int _num_cols, double _matrix[])
        {
            if ( this->num_cols != _num_cols || this->num_rows != _num_rows) {
                printf("[Matrix] Error! Matrix cannot be setted!\n");
                return;
            }

            for(unsigned short i=0; i<num_rows; i++){
                for(unsigned short j=0; j<num_cols; j++)
                {
                    matrix[i][j] = _matrix[j + num_cols*i];
                }
            }
        }

        void setDiagValue (unsigned int order, double value)
        {
            if ( this->num_cols != order || this->num_rows != order) {
                printf("[Matrix] Error! Matrix is not squared!\n");
                return;
            }

            for(unsigned short i=0; i<order; i++)
            {
                this->matrix[i][i] = value;
            }
        }

        static void transpose (Matrix* t1, Matrix* res)
        {
            if (t1 == nullptr || res == nullptr) {
                printf("[Matrix] Error on transposing!\n");
                return;
            }
            unsigned int t_rows = t1->rows();
            unsigned int t_cols = t1->cols();
            double** t_matrix = t1->getMatrix();
            double* temp_matrix = new double[t_rows * t_cols];
            unsigned int index = 0;
            for(unsigned short i=0; i<t_rows; i++)
            {
                for(unsigned short j=0; j<t_cols; j++)
                {
                    temp_matrix[index] = t_matrix[j][i];
                    index++;
                }
            }
            res->set(t_cols, t_rows, temp_matrix);
        }

        static void multiply_row_per_cols(Matrix* m1, Matrix* m2, Matrix* res)
        {
            if(m1 == nullptr || m2 == nullptr || res == nullptr) {
                printf("[Matrix] Error on multiplying!\n");
                return;
            }

            if(m1->cols() != m2->rows()){
                printf("[Matrix] Not possible to multiply!");
                return;
            }

            double** m1_matrix = m1->getMatrix();
            double** m2_matrix = m2->getMatrix();
            unsigned int _num_rows = m1->rows();
            unsigned int _num_cols = m2->cols();

            double temp_m[_num_cols * _num_rows];
            double temp_res = 0.0;
            unsigned short index = 0;

            for(unsigned short i=0; i<m1->rows(); i++)
            {
                for(unsigned short j=0; j<m2->cols(); j++)
                {
                    for(unsigned short k=0; k<m1->cols(); k++)
                    {
                        temp_res += m1_matrix[i][k] * m2_matrix[k][j];
                    }
                    temp_m[index] = temp_res;
                    index++;
                    temp_res = 0;
                }
                
            }
            res->set(_num_rows, _num_cols, temp_m);
        }

        static void multiply_per_value(Matrix* m, double val, Matrix* res)
        {
            if (m == nullptr || res == nullptr) {
                printf("[Matrix] Error on transposing!\n");
                return;
            }
            double** m_matrix = m->getMatrix();
            unsigned int _num_rows = m->rows();
            unsigned int _num_cols = m->cols();
            double temp_m[_num_cols * _num_rows];
            unsigned short index = 0;

            for(unsigned short i=0; i<_num_rows; i++)
            {
                for(unsigned short j=0; j<_num_cols; j++)
                {
                    temp_m[index] = m_matrix[i][j] * val;
                    index++;
                }
            }
            res->set(_num_rows, _num_cols, temp_m);
        } 

        static void sum(Matrix* s1, Matrix* s2, Matrix* res)
        {
            if(s1->rows() != s2->rows() || s1->cols() != s2->cols()){
                printf("[Matrix] Not possible to sum!");
                return;
            }

            unsigned int _num_rows = s1->rows();
            unsigned int _num_cols = s1->cols();
            double** s1_matrix = s1->getMatrix();
            double** s2_matrix = s2->getMatrix();
            double temp_m[_num_cols * _num_rows];
            unsigned short index = 0;

            for(unsigned short i=0; i<_num_rows; i++)
            {
                for(unsigned short j=0; j<_num_cols; j++)
                {
                    temp_m[index] = s1_matrix[i][j] + s2_matrix[i][j];
                    index++;
                }
            }
            res->set(_num_rows, _num_cols, temp_m);
        }

        static void subtract(Matrix* s1, Matrix* s2, Matrix* res)
        {
            if(s1->rows() != s2->rows() || s1->cols() != s2->cols()){
                printf("[Matrix] Not possible to sum!");
                return;
            }

            unsigned int _num_rows = s1->rows();
            unsigned int _num_cols = s1->cols();
            double** s1_matrix = s1->getMatrix();
            double** s2_matrix = s2->getMatrix();
            double temp_m[_num_cols * _num_rows];
            unsigned short index = 0;

            for(unsigned short i=0; i<_num_rows; i++)
            {
                for(unsigned short j=0; j<_num_cols; j++)
                {
                    temp_m[index] = s1_matrix[i][j] - s2_matrix[i][j];
                    index++;
                }
            }
            res->set(_num_rows, _num_cols, temp_m);
        }

        void determinante (double* det)
        {
            *det = 0;
            if(this->num_cols != this->num_rows){
                printf("[Matrix] la matrice non è quadrata, non è possibile calcolarne il determinante\n");
                return;
            }

            if(num_cols < 1){
                printf("[Matrix] not possible to calculate\n");
            }

            switch(num_cols)
            {
                case 1:
                    *det = matrix[0][0];
                    return;
                case 2:
                    *det = (matrix[0][0] * matrix[1][1]) -
                           (matrix[1][0] * matrix[0][1]);
                    return;
                case 3:
                    *det = (matrix[0][0] * matrix[1][1] * matrix[2][2]) +
                           (matrix[0][1] * matrix[1][2] * matrix[2][0]) +
                           (matrix[0][2] * matrix[1][0] * matrix[2][1]) -
                           (matrix[2][0] * matrix[1][1] * matrix[0][2]) -
                           (matrix[2][1] * matrix[1][2] * matrix[0][0]) -
                           (matrix[2][2] * matrix[1][0] * matrix[0][1]);
                        return;
            }


            // printf("\n[Matrix] Matrice %dx%d\n", num_cols, num_cols);
            // this->print();
            // printf("\n");
            for(unsigned short j=0; j<num_cols; j++)
            {
                // printf("[Matrix] Calcolo il cofattore di [%d,%d] della matrice %dx%d: \n",j,0,num_cols, num_cols);
                double val = this->cofattore(j,0) * this->matrix[j][0];
                *det += val;
                // printf("[Matrix] determinante locale: %f\n", val);

            }
            // printf("\n");
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

            // printf("[Matrix] cof (%d,%d): %f\n", ix, iy, (double) det * pow((-1),(ix+iy)));

            return (double) det * pow((-1),(ix+iy));
        }

        static void invert (Matrix* inv, Matrix* res)
        {
            // STEP 0 -> controllare se la matrice è quadrata
            if(inv->cols() != inv->rows()){
                printf("[Matrix] la matrice non è quadrata, quindi non è invertibile\n");
                return ;
            }

            // STEP 1 -> controllare se il determinante della matrice è nullo
            double det;
            inv->determinante(&det);
            if(det == 0){
                printf("[Matrix] il determinante è nullo, quindi non è invertibile\n");
                return;
            }

            unsigned int _cols = inv->cols();
            unsigned int _rows = inv->rows();

            // STEP 2 -> se non è nullo, la matrice è invertibile
            double inverted_matrix[_cols * _rows];
            unsigned int index = 0;

            for(unsigned short i=0; i<_rows; i++)
            {
                for(unsigned short j=0; j<_cols; j++)
                {
                    inverted_matrix[index] = inv->cofattore(i,j);
                    index++;
                }
            }

            res->set(_cols,_rows,inverted_matrix);
            //M = M.transpose();
            Matrix::transpose(res,res);
            Matrix::multiply_per_value(res, (1/det), res);
        }
};