// #include "optimized_matrix.cpp"
#include <stdio.h>
#include <math.h>

class KalmanOdometry
{
    public:

    unsigned int order;
    double wheelbase;
    double wheel_factor_left;
    double wheel_factor_right;

    // state vector x y th, initial state to 0 [3x1]
    // Matrix* X;
    double X[3][1];

    double x_r;
    double y_r;
    double th_r;
    double a;

    // process covariance [3x3]
    // Matrix* Q;
    double Q[3][3];

    // measure covariance [3x3]
    // Matrix* R;
    double R[3][3];

    // [3x3]
    // Matrix* H;
    double H[3][3];

    // error covariance matrix [3x3]
    // Matrix* P;
    double P[3][3];

    // prediction [3x3]
    // Matrix* K;
    double K[3][3];

    // state matrix [3x3]
    // Matrix* _A;
    double _A[3][3];


    // Identity [3x3]
    // Matrix* Identity;
    double Identity[3][3];

    // [3x3]
    // Matrix* S;
    double S[3][3];

    KalmanOdometry (double _wheelbase, double wfl, double wfr)
    {
        this->order = 3;
        this->wheelbase = _wheelbase;
        this-> wheel_factor_left = wfl;
        this->wheel_factor_right = wfr;

        printf("[KalmanOdometry] wb = %.5f, wfl = %.5f, wfr = %.5f\n", _wheelbase, wfl, wfr);

        // state vector x y th, initial state to 0
        // double x[] = {0.0, 0.0, 0.0};
        // X = new Matrix(3,1,x);
        X[0][0] = 0.0;
        X[1][0] = 0.0;
        X[2][0] = 0.0;
        // printf("[KalmanOdometry] state vector\n");
        // X->print();
        // printf("\n");

        this->x_r = 0.0;
        this->y_r = 0.0;
        this-> th_r = 0.0;

        // a = 0.0002
        this->a = 0.0002;

        // process covariance [3x3]
        for(unsigned short i=0; i<order; i++){
            for(unsigned short j=0; j<order; j++)
            {
                if(i == j) {
                    Q[i][j] = this->a;
                    continue;
                }
                Q[i][j] = 0.0;
            }
        }

        // measure covariance (initially high) [3x3]
        for(unsigned short i=0; i<order; i++){
            for(unsigned short j=0; j<order; j++)
            {
                if(i == j) {
                    R[i][j] = (double)5.39;
                    continue;
                }
                R[i][j] = 0.0;
            }
        }

        // H matrix of sensor [3x3]
        for(unsigned short i=0; i<order; i++){
            for(unsigned short j=0; j<order; j++)
            {
                H[i][j] = 0.0;
            }
        }
        H[1][1] = 1.0;

        // error covariance matrix [3x3]
        for(unsigned short i=0; i<order; i++){
            for(unsigned short j=0; j<order; j++)
            {
                P[i][j] = 0.0;
            }
        }
        // printf([KalmanOdometry] "error covariance matrix\n");

        // prediction [3x3]
        for(unsigned short i=0; i<order; i++){
            for(unsigned short j=0; j<order; j++)
            {
                K[i][j] = 0.0;
            }
        }

        // state matrix [3x3]
        for(unsigned short i=0; i<order; i++){
            for(unsigned short j=0; j<order; j++)
            {
                if(i == j) {
                    _A[i][j] = 1.0;
                    continue;
                }
                _A[i][j] = 0.0;
            }
        }

        // Identity [3x3]
        for(unsigned short i=0; i<order; i++){
            for(unsigned short j=0; j<order; j++)
            {
                if(i == j) {
                    Identity[i][j] = 1.0;
                    continue;
                }
                Identity[i][j] = 0.0;
            }
        }

        // S matrix [3x3]
        for(unsigned short i=0; i<order; i++){
            for(unsigned short j=0; j<order; j++)
            {
                S[i][j] = 0.0;
            }
        }
    }

    static double cofattore(double m[][3], unsigned int ix, unsigned int iy)
    {
        unsigned int index = 0;
        double temp_m[4] = {0, 0, 0, 0};
        for(unsigned short i=0; i<3; i++)
        {
            for(unsigned short j=0; j<3; j++)
            {
                if(i == ix || j == iy){
                    continue;
                }
                temp_m[index] = m[i][j];
                index++;
            }
        }

        double matrix[2][2];
        for(unsigned short i=0; i<2; i++){
            for(unsigned short j=0; j<2; j++)
            {
                matrix[i][j] = temp_m[j + 2*i];
            }
        }

        double det = (matrix[0][0] * matrix[1][1]) - (matrix[1][0] * matrix[0][1]);
        printf("det: %lf\n", det);
        if ( !ix+iy % 2 ) {
            return det;
        }
        return -det;
    }

    static double det3x3(double matrix[][3]) 
    {
        return (double) (matrix[0][0] * matrix[1][1] * matrix[2][2]) +
                        (matrix[0][1] * matrix[1][2] * matrix[2][0]) +
                        (matrix[0][2] * matrix[1][0] * matrix[2][1]) -
                        (matrix[2][0] * matrix[1][1] * matrix[0][2]) -
                        (matrix[2][1] * matrix[1][2] * matrix[0][0]) -
                        (matrix[2][2] * matrix[1][0] * matrix[0][1]);
    }

    void prediction (double delta_left, double delta_right, double _x_r, double _y_r, double _th_r)
    {
        this->x_r = _x_r;
        this->y_r = _y_r;
        this->th_r = _th_r;

        // printf("[KalmanOdometry] Wb: %f\n", this->wheelbase);

        printf("[KalmanOdometry] Delta left = %f, delta right = %f\n", delta_left, delta_right);

        double delta_l = (delta_left + delta_right) / 2.0;
        double delta_th = (delta_right - delta_left) / this->wheelbase;

        printf("[KalmanOdometry] Delta l = %f, delta th = %f\n", delta_l, delta_th);

        double delta_x = delta_l * cos(this->th_r + delta_th / 2.0);
        double delta_y = delta_l * sin(this->th_r + delta_th / 2.0);

        this->x_r = this->x_r + delta_x;
        this->y_r = this->y_r + delta_y;
        this->th_r = this->th_r + delta_th;
        printf("[KalmanOdometry] x:%f, y:%f, th:%f\n", this->x_r, this->y_r, this->th_r);

        // double _a[] = {1.0, 0.0, -delta_y,
        //               0.0, 1.0, delta_x,
        //               0.0, 0.0, 1};
        // _A->set(3,3,_a);
        _A[0][2] = -delta_y;
        _A[1][2] = delta_x;

        // double _x[] = {this->x_r, this->y_r, this->th_r};
        // // già trasposta
        // X->set(3,1,_x);
        X[0][0] = this->x_r;
        X[1][0] = this->y_r;
        X[2][0] = this->th_r;

        // dynamic update of covariance factor
        double covariance_process_factor = (delta_l * this->a) * (delta_l * this->a);
        printf("[KalmanOdometry] covariance factor = %.10f\n", covariance_process_factor);

        // Q->setDiagValue(this->order, covariance_process_factor);
        for(unsigned short i=0; i<order; i++){
            for(unsigned short j=0; j<order; j++)
            {
                if(i == j) {
                    Q[i][j] = this->a;
                    continue;
                }
                Q[i][j] = 0.0;
            }
        }

        // first step ----> (*P) = (*_A) * (*P) * (_A->transpose()) + (*Q);
        // Matrix::multiply_row_per_cols(_A, P, &Partial1);
        double partial1[3][3];
        double temp_res = 0;
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                for(unsigned short k=0; k<order; k++)
                {
                    temp_res += _A[i][k] * P[k][j];
                }
                partial1[i][j] = temp_res;
                temp_res = 0;
            }
        }
        temp_res = 0;

        // Matrix::transpose(_A, &Partial2);
        double partial2[3][3];
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                partial2[i][j] = _A[j][i];
                // printf("%lf ", partial2[i][j]);
            }
            // printf("\n");
        }

        // Matrix::multiply_row_per_cols(&Partial1, &Partial2, &Partial3);
        double partial3[3][3];
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                for(unsigned short k=0; k<order; k++)
                {
                    temp_res += partial1[i][k] * partial2[k][j];
                }
                partial3[i][j] = temp_res;
                temp_res = 0;
                printf("%lf ", partial3[i][j]);
            }
            printf("\n");
        }
        temp_res = 0;

        // Matrix::sum(&Partial3, Q, P);
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                P[i][j] = partial3[i][j] + Q[i][j];
            }
        }

        // second step ----> (*S) = (*H) * (*P) * (H->transpose()) + (*R);
        // Matrix::multiply_row_per_cols(H, P, &Partial1);
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                for(unsigned short k=0; k<order; k++)
                {
                    temp_res += H[i][k] * P[k][j];
                }
                partial1[i][j] = temp_res;
                temp_res = 0;
            }
            
        }
        temp_res = 0;

        // Matrix::transpose(H, &Partial2);
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                partial2[i][j] = H[j][i];
            }
        }

        // Matrix::multiply_row_per_cols(&Partial1, &Partial2, &Partial3);
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                for(unsigned short k=0; k<order; k++)
                {
                    temp_res += partial1[i][k] * partial2[k][j];
                }
                partial3[i][j] = temp_res;
                temp_res = 0;
            }
            
        }
        temp_res = 0;

        // Matrix::sum(&Partial3, R, S);
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                S[i][j] = partial3[i][j] + R[i][j];
            }
        }

        // third step ----> (*K) = ((*P) * (H->transpose())) * (S->invert());
        // Matrix::transpose(H, &Partial1);
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                partial1[i][j] = H[j][i];
            }
        }

        // Matrix::multiply_row_per_cols(P, &Partial1, &Partial2);
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                for(unsigned short k=0; k<order; k++)
                {
                    temp_res += P[i][k] * partial1[k][j];
                }
                partial2[i][j] = temp_res;
                temp_res = 0;
            }
            
        }
        temp_res = 0;

        printf("[KalmanOdometry] Start invert\n");
        // Matrix::invert(S, &Partial3);
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                partial3[i][j] = KalmanOdometry::cofattore(S, i, j) * (1/KalmanOdometry::det3x3(S)); 
            }
        }

        double partial4[3][3];
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                partial4[i][j] = partial3[j][i];
            }
        }

        printf("[KalmanOdometry] End invert\n");
        // Matrix::multiply_row_per_cols(&Partial2, &Partial4, K);
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                for(unsigned short k=0; k<order; k++)
                {
                    temp_res += partial2[i][k] * partial4[k][j];
                }
                K[i][j] = temp_res;
                temp_res = 0;
            }
            
        }
        temp_res = 0;
        printf("[KalmanOdometry] End prediction\n");
    }

    void measure(double Measure[3][1])
    {

        // (*X) = (*X) + (*K) * ((*Measure) - (*H) * (*X));
        // Matrix::multiply_row_per_cols(H, X, &Partial1);
        double partial1[3][3];
        double temp_res = 0;
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<1; j++)
            {
                for(unsigned short k=0; k<order; k++)
                {
                    temp_res += H[i][k] * X[k][j];
                }
                partial1[i][j] = temp_res;
                temp_res = 0;
            }
            
        }
        temp_res = 0;

        // Matrix::subtract(Measure, &Partial1, &Partial2);
        double partial2[3][1];
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<1; j++)
            {
                partial2[i][j] = Measure[i][j] - partial1[i][j];
            }
        }

        // Matrix::multiply_row_per_cols(K, &Partial2, &Partial3);
        double partial3[3][1];
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<1; j++)
            {
                for(unsigned short k=0; k<order; k++)
                {
                    temp_res += K[i][k] * partial2[k][j];
                }
                partial3[i][j] = temp_res;
                temp_res = 0;
            }
            
        }
        temp_res = 0;

        // Matrix::sum(X, &Partial3, X);
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<1; j++)
            {
                X[i][j] = X[i][j] + partial3[i][j];
            }
        }

        this->x_r = X[0][0];
        this->y_r = X[1][0];
        this->th_r = X[2][0];
        printf("[KalmanOdometry] x:%f, y:%f, th:%f\n", this->x_r, this->y_r, this->th_r);
        printf("[KalmanOdometry] End measure\n");
    }

    void update()
    {

        //(*P) = ((*Identity) - (*K) * (*H)) * (*P);
        // Matrix::multiply_row_per_cols(K, H, &Partial1);
        double temp_res = 0;
        double partial1[3][3];
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                for(unsigned short k=0; k<order; k++)
                {
                    temp_res += K[i][k] * H[k][j];
                }
                partial1[i][j] = temp_res;
                temp_res = 0;
            }
            
        }
        temp_res = 0;

        // Matrix::subtract(Identity, &Partial1, &Partial2);
        double partial2[3][3];
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                partial2[i][j] = Identity[i][j] - partial1[i][j];
            }
        }

        // Matrix::multiply_row_per_cols(&Partial2, P, P);
        for(unsigned short i=0; i<order; i++)
        {
            for(unsigned short j=0; j<order; j++)
            {
                for(unsigned short k=0; k<order; k++)
                {
                    temp_res += partial2[i][k] * P[k][j];
                }
                P[i][j] = temp_res;
                temp_res = 0;
            }
            
        }
        temp_res = 0;

        printf("[KalmanOdometry] End update\n");
    }
};
