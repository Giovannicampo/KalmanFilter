#include "optimized_matrix.cpp"

class KalmanOdometry
{
    public:

    unsigned int order;
    double wheelbase;
    double wheel_factor_left;
    double wheel_factor_right;

    // state vector x y th, initial state to 0 [3x1]
    Matrix* X;

    double x_r;
    double y_r;
    double th_r;
    double a;

    // process covariance [3x3]
    Matrix* Q;

    // measure covariance [3x3]
    Matrix* R;

    // [3x3]
    Matrix* H;

    // error covariance matrix [3x3]
    Matrix* P;

    // prediction [3x3]
    Matrix* K;

    // state matrix [3x3]
    Matrix* _A;

    // Identity [3x3]
    Matrix* Identity;

    // [3x3]
    Matrix* S;

    KalmanOdometry (double _wheelbase, double wfl, double wfr)
    {
        this->order = 3;
        this->wheelbase = _wheelbase;
        this-> wheel_factor_left = wfl;
        this->wheel_factor_right = wfr;

        printf("[KalmanOdometry] wb = %.5f, wfl = %.5f, wfr = %.5f\n", _wheelbase, wfl, wfr);

        // state vector x y th, initial state to 0
        double x[] = {0.0, 0.0, 0.0};
        X = new Matrix(3,1,x);
        // printf("[KalmanOdometry] state vector\n");
        // X->print();
        // printf("\n");

        this->x_r = 0.0;
        this->y_r = 0.0;
        this-> th_r = 0.0;

        // a = 0.0002
        this->a = 0.0002;

        // process covariance [3x3]
        Q = new Matrix(this->order);
        Q->setDiagValue(this->order, this->a);
        // *Q = (*Q)*this->a; // value to modify
        // printf("[KalmanOdometry] process covariance\n");
        // Q->print();
        // printf("\n");

        // measure covariance (initially high) [3x3]
        R = new Matrix(this->order);
        R->setDiagValue(this->order, (double) 5.39);
        //*R = (*R)*5.39;
        // printf("[KalmanOdometry] measure covariance\n");
        // R->print();
        // printf("\n");

        // H matrix of sensor [3x3]
        double h[] = {0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0};
        H = new Matrix(3,3,h);
        // printf("[KalmanOdometry] H matrix of sensor\n");
        // H->print();
        // printf("\n");

        // error covariance matrix [3x3]
        double p[] = {0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0};
        P = new Matrix(3,3,p);
        // printf([KalmanOdometry] "error covariance matrix\n");
        // P->print();
        // printf("\n");

        // prediction [3x3]
        double k[] = {0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0};
        K = new Matrix(3,3,k);

        // state matrix [3x3]
        double _a[] = {1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1};
        _A = new Matrix(3,3,_a);

        // Identity [3x3]
        Identity = new Matrix(this->order);

        // S matrix [3x3]
        double s[] = {0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0};
        S = new Matrix(3,3,s);
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

        double _a[] = {1.0, 0.0, -delta_y,
                      0.0, 1.0, delta_x,
                      0.0, 0.0, 1};
        _A->set(3,3,_a);

        double _x[] = {this->x_r, this->y_r, this->th_r};
        // già trasposta
        X->set(3,1,_x);

        // dynamic update of covariance factor
        double covariance_process_factor = (delta_l * this->a) * (delta_l * this->a);
        printf("[KalmanOdometry] covariance factor = %.10f\n", covariance_process_factor);

        Q->setDiagValue(this->order, covariance_process_factor);

        double _init[] = {0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0};
        Matrix Partial1(3,3,_init);
        Matrix Partial2(3,3,_init);
        Matrix Partial3(3,3,_init);

        // first step ----> (*P) = (*_A) * (*P) * (_A->transpose()) + (*Q);
        Matrix::multiply_row_per_cols(_A, P, &Partial1);
        Matrix::transpose(_A, &Partial2);
        Matrix::multiply_row_per_cols(&Partial1, &Partial2, &Partial3);
        Matrix::sum(&Partial3, Q, P);

        // second step ----> (*S) = (*H) * (*P) * (H->transpose()) + (*R);
        Matrix::multiply_row_per_cols(H, P, &Partial1);
        Matrix::transpose(H, &Partial2);
        Matrix::multiply_row_per_cols(&Partial1, &Partial2, &Partial3);
        Matrix::sum(&Partial3, R, S);

        // third step ----> (*K) = ((*P) * (H->transpose())) * (S->invert());
        Matrix::transpose(H, &Partial1);
        Matrix::multiply_row_per_cols(P, &Partial1, &Partial2);
        printf("[KalmanOdometry] Start invert\n");
        Matrix::invert(S, &Partial3);
        printf("[KalmanOdometry] End invert\n");
        Matrix::multiply_row_per_cols(&Partial2, &Partial3, K);
        printf("[KalmanOdometry] End prediction\n");
    }

    void measure(Matrix* Measure)
    {
        if(Measure == nullptr){
            printf("[KalmanOdometry] Error\n");
            return;
        }

        double _init[] = {0.0, 0.0, 0.0};
        Matrix Partial1(3,1,_init);
        Matrix Partial2(3,1,_init);
        Matrix Partial3(3,1,_init);

        // (*X) = (*X) + (*K) * ((*Measure) - (*H) * (*X));
        Matrix::multiply_row_per_cols(H, X, &Partial1);
        Matrix::subtract(Measure, &Partial1, &Partial2);
        Matrix::multiply_row_per_cols(K, &Partial2, &Partial3);
        Matrix::sum(X, &Partial3, X);

        this->x_r = X->getMatrix()[0][0];
        this->y_r = X->getMatrix()[1][0];
        this->th_r = X->getMatrix()[2][0];
        printf("[KalmanOdometry] x:%f, y:%f, th:%f\n", this->x_r, this->y_r, this->th_r);
        printf("[KalmanOdometry] End measure\n");
    }

    void update()
    {
        double _init[] = {0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0};
        Matrix Partial1(3,3,_init);
        Matrix Partial2(3,3,_init);

        //(*P) = ((*Identity) - (*K) * (*H)) * (*P);
        Matrix::multiply_row_per_cols(K, H, &Partial1);
        Matrix::subtract(Identity, &Partial1, &Partial2);
        Matrix::multiply_row_per_cols(&Partial2, P, P);
        printf("[KalmanOdometry] End update\n");
    }
};
