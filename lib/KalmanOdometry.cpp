#include "Matrix.cpp"

class KalmanOdometry
{
    private:

    unsigned int order;
    double wheelbase;

    // state vector x y th, initial state to 0
    Matrix* X;

    double x_r;
    double y_r;
    double th_r;

    // process covariance
    Matrix* Q;

    // measure covariance 
    Matrix* R;

    Matrix* H;

    // error covariance matrix
    Matrix* P;

    // prediction
    double K;

    public:
        KalmanOdometry (double _wheelbase)
        {
            this->order = 3;
            this->wheelbase = _wheelbase;

            // state vector x y th, initial state to 0
            double x[] = {0.0, 0.0, 0.0};
            X = new Matrix(3,1,x);
            printf("state vector\n");
            X->print();
            printf("\n");

            this->x_r = 0.0;
            this->y_r = 0.0;
            this-> th_r = 0.0;

            // process covariance
            Q = new Matrix(this->order);
            *Q = (*Q)*0.005; // value to modify
            printf("process covariance\n");
            Q->print();
            printf("\n");

            // measure covariance (initially high)
            R = new Matrix(this->order);
            *R = (*R)*1000;
            printf("measure covariance\n");
            R->print();
            printf("\n");

            // H matrix of sensor
            double h[] = {0.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 0.0};
            H = new Matrix(3,3,h);
            printf("H matrix of sensor\n");
            H->print();
            printf("\n");

            // error covariance matrix
            double p[] = {0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0};
            P = new Matrix(3,3,p);
            printf("error covariance matrix\n");
            P->print();
            printf("\n");

            // prediction
            K = 0;
        }

};