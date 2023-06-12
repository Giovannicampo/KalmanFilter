#include "Matrix.cpp"

class KalmanOdometry
{
    public:

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
    Matrix* K;

    KalmanOdometry (double _wheelbase)
    {
        this->order = 3;
        this->wheelbase = _wheelbase;

        // state vector x y th, initial state to 0
        double x[] = {0.0, 0.0, 0.0};
        X = new Matrix(3,1,x);
        // printf("state vector\n");
        // X->print();
        // printf("\n");

        this->x_r = 0.0;
        this->y_r = 0.0;
        this-> th_r = 0.0;

        // process covariance
        Q = new Matrix(this->order);
        *Q = (*Q)*0.005; // value to modify
        // printf("process covariance\n");
        // Q->print();
        // printf("\n");

        // measure covariance (initially high)
        R = new Matrix(this->order);
        *R = (*R)*1000;
        // printf("measure covariance\n");
        // R->print();
        // printf("\n");

        // H matrix of sensor
        double h[] = {0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0};
        H = new Matrix(3,3,h);
        // printf("H matrix of sensor\n");
        // H->print();
        // printf("\n");

        // error covariance matrix
        double p[] = {0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0};
        P = new Matrix(3,3,p);
        // printf("error covariance matrix\n");
        // P->print();
        // printf("\n");

        // prediction
        K = new Matrix(0);
    }

    void prediction (double delta_left, double delta_right)
    {
        double delta_l = (delta_left + delta_right) / 2.0;
        double delta_th = (delta_right - delta_left) / this->wheelbase;

        double delta_x = delta_l * cos(this->th_r + delta_th / 2.0);
        double delta_y = delta_l * sin(this->th_r + delta_th / 2.0);

        this->x_r = this->x_r + delta_x;
        this->y_r = this->y_r + delta_y;
        this->th_r = this->th_r + delta_th;

        double a[] = {1.0, 0.0, -delta_y,
                      0.0, 1.0, delta_x,
                      0.0, 0.0, 1};
        Matrix* A = new Matrix(3,3,a);
        // printf("A\n");
        // A->print();
        // printf("\n");

        double _x[] = {this->x_r, this->y_r, this->th_r};
        X = new Matrix(3,1,_x);
        // printf("X\n");
        // X->print();
        // printf("\n");

        (*P) = (*A) * (*P) * (A->transpose()) + (*Q);
        // printf("P\n");
        // P->print();
        // printf("\n");

        Matrix *S = new Matrix(0);
        (*S) = (*H) * (*P) * (H->transpose()) + (*R);
        // printf("S\n");
        // S->print();
        // printf("\n");

        (*K) = ((*P) * (H->transpose())) * (S->invert());
        // K->print();
    }

    void measure(Matrix* measure)
    {
        (*measure) = measure->transpose();
        (*X) = (*X) + (*K) * ((*measure) - (*H) * (*X));
        this->x_r = X->getMatrix()[0][0];
        this->y_r = X->getMatrix()[1][0];
        this->th_r = X->getMatrix()[2][0];
    }

    void update()
    {
        Matrix* Identity = new Matrix(this->order);
        (*P) = ((*Identity) - (*K) * (*H)) * (*P);
    }
};