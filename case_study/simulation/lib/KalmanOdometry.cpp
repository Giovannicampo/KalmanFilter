#include "Matrix.cpp"

class KalmanOdometry
{
    public:

    public:

    unsigned int order;
    double wheelbase;
    double wheel_factor_left;
    double wheel_factor_right;

    // state vector x y th, initial state to 0
    Matrix* X;

    double x_r;
    double y_r;
    double th_r;
    double a;

    // process covariance
    Matrix* Q;

    // measure covariance 
    Matrix* R;

    Matrix* H;

    // error covariance matrix
    Matrix* P;

    // prediction
    Matrix* K;

    KalmanOdometry (double _wheelbase, double wfl, double wfr)
    {
        this->order = 3;
        this->wheelbase = _wheelbase;
        this-> wheel_factor_left = wfl;
        this->wheel_factor_right = wfr;

        printf("[Kalman Odometry] wb = %.5f, wfl = %.5f, wfr = %.5f\n", _wheelbase, wfl, wfr);

        // state vector x y th, initial state to 0
        double x[] = {0.0, 0.0, 0.0};
        X = new Matrix(3,1,x);
        // printf("[KalmanOdometry] state vector\n");
        // X->print();
        // printf("\n");

        this->x_r = 0.0;
        this->y_r = 0.0;
        this-> th_r = 0.0;

        // a = 0.07
        this->a = 0.0002;

        // process covariance
        Q = new Matrix(this->order);
        *Q = (*Q)*this->a; // value to modify
        // printf("[KalmanOdometry] process covariance\n");
        // Q->print();
        // printf("\n");

        // measure covariance (initially high)
        R = new Matrix(this->order);
        *R = (*R)*5.39;
        // printf("[KalmanOdometry] measure covariance\n");
        // R->print();
        // printf("\n");

        // H matrix of sensor
        double h[] = {0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0};
        H = new Matrix(3,3,h);
        // printf("[KalmanOdometry] H matrix of sensor\n");
        // H->print();
        // printf("\n");

        // error covariance matrix
        double p[] = {0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0};
        P = new Matrix(3,3,p);
        // printf([KalmanOdometry] "error covariance matrix\n");
        // P->print();
        // printf("\n");

        // prediction
        K = new Matrix(0);
    }

    void prediction (double delta_left, double delta_right, double _x_r, double _y_r, double _th_r)
    {
        this->x_r = _x_r;
        this->y_r = _y_r;
        this->th_r = _th_r;

        //printf("Wb: %f", this->wheelbase);

        //printf("[Kalman Odometry] Delta left = %f, delta right = %f\n", delta_left, delta_right);

        double delta_l = (delta_left + delta_right) / 2.0;
        double delta_th = (delta_right - delta_left) / this->wheelbase;

       // printf("[Kalman Odometry] Delta l = %f, delta th = %f\n", delta_l, delta_th);

        double delta_x = delta_l * cos(this->th_r + delta_th / 2.0);
        double delta_y = delta_l * sin(this->th_r + delta_th / 2.0);

        // printf("ci sonooo 2\n");

        this->x_r = this->x_r + delta_x;
        this->y_r = this->y_r + delta_y;
        this->th_r = this->th_r + delta_th;
        //printf("[KalmanOdometry] x:%f, y:%f, th:%f\n", this->x_r, this->y_r, this->th_r);

        double _a[] = {1.0, 0.0, -delta_y,
                      0.0, 1.0, delta_x,
                      0.0, 0.0, 1};
        Matrix* A = new Matrix(3,3,_a);

        double _x[] = {this->x_r, this->y_r, this->th_r};
        // già trasposta
        X = new Matrix(3,1,_x);

        // dynamic update of covariance factor
        double covariance_process_factor = (delta_l * this->a) * (delta_l * this->a);
        // printf("covariance factor = %.10f\n", covariance_process_factor);

        Q = new Matrix(this->order);
        *Q = (*Q)*covariance_process_factor; // value to modify

        // Q->print();

        // printf("ci sonooo 3\n");

        (*P) = (*A) * (*P) * (A->transpose()) + (*Q);

        // P->print();

        Matrix *S = new Matrix(0);
        (*S) = (*H) * (*P) * (H->transpose()) + (*R);

        // S->print();

        (*K) = ((*P) * (H->transpose())) * (S->invert());

        // K->print();
    }

    void measure(Matrix* Measure)
    {
        if(Measure == nullptr){
            printf("[KalmanOdometry] Error\n");
            return;
        }
        (*X) = (*X) + (*K) * ((*Measure) - (*H) * (*X));
        this->x_r = X->getMatrix()[0][0];
        this->y_r = X->getMatrix()[1][0];
        this->th_r = X->getMatrix()[2][0];
        printf("[KalmanOdometry] x:%f, y:%f, th:%f\n", this->x_r, this->y_r, this->th_r);
    }

    void update()
    {
        Matrix* Identity = new Matrix(this->order);
        (*P) = ((*Identity) - (*K) * (*H)) * (*P);
    }
};