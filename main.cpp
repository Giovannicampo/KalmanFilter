#include "lib/Matrix.cpp"
#include "lib/KalmanOdometry.cpp"
#include <map>
#include <random>

int main () 
{
    double measure_error_std = 5;

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{0.0,measure_error_std};

    // double delta_t = 2.5e-3;
    int delta_t = 25;

    int t = 0;

    double x = 0.0;
    double y = 0.0;

    KalmanOdometry* ko = new KalmanOdometry(270.0);
    ko->y_r = 100.0;
    unsigned int i = 0;

    Matrix* Measures = nullptr;
    double _measures[3] = {0.0, 0.0, 0.0};

    while(t < 150000)
    {
        double delta_l = 79.6;
        double delta_r = 79.6005;

        if(t > 500)
        {
            delta_l = 0;
            delta_r = 0;
        }

        ko->prediction(delta_l, delta_r);

        double _measure = 100.0 + std::round(d(gen));
        
        _measures[1] = _measure;
        Measures = new Matrix(1,3,_measures);

        ko->measure(Measures);
        ko->update();

        printf("x %f, y %f\n", ko->x_r, ko->y_r);
        t = t + delta_t;
        i++;
    }
}

