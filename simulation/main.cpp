#include "lib/Matrix.cpp"
#include "lib/KalmanOdometry.cpp"
#include <iostream>
#include <iomanip>
#include <map>
#include <random>

int main () 
{
    double measure_error_std = 2.4;

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{0.0,measure_error_std};

    std::map<int, int> hist_measure{};
    std::map<int, int> hist_prediction{};

    // double delta_t = 2.5e-3;
    int delta_t = 25;

    int t = 0;

    double x = 0.0;
    double y = 0.0;

    KalmanOdometry* ko = new KalmanOdometry(278.0, -20.57, -20.9);
    ko->y_r = 100.0;
    unsigned int i = 0;

    Matrix* Measures = new Matrix(0);
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

        ko->prediction(delta_l, delta_r, ko->x_r, ko->y_r, ko->th_r);

        double _measure = 100.0 + std::round(d(gen));
        ++hist_measure[_measure];
        // printf("[Main] Normal random: %f\n", _measure);
        
        _measures[1] = _measure;

        // già trasposta
        Measures->set(3,1,_measures);

        ko->measure(Measures);
        ko->update();

        ++hist_prediction[ko->y_r];

        printf("[Main] x %f, y %f\n", ko->x_r, ko->y_r);
        t = t + delta_t;
        i++;
    }

    printf("KALMAN FILTER\n\n");

    printf("[Main] MEASURES HISTOGRAM\n");
    for(auto p : hist_measure) {
        std::cout << std::setw(2)
                  << p.first << ' ' << std::string(p.second/50, '*') << '\n';
    }
    printf("\n");

    printf("[Main] PREDICTION HISTOGRAM\n");
    for(auto p : hist_prediction) {
        std::cout << std::setw(2)
                  << p.first << ' ' << std::string(p.second/50, '*') << '\n';
    }
}

