#include "lib/Matrix.cpp"
#include "lib/KalmanOdometry.cpp"

int main () 
{
    // double identity[] = { 1.0, 0.0, 0.0,
    //                       0.0, 1.0, 0.0,
    //                       0.0, 0.0, 1.0};
    // Matrix* m = new Matrix(3,3,identity);
    // Matrix* i = new Matrix(3);
    // // m->print();
    // // i->print();

    // double m1[] = {1.0, 2.0, 3.0, 4.0,
    //                5.0, 6.0, 7.0, 8.0};
    // Matrix* M1 = new Matrix(2,4,m1);
    // // M1->print();
    // // M1->transpose();
    // // M1->print();

    // double m2[] = {1.0, 0.0, 2.0,
    //                0.0, 3.0, -1.0};
    // double m3[] = {4.0, 1.0,
    //                -2.0, 2.0,
    //                0.0, 3.0};
    
    // Matrix M2(2,3,m2);
    // Matrix M3(3,2,m3);
    // Matrix M4 = M2*M3;
    // // M4.print();

    // Matrix M5 = (*i)*5.5;
    // // M5.print();

    // double m6[] = {1.0, 0.0, 1.0,
    //                0.0, 2.0, 0.0,
    //                3.0, 2.0, 0.0};
    // Matrix M6(3,3,m6);
    // double det_m6 = 0;
    // M6.determinante(&det_m6);
    // printf("determinante di M6: %f", det_m6);
    // printf("\n");

    // M6.invert3x3();
    // M6.print();

    KalmanOdometry ko(270.0);
}

