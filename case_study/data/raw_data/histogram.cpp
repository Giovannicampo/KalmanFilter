#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <map>
#include <random>

#define DIM 111

int main()
{
    std::map<double, int> hist_measure_0_6{};
    std::map<double, int> hist_measure_0_75{};
    std::map<double, int> hist_measure_1_5{};
    std::map<double, int> hist_measure_3_0{};
    std::map<double, int> hist_measure_4_5{};
    std::map<int, int> hist_measure_tof_0_2{};

    FILE* fp = fopen("0_6m.txt", "r");
    int temp_value = 0;
    double temp_d_value = 0.0;

    for(unsigned short i=0; i<DIM; i++)
    {
        fscanf(fp, "%lf", &temp_d_value);
        ++hist_measure_0_6[(double)temp_d_value];
    }
    temp_d_value = 0.0;

    fclose(fp);

    fp = fopen("0_75m.txt", "r");
    for(unsigned short i=0; i<DIM; i++)
    {
        fscanf(fp, "%lf", &temp_d_value);
        ++hist_measure_0_75[(double)temp_d_value];
    }
    temp_d_value = 0.0;

    fclose(fp);

    fp = fopen("1_5m.txt", "r");
    for(unsigned short i=0; i<DIM; i++)
    {
        fscanf(fp, "%lf", &temp_d_value);
        ++hist_measure_1_5[(double)temp_d_value];
    }
    temp_d_value = 0.0;

    fclose(fp);

    fp = fopen("3_0m.txt", "r");
    for(unsigned short i=0; i<DIM; i++)
    {
        fscanf(fp, "%lf", &temp_d_value);
        ++hist_measure_3_0[(double)temp_d_value];
    }
    temp_d_value = 0.0;

    fclose(fp);

    fp = fopen("4_5m.txt", "r");
    for(unsigned short i=0; i<DIM; i++)
    {
        fscanf(fp, "%lf", &temp_d_value);
        ++hist_measure_4_5[(double)temp_d_value];
    }
    temp_d_value = 0.0;

    fclose(fp);

    fp = fopen("tof_0_2m.txt", "r");
    for(unsigned short i=0; i<DIM+1; i++)
    {
        fscanf(fp, "%d", &temp_value);
        ++hist_measure_tof_0_2[(int)temp_value];
    }

    fclose(fp);

    printf("[Case_study] 0.6 m HISTOGRAM\n");
    for(auto p : hist_measure_0_6) {
        std::cout << std::setw(2)
                  << p.first << ' ' << std::string((int)p.second*3, '*') << '\n';
    }
    printf("\n");

    printf("[Case_study] 0.6 m DATA\n");
    for(auto p : hist_measure_0_6) {
        std::cout << std::setw(2)
                  << p.first << ' ' << p.second << '\n';
    }
    printf("\n");

    printf("[Case_study] 0.75 m HISTOGRAM\n");
    for(auto p : hist_measure_0_75) {
        std::cout << std::setw(2)
                  << p.first << ' ' << std::string(p.second*3, '*') << '\n';
    }
    printf("\n");

    printf("[Case_study] 0.75 m DATA\n");
    for(auto p : hist_measure_0_75) {
        std::cout << std::setw(2)
                  << p.first << ' ' << p.second << '\n';
    }
    printf("\n");

    printf("[Case_study] 1.5 m HISTOGRAM\n");
    for(auto p : hist_measure_1_5) {
        std::cout << std::setw(2)
                  << p.first << ' ' << std::string(p.second*3, '*') << '\n';
    }
    printf("\n");

    printf("[Case_study] 1.5 m DATA\n");
    for(auto p : hist_measure_1_5) {
        std::cout << std::setw(2)
                  << p.first << ' ' << p.second << '\n';
    }
    printf("\n");

    printf("[Case_study] 3.0 m HISTOGRAM\n");
    for(auto p : hist_measure_3_0) {
        std::cout << std::setw(2)
                  << p.first << ' ' << std::string(p.second*3, '*') << '\n';
    }
    printf("\n");

    printf("[Case_study] 3.0 m DATA\n");
    for(auto p : hist_measure_3_0) {
        std::cout << std::setw(2)
                  << p.first << ' ' << p.second << '\n';
    }
    printf("\n");

    printf("[Case_study] 4.5 m HISTOGRAM\n");
    for(auto p : hist_measure_4_5) {
        std::cout << std::setw(2)
                  << p.first << ' ' << std::string(p.second*3, '*') << '\n';
    }
    printf("\n");

    printf("[Case_study] 4.5 m DATA\n");
    for(auto p : hist_measure_4_5) {
        std::cout << std::setw(2)
                  << p.first << ' ' << p.second << '\n';
    }
    printf("\n");

    printf("[Case_study] tof 0.2 m HISTOGRAM\n");
    for(auto p : hist_measure_tof_0_2) {
        std::cout << std::setw(2)
                  << p.first << ' ' << std::string(p.second*3, '*') << '\n';
    }
    printf("\n");

    printf("[Case_study] tof 0.2 mDATA\n");
    for(auto p : hist_measure_tof_0_2) {
        std::cout << std::setw(2)
                  << p.first << ' ' << p.second << '\n';
    }
    printf("\n");
}