#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <map>
#include <random>

#define LEFT_DIM 174
#define RIGHT_DIM 175
#define LEFT_1_5 111

int main()
{
    std::map<int, int> hist_measure_left{};
    std::map<int, int> hist_measure_right{};
    std::map<int, int> hist_measure_left_1_5{};

    FILE* fp = fopen("data/first_data/tot_left_encoder.txt", "r");
    int temp_value = 0;

    for(unsigned short i=0; i<LEFT_DIM; i++)
    {
        fscanf(fp, "%d", &temp_value);
        ++hist_measure_left[(int)temp_value];
    }
    temp_value = 0;

    fclose(fp);

    fp = fopen("data/first_data/tot_right_encoder.txt", "r");
    for(unsigned short i=0; i<RIGHT_DIM; i++)
    {
        fscanf(fp, "%d", &temp_value);
        ++hist_measure_right[(int)temp_value];
    }

    fclose(fp);

    fp = fopen("data/1_5m_measures_left.txt", "r");
    for(unsigned short i=0; i<LEFT_1_5; i++)
    {
        fscanf(fp, "%d", &temp_value);
        ++hist_measure_left_1_5[(int)temp_value];
    }

    fclose(fp);

    printf("[Case_study] LEFT ENCODER HISTOGRAM\n");
    for(auto p : hist_measure_left) {
        std::cout << std::setw(2)
                  << p.first << ' ' << std::string(p.second*3, '*') << '\n';
    }
    printf("\n");

    printf("[Case_study] RIGHT ENCODER HISTOGRAM\n");
    for(auto p : hist_measure_right) {
        std::cout << std::setw(2)
                  << p.first << ' ' << std::string(p.second*3, '*') << '\n';
    }
    printf("\n");

    printf("[Case_study] 1.5 M LEFT ENCODER HISTOGRAM\n");
    for(auto p : hist_measure_left_1_5) {
        std::cout << std::setw(2)
                  << p.first << ' ' << std::string(p.second*3, '*') << '\n';
    }

    printf("\n");
    printf("[Case_study] LEFT ENCODER DATA\n");

    for(auto p : hist_measure_left) {
        std::cout << std::setw(2)
                  << p.first << ' ' << p.second << '\n';
    }

    printf("\n");
    printf("[Case_study] RIGHT ENCODER DATA\n");

    for(auto p : hist_measure_right) {
        std::cout << std::setw(2)
                  << p.first << ' ' << p.second << '\n';
    }

    printf("\n");
    printf("[Case_study] 1.5 M LEFT ENCODER DATA\n");

    for(auto p : hist_measure_left_1_5) {
        std::cout << std::setw(2)
                  << p.first << ' ' << p.second << '\n';
    }
}