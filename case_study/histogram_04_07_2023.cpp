#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <map>
#include <random>

#define LEFT_DIM 174
#define RIGHT_DIM 175

int main()
{
    std::map<int, int> hist_measure_left{};
    std::map<int, int> hist_measure_right{};

    FILE* fp = fopen("data/tot_left_encoder.txt", "r");
    int temp_value = 0;

    for(unsigned short i=0; i<LEFT_DIM; i++)
    {
        fscanf(fp, "%d", &temp_value);
        ++hist_measure_left[(int)temp_value];
    }
    temp_value = 0;

    fclose(fp);

    fp = fopen("data/tot_right_encoder.txt", "r");
    for(unsigned short i=0; i<RIGHT_DIM; i++)
    {
        fscanf(fp, "%d", &temp_value);
        ++hist_measure_right[(int)temp_value];
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
}