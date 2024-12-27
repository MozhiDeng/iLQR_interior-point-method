#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "road_data.h"

std::vector<std::vector<double>> load_data(int i) {

    std::vector<std::string> fileNames;
    std::string input_path = "/home/car/Project/iLQR_interior-point-method/road_data";
    std::string fileName = input_path + "/" + "data" + std::to_string(i) + ".txt";
 
    std::ifstream file(fileName);
    std::cout << fileName << std::endl;
    if(!file.is_open()) {
        std::cout << "Failed to open the file" << std::endl;
    } else {
        //std::cout << "Succeed open the file" << std::endl;
    }

    std::vector<std::vector<double>> data;
    std::string str;
    while(std::getline(file, str)) {
        std::istringstream ss(str);
        std::vector<double> row;
        double value;
        while(ss>>value) {
            row.push_back(value);
        }
        data.push_back(row);
    }
    return data;
}

void print_road_data(std::vector<std::vector<double>>& data) {
    for(int i=0; i<data.size(); i++) {
        for(int j=0; j<data[0].size(); j++) {
            std::cout << data[i][j] << " ";
        }
        std::cout << "\n";
    }
}