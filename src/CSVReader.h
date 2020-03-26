#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

class CSVReader
{
private:
    std::string file_name;
    std::string line;
    std::string element;
    
    std::vector<std::pair<std::string, std::vector<float>>> data;
    
    int row_nr;
    
public:
    CSVReader();
    std::vector<std::pair<std::string, std::vector<float>>> get_columns(std::string &filename);
    void display_data();
};