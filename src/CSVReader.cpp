#include "CSVReader.h"

CSVReader::CSVReader()
{
    
}

void CSVReader::display_data()
{
    if (data.size() == 0)
    {
        std::cout << "No data found.";
        return;
    }
    
    int datasize = data.size();
    for (int i = 0; i < datasize; i++)
    {
        std::cout << data.at(i).first;
        for (int j = 0; j < 10; j++)
        {
            std::cout << data.at(i).second.at(j);
        }
        std::cout << std::endl;
    }
}


std::vector<std::pair<std::string, std::vector<float>>> CSVReader::get_columns(std::string &filename)
{
    file_name = filename;
    std::cout << "Reading from file: \"" << file_name << "\"\n";

    std::ifstream file("data/" + file_name);
    
    getline(file,line);
    std::istringstream row(line);
    while (getline(row, element, ','))
        {
            data.push_back({{element},{}});
        }

    while (getline(file,line))
    {
        std::istringstream row(line);
        row_nr = 0;
        while (getline(row, element, ','))
        {
            data.at(row_nr).second.push_back(std::stof(element));
            row_nr++;
        }
    }

    file.close();
    
    return data;
}