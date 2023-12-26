#pragma once

#include <fstream>
#include <string>
#include <vector>

class fileHandler{
public:
    fileHandler(std::string file_name);
    ~fileHandler();
    void writeFile(std::vector<double> data);
private:
	//ファイル出力用
	std::string output_file_;
	std::ofstream writing_file_;

};