#include "rollerwalker_sim/file_handler.hpp"
#include <iostream>

fileHandler::fileHandler(std::string file_name){
    output_file_ = file_name;
    writing_file_.open(output_file_,std::ios::out);

}
fileHandler::~fileHandler(){
    writing_file_.close();
}

void fileHandler::writeFile(std::vector<double> data){
    std::string writing_text;

    for(int i=0;i<data.size()-1;i++){
        writing_text += std::to_string(data[i]);
        writing_text += "\t," ;
    }
    writing_text += std::to_string(data[data.size()-1]);
    std::cout << writing_text << std::endl;
    writing_file_ << writing_text << std::endl;
}

