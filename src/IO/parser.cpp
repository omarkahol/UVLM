#include <iostream>
#include <vector>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string.hpp>

#include "parser.h"

VLM::IO::parser::parser(const char *filename) {
    file.open(filename,std::ios::in);
    if (! file.is_open()) {
        throw "file not found exception";
    } else {
        data["BLADES"] = std::make_pair<double,bool>(0,false);
        data["RADIUS"] = std::make_pair<double,bool>(0,false);
        data["CHORD"] = std::make_pair<double,bool>(0,false);
        data["CUTOFF"] = std::make_pair<double,bool>(0,false);
        data["NPANELS"] = std::make_pair<double,bool>(0,false);
        data["PITCH"] = std::make_pair<double,bool>(0,false);
        data["ASHAFT"] = std::make_pair<double,bool>(0,false);
        data["OMEGA"] = std::make_pair<double,bool>(0,false);
        data["TFINAL"] = std::make_pair<double,bool>(0,false);
        data["NT"] = std::make_pair<double,bool>(0,false);
        data["ITPRINT"] = std::make_pair<double,bool>(0,false);
    }
}

VLM::IO::parser::~parser() {
    this -> file.close();
}

void VLM::IO::parser::parse() {
    while(file) {
        std::string line;
        std::getline(file,line);
        boost::algorithm::ireplace_all(line," ","");

        if (line.length() < 1 || line[0]=='%') {
            continue;
        } else {
            std::vector<std::string> split_lines;
            boost::algorithm::split(split_lines,line,boost::is_any_of("="));
            data[split_lines[0]] = std::make_pair<double, bool>(std::atof(split_lines[1].c_str()),true);
        }
    }
}

bool VLM::IO::parser::check(std::ostream &out) {
    bool parsed = true; 

    for (auto v: data) {
        if ( !v.second.second) {
            parsed = false;
            out << "KEY " << v.first << " NOT FOUND IN CONFIG FILE!" << std::endl;
        }
    }
    return parsed;
}

const std::map<std::string, std::pair<double,bool>> &VLM::IO::parser::get() {
    return data;
}