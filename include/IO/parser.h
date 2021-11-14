#ifndef PARSER_H
#define PARSER_H

#include <fstream>
#include <map>

namespace VLM::IO {
    class parser {
        private: 
            std::fstream file;
            std::map<std::string, std::pair<double,bool>> data;
        public:
            parser(const char *filename);
            ~parser();
            void parse();
            bool check(std::ostream &out);
            const std::map<std::string, std::pair<double,bool>> &get();
    };
}
#endif