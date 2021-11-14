#if !defined(MESSAGE_H)
#define MESSAGE

#include <string>
#include <fstream>
#include <iostream>

namespace VLM::IO {   

    void log(std::ostream &out, std::string msg) {
        out << msg << std::endl<< std::flush;
    }

    std::string welcome_msg = 
"-----------------------------------------------------------------------------------------\n"
"VVVVVVVV           VVVVVVVVLLLLLLLLLLL             MMMMMMMM               MMMMMMMM\n"
"V::::::V           V::::::VL:::::::::L             M:::::::M             M:::::::M\n"
"V::::::V           V::::::VL:::::::::L             M::::::::M           M::::::::M\n"
"V::::::V           V::::::VLL:::::::LL             M:::::::::M         M:::::::::M\n"
" V:::::V           V:::::V   L:::::L               M::::::::::M       M::::::::::M\n"
"  V:::::V         V:::::V    L:::::L               M:::::::::::M     M:::::::::::M\n"
"   V:::::V       V:::::V     L:::::L               M:::::::M::::M   M::::M:::::::M\n"
"    V:::::V     V:::::V      L:::::L               M::::::M M::::M M::::M M::::::M\n"
"     V:::::V   V:::::V       L:::::L               M::::::M  M::::M::::M  M::::::M\n"
"      V:::::V V:::::V        L:::::L               M::::::M   M:::::::M   M::::::M\n"
"       V:::::V:::::V         L:::::L               M::::::M    M:::::M    M::::::M\n"
"        V:::::::::V          L:::::L         LLLLLLM::::::M     MMMMM     M::::::M\n"
"         V:::::::V         LL:::::::LLLLLLLLL:::::LM::::::M               M::::::M\n"
"          V:::::V          L::::::::::::::::::::::LM::::::M               M::::::M\n"
"           V:::V           L::::::::::::::::::::::LM::::::M               M::::::M\n"
"            VVV            LLLLLLLLLLLLLLLLLLLLLLLLMMMMMMMM               MMMMMMMM\n"
"------------------------------------------------------------------------------------------\n"                                                                                 
"Author: Omar Kahol                                                                         \n"
"Year: 2021                                                                                 \n"
"Mail: omar.kahol@mail.polimi.it                                                            \n"
"                                                                                           \n"
"                             POLITECNICO DI MILANO                                         \n"
"                    Rotorcraft Aerodynamics course Project                                 \n"
"------------------------------------------------------------------------------------------\n";
std::string parser_msg = "Geometry Loaded";
std::string malloc_msg = "memory allocation completed";
std::string meshinit_msg = "mesh initialized";
std::string free_msg = "memory cleared";
}

#endif // MACRO

