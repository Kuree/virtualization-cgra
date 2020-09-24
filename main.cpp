#include <iostream>
#include "io.hh"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <netlist.packed>" << std::endl;
        return EXIT_FAILURE;
    }
    std::string filename = argv[1];
    auto netlist = load_netlist(filename);

    return EXIT_SUCCESS;
}
