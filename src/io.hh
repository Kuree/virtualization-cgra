#ifndef VIRTUALIZATION_IO_HH
#define VIRTUALIZATION_IO_HH

#include <string>
#include "graph.hh"

// these code copied from the placer
std::unique_ptr<Netlist> load_netlist(const std::string &filename);

void dump_dot_graph(const Graph &graph, const std::string &filename);


#endif//VIRTUALIZATION_IO_HH
