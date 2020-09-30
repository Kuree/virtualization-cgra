#ifndef VIRTUALIZATION_PASS_HH
#define VIRTUALIZATION_PASS_HH

#include "graph.hh"
#include <unordered_set>

// get the reset net
std::unordered_set<const Port*> get_reset_ports(Graph *graph);

// remove the reset signal net from the input graph
// reset will be inserted later at each stage
void remove_reset(Graph *graph);

void set_const_pe_wave(Graph* graph);

#endif  // VIRTUALIZATION_PASS_HH
