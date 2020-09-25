#include "../src/io.hh"
#include "gtest/gtest.h"

TEST(graph, data_wave) {  // NOLINT
    auto netlist = load_netlist("cascade.packed");
    auto graph = netlist->graph();

    // compute data wave
    EXPECT_NO_THROW(graph->compute_data_wave());

}