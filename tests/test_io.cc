#include "gtest/gtest.h"
#include "../src/io.hh"

TEST(io, load_netlist) {    // NOLINT
    auto netlist = load_netlist("cascade.packed");
    auto graph = netlist->graph();
    EXPECT_NE(graph, nullptr);
}