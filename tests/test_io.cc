#include "gtest/gtest.h"
#include "../src/io.hh"

TEST(io, load_netlist) {    // NOLINT
    auto netlist = load_netlist("cascade.packed");
    auto graph = netlist->graph();
    EXPECT_NE(graph, nullptr);
    auto io_in = graph->port("I20", "io2f_16");
    auto io_out = graph->port("I19", "f2io_16");
    EXPECT_NE(io_in, nullptr);
    EXPECT_NE(io_out, nullptr);
    EXPECT_TRUE(has_path(io_in, io_out));
}