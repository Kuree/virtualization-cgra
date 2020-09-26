#include "../src/io.hh"
#include "../src/pass.hh"
#include "gtest/gtest.h"

TEST(graph, data_wave) {  // NOLINT
    auto netlist = load_netlist("cascade.packed");
    auto graph = netlist->graph();

    // compute data wave
    EXPECT_NO_THROW(graph->compute_data_wave());

}

TEST(multi_graph, merge) {  // NOLINT
    auto netlist = load_netlist("cascade.packed");
    auto graph = netlist->graph();
    // remove reset
    remove_reset(graph);

    auto mg = MultiGraph(graph, 2);
    // a random vertex
    auto m58 = graph->vertex("m58");
    auto sv_m58 = mg.find(m58);
    EXPECT_NE(sv_m58, nullptr);
    auto se = *(sv_m58->edges_from.begin());
    auto num_edges_m58 = sv_m58->edges_to.size();

    EXPECT_EQ(se->from->vertices.size(), 1);
    auto io = *(se->from->vertices.begin());
    auto s_io = mg.find(io);
    auto num_edges_io = s_io->edges_to.size();

    EXPECT_NE(s_io, nullptr);
    auto io_name = io->name;
    EXPECT_EQ(io_name, "I20");

    // merge that edge
    mg.merge(se);
    EXPECT_EQ(s_io, mg.find(m58));
    EXPECT_EQ(s_io->edges_to.size(), num_edges_io + num_edges_m58 - 1);
}