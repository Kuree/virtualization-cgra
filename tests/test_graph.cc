#include "../src/io.hh"
#include "../src/pass.hh"
#include "gtest/gtest.h"

TEST(graph, data_wave) {  // NOLINT
    auto netlist = load_netlist("cascade.packed");
    auto graph = netlist->graph();

    // compute data wave
    EXPECT_NO_THROW(graph->compute_data_wave());
}

TEST(multi_graph, merge_cascade_edge) {  // NOLINT
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

TEST(multi_graph, merge_duplicated_edge) {
    std::map<std::string, std::vector<std::pair<std::string, std::string>>> netlist = {
        {"e1", {{"p1", "a"}, {"p2", "b"}, {"p3", "b"}}},
        {"e2", {{"p2", "c"}, {"p3", "d"}}},
        {"e3", {{"p3", "e"}, {"p1", "f"}}},
    };
    std::map<std::string, uint32_t> bus_mode = {{"e1", 16}, {"e2", 16}, {"e3", 16}};

    Graph graph(netlist, bus_mode);
    MultiGraph mg(&graph, 1);

    // we merge p1 and p2, to see if duplicated edges are removed
    auto p1 = graph.vertex("p1");
    auto p2 = graph.vertex("p2");
    auto s_p1 = mg.find(p1);
    auto s_p2 = mg.find(p2);
    mg.merge(s_p1, s_p2);
    // should be only two vertices left
    EXPECT_EQ(mg.edge_size(), 2);
    // total vertices should be 2 as well
    EXPECT_EQ(mg.vertex_size(), 2);

    // need to further verify the connections
    auto vertices = mg.vertices();
    EXPECT_EQ(vertices.size(), 2);
    EXPECT_TRUE(vertices[0]->vertices.size() == 2 || vertices[1]->vertices.size() == 2);
    auto s_p1_ = vertices[0]->vertices.size() == 2 ? vertices[0] : vertices[1];
    EXPECT_NE(s_p1_->vertices.find(p1), s_p1_->vertices.end());

    // check edges as well
    auto edges = mg.edges();
    EXPECT_EQ(edges.size(), 2);
    EXPECT_TRUE(edges[0]->edges.size() == 2 || edges[1]->edges.size() == 2);
    auto s_p1_p3 = edges[0]->edges.size() == 2 ? edges[0] : edges[1];
    EXPECT_EQ(s_p1_p3->from, s_p1_);
}

TEST(multi_graph, merge_cascade_wave) {
    auto netlist = load_netlist("cascade.packed");
    auto graph = netlist->graph();
    // remove reset
    remove_reset(graph);

    // compute data waves
    graph->compute_data_wave();

    auto mg = MultiGraph(graph, 2);
    constexpr uint32_t seed = 1;
    mg.merge(seed);

    auto vertices = mg.vertices();
    EXPECT_EQ(vertices.size(), 2);
    auto edges = mg.edges();
    EXPECT_EQ(edges.size(), 1);
    auto g_edges = edges[0]->edges;
    EXPECT_EQ(g_edges.size(), 3);
    auto result_ports = mg.edges_to_cut();
    EXPECT_EQ(result_ports.size(), 2);
    // we need to keep two vertices as IO ports
    EXPECT_EQ(mg.score(), 2);
    // seed 0: p8 and p51
    auto p8_alu_res = graph->vertex("p8")->ports["alu_res"];
    auto p51_alu_res = graph->vertex("p51")->ports["alu_res"];
    EXPECT_NE(result_ports.find(p8_alu_res), result_ports.end());
    EXPECT_NE(result_ports.find(p51_alu_res), result_ports.end());
}