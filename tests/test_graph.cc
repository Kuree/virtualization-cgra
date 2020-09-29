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
        {"e2", {{"p3", "c"}, {"p1", "d"}}},
        {"e3", {{"p3", "e"}, {"p4", "f"}}},
    };
    std::map<std::string, uint32_t> bus_mode = {{"e1", 16}, {"e2", 16}, {"e3", 16}};

    Graph graph(netlist, bus_mode);
    MultiGraph mg(&graph, 1);

    // we merge p1 and p2, to see if duplicated edges are removed
    auto p1 = graph.vertex("p1");
    auto p2 = graph.vertex("p2");
    auto p3 = graph.vertex("p3");
    auto s_p1 = mg.find(p1);
    auto s_p2 = mg.find(p2);
    mg.merge(s_p1, s_p2);
    // should be only two edges left
    EXPECT_EQ(mg.edge_size(), 2);
    // total vertices should be 2 as well
    EXPECT_EQ(mg.vertex_size(), 3);

    // need to further verify the connections
    auto vertices = mg.vertices();
    EXPECT_EQ(vertices.size(), 3);
    auto s_p1_ = mg.find(p1);
    auto s_p2_ = mg.find(p2);
    auto s_p3_ = mg.find(p3);
    EXPECT_EQ(s_p1_, s_p2_);

    // check edges as well
    auto edges = mg.edges();
    EXPECT_EQ(edges.size(), 2);
    EXPECT_TRUE(edges[0]->edges.size() == 2 || edges[1]->edges.size() == 2);
    auto s_p1_p3 = edges[0]->edges.size() == 2 ? edges[0] : edges[1];
    EXPECT_TRUE(s_p1_p3->from == s_p1_ || s_p1_p3->from == s_p3_);
}

TEST(multi_graph, merge_cascade_wave) {
    auto netlist = load_netlist("cascade.packed");
    auto graph = netlist->graph();
    // remove reset
    remove_reset(graph);

    // compute data waves
    graph->compute_data_wave();

    auto mg = MultiGraph(graph, 2);
    constexpr uint32_t seed = 42;
    mg.merge(seed);

    auto vertices = mg.vertices();
    EXPECT_EQ(vertices.size(), 2);
    auto edges = mg.edges();
    EXPECT_EQ(edges.size(), 1);
    auto g_edges = edges[0]->edges;
    EXPECT_EQ(g_edges.size(), 2);

    auto result = CutResult(&mg);
    auto score = result.score();
    EXPECT_TRUE(score > 10);
    auto ports = result.get_ports();
    EXPECT_EQ(ports.size(), 2);
    // p44 and r56
    auto p44 = graph->vertex("p44");
    auto p44_port = p44->ports["alu_res"];
    auto r56 = graph->vertex("r56");
    auto r56_port = r56->ports["reg"];
    uint32_t count = 0;
    for (auto const *p: ports) {
        if (p == p44_port)
            count++;
        if (p == r56_port)
            count++;
    }
    EXPECT_EQ(count, 2);
}

TEST(netlist, partition) {
    auto netlist = load_netlist("cascade.packed");
    auto result = netlist->partition(2);
    auto ports = result.get_ports();
    EXPECT_EQ(ports.size(), 1);
}