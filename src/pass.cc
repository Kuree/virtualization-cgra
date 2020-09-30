#include "pass.hh"

#include <fmt/format.h>

using fmt::format;

constexpr const char *FLUSH_NAME = "flush";

bool is_reset_edge(const Edge *edge) {
    auto const *from = edge->from->vertex;
    // need to connect to flush
    bool connected_to_flush = false;
    for (auto const *e: from->edges_to) {
        if (e->to->name == FLUSH_NAME) {
            connected_to_flush = true;
            break;
        }
    }
    return from->name[0] == 'i' && edge->width == 1 && connected_to_flush;
};

void remove_reset(Graph *graph) {
    auto edges = graph->get_edges(is_reset_edge);
    Vertex *reset_input = nullptr;
    for (auto const *e : edges) {
        if (reset_input) {
            if (reset_input != e->from->vertex)
                throw std::runtime_error(::format("Only one reset signal allowed. Got {0} {1}",
                                                  reset_input->name, e->from->vertex->name));
        } else {
            reset_input = e->from->vertex;
        }
    }
    graph->remove_vertices({reset_input});

    graph->identify_io();
}

void set_const_pe_wave(Graph* graph) {
    // only works for constant output edge (1-bit), due to the limitation of packer
    auto edges = graph->get_edges([](const Edge *e) -> bool {
        return !e->valid;
    });

    for (auto *e: edges) {
        auto *from = e->from;
        if (from->vertex->name[0] == 'p' && from->vertex->edges_from.empty()) {
            e->wave_number = 0;
            e->valid = true;
        }
    }
}