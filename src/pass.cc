#include "pass.hh"

#include <fmt/format.h>

using fmt::format;

bool is_reset_edge(const Edge *edge) {
    auto const *from = edge->from->vertex;
    return from->name[0] == 'i' && edge->width == 1;
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
}