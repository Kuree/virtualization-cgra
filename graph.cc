#include "graph.hh"

#include <cmath>
#include <unordered_map>
#include <unordered_set>

Graph::Graph(const std::map<std::string, std::vector<std::pair<std::string, std::string>>> &netlist,
             const std::map<std::string, uint32_t> &track_mode) {
    // need to maintain some temp data structures for lookup
    std::map<std::pair<std::string, std::string>, Port *> port_to_ptr;
    std::map<std::string, Vertex *> vertex_to_ptr;

    for (auto const &[net_id, net] : netlist) {
        auto const &src = net[0];
        auto src_port = get_port(src, port_to_ptr, vertex_to_ptr);
        auto width = track_mode.at(net_id);

        for (uint64_t i = 1; i < net.size(); i++) {
            // convert the hyperedge into a digraph
            auto const &sink = net[i];
            auto sink_port = get_port(sink, port_to_ptr, vertex_to_ptr);
            // connect the ports
            auto edge = connect(src_port, sink_port);
            edge->width = width;
        }
    }
}

Edge *Graph::connect(Port *from, Port *to) {
    edges_.emplace_back(std::make_unique<Edge>());
    auto edge = edges_.back().get();
    edge->from = from;
    edge->to = to;
    from->vertex->edges_to.emplace_back(edge);
    to->vertex->edges_from.emplace_back(edge);
    return edge;
}

Port *Graph::get_port(const std::pair<std::string, std::string> &name,
                      std::map<std::pair<std::string, std::string>, Port *> &port_to_ptr,
                      std::map<std::string, Vertex *> &vertex_to_ptr) {
    if (port_to_ptr.find(name) == port_to_ptr.end()) {
        auto ptr = std::make_unique<Port>();
        // need to check if the vertex exists or not
        auto blk_name = name.first;
        if (vertex_to_ptr.find(blk_name) == vertex_to_ptr.end()) {
            vertices_.emplace_back(std::make_unique<Vertex>());
            auto vertex = vertices_.back().get();
            vertex->name = blk_name;
            vertex_to_ptr.emplace(blk_name, vertex);
        }

        Vertex *vertex = vertex_to_ptr.at(blk_name);
        ports_.emplace_back(std::make_unique<Port>());
        Port *port = ports_.back().get();
        port->name = name.second;
        port->vertex = vertex;
        vertex->ports.emplace(port->name, port);
    }
    return port_to_ptr.at(name);
}

void label_edge_data_wave(Vertex *vertex, uint32_t wave_number,
                          std::unordered_set<Vertex *> &visited) {
    const static std::unordered_set<char> timed_vertex = {'r', 'm'};
    auto blk_type = vertex->name[0];
    uint32_t next_wave_number = wave_number;
    if (timed_vertex.find(blk_type) != timed_vertex.end()) next_wave_number++;
    // visited is used for loops
    // if we loop back to visited, it means we have a combinational loop, which is illegal
    // in this algorithm
    // we can use a pre-pass on the graph that group the loop body into a single vertex.
    visited.emplace(vertex);

    for (auto const &edge : vertex->edges_to) {
        edge->valid = true;
        if (next_wave_number > edge->wave_number) edge->wave_number = next_wave_number;
        auto v = edge->to->vertex;
        // prevent loop
        if (visited.find(v) != visited.end())
            throw std::runtime_error("Loop detected in " + v->name);
        // recursive call
        label_edge_data_wave(v, next_wave_number, visited);
    }
    visited.erase(vertex);
}

void Graph::compute_data_wave() {
    // we hardcoded some coded some blk types here
    const static std::unordered_set<char> io_vertex = {'i', 'I'};

    std::unordered_set<Vertex *> visited;

    // start searching from the vertices
    // notice that all the inputs
    for (auto const &v : vertices_) {
        if (io_vertex.find(v->name[0]) == io_vertex.end()) continue;
        // input should not have incoming edges
        if (!v->edges_from.empty()) continue;

        // need to recursively label the wave
        // any edges start from IO is 0
        label_edge_data_wave(v.get(), 0, visited);
    }

    // sanity check
    for (auto const &edge : edges_) {
        if (!edge->valid)
            throw std::runtime_error("Invalid edge " + edge->from->name + " -> " + edge->to->name);
    }
}

std::vector<Edge *> Graph::partition(uint32_t max_partition_size) {
    // for PnR's purpose, given a max size, usually can do a good job around 80%-90% usage
    // we use 0.8 here just to do a approximation
    constexpr double ratio = 0.8;
    auto partition_size = static_cast<int>(max_partition_size * ratio);
    // we use modified Karger's algorithm that preserves the data wave
    // first we index the edge by its wave number
    // notice that map is sorted
    std::map<int, std::unordered_set<Edge *>> wave_edge_map;
    std::map<int, uint64_t> edge_sizes;
    for (auto const &edge : edges_) {
        wave_edge_map[edge->wave_number].emplace(edge.get());
    }
    for (auto const &[cut_num, edges] : wave_edge_map) {
        edge_sizes.emplace(cut_num, edges.size());
    }

    auto cut_eave_numbers = compute_cut_groups(edge_sizes, partition_size);

    (void)cut_eave_numbers;

    return {};
}

std::vector<std::pair<uint32_t, int>> compute_cut_groups(const std::map<int, uint64_t> &edge_sizes,
                                                         int partition_size) {
    // calculate the place to cut, i.e., which wave number to cut
    std::vector<std::pair<uint32_t, int>> cut_wave_numbers;
    int current_size = 0;
    for (auto const &[wave_number, edge_size_] : edge_sizes) {
        auto edge_size = static_cast<int>(edge_size_);
        while (edge_size > 0) {
            if (current_size + partition_size < edge_size) {
                // this is one partition
                auto size = partition_size - current_size;
                cut_wave_numbers.emplace_back(std::make_pair(wave_number, size));
                current_size = 0;
                edge_size -= size;
            } else {
                edge_size = 0;
                current_size += edge_size;
            }
        }
    }
    return cut_wave_numbers;
}