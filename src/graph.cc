#include "graph.hh"

#include <fmt/format.h>

#include <cmath>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

using fmt::format;

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
        port_to_ptr.emplace(name, port);
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
            throw std::runtime_error(::format("Loop detected in {0}", v->name));
        // recursive call
        label_edge_data_wave(v, next_wave_number, visited);
    }
    visited.erase(vertex);
}

std::string Graph::dump_dot_graph() const {
    std::stringstream stream;
    using std::endl;

    stream << "digraph D { " << endl;
    // dump all edges
    for (auto const &edge : edges_) {
        auto from = edge->from->vertex;
        auto to = edge->to->vertex;
        stream << "    " << from->name << " -> " << to->name;
        if (edge->valid) {
            stream << ::format(" [ label = \"{0}\" ]", edge->wave_number);
        }
        stream << ";" << endl;
    }

    stream << "}" << endl;

    return stream.str();
}

std::vector<const Edge *> Graph::get_edges(const std::function<bool(const Edge *)> &predicate) {
    std::vector<const Edge *> result;
    for (auto const &edge : edges_) {
        if (predicate(edge.get())) {
            result.emplace_back(edge.get());
        }
    }
    return result;
}

[[maybe_unused]] std::vector<const Vertex *> Graph::get_vertices(
    const std::function<bool(const Vertex *)> &predicate) {
    std::vector<const Vertex *> result;
    for (auto const &vertex : vertices_) {
        if (predicate(vertex.get())) {
            result.emplace_back(vertex.get());
        }
    }
    return result;
}

void Graph::remove_edges(const std::vector<const Edge *> &edges) {
    for (auto const &edge : edges) {
        auto *from = edge->from->vertex;
        auto *to = edge->to->vertex;
        // use templated lambda once C++20 is out
        auto remove_edge = [](std::vector<Edge *> &edges, const Edge *e) {
            auto pos = std::find(edges.begin(), edges.end(), e);
            if (pos == edges.end()) {
                throw std::runtime_error("Invalid graph state");
            }
            edges.erase(pos);
        };
        remove_edge(from->edges_to, edge);
        remove_edge(to->edges_from, edge);

        // remove unique_ptr as well
        bool deleted = false;
        for (auto it = edges_.begin(); it != edges_.end(); it++) {
            if ((*it).get() == edge) {
                edges_.erase(it);
                deleted = true;
                break;
            }
        }
        if (!deleted) throw std::runtime_error("Unable to find target edge to delete");
    }
}

void Graph::remove_vertices(const std::vector<const Vertex *> &vertices) {
    for (auto const *vertex : vertices) {
        auto edges_to = std::vector<const Edge *>(vertex->edges_to.begin(), vertex->edges_to.end());
        auto edges_from =
            std::vector<const Edge *>(vertex->edges_from.begin(), vertex->edges_from.end());
        remove_edges(edges_to);
        remove_edges(edges_from);

        // remove vertex from memory holder
        bool deleted = false;
        for (auto it = vertices_.begin(); it != vertices_.end(); it++) {
            if ((*it).get() == vertex) {
                vertices_.erase(it);
                deleted = true;
                break;
            }
        }
        if (!deleted) throw std::runtime_error("Unable to find target vertex to delete");
    }
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
            throw std::runtime_error(
                ::format("Invalid edge: {0} -> {1}", edge->from->name, edge->to->name));
    }
}

class Partition {
public:
    void add_vertex(Vertex *vertex) {
        vertices_.emplace(vertex);
        // compute if it removes a boundary vertices
        std::unordered_set<const Vertex *> vertex_to_remove;
        for (auto const *v : boundary_vertices_) {
            bool remove = true;
            for (auto const *e : v->edges_to) {
                auto const *e_v = e->to->vertex;
                if (vertices_.find(e_v) != vertices_.end()) {
                    remove = false;
                    break;
                }
            }
            if (remove) {
                vertex_to_remove.emplace(v);
            }
        }
        for (auto const *v : vertex_to_remove) boundary_vertices_.erase(v);

        // compute if it is boundary vertices
        bool is_boundary = false;
        for (auto const *e : vertex->edges_to) {
            auto *e_v = e->to->vertex;
            if (vertices_.find(e_v) == vertices_.end()) {
                is_boundary = true;
                break;
            }
        }
        if (is_boundary) {
            boundary_vertices_.emplace(vertex);
        }
    }

    [[nodiscard]] uint64_t num_boundary_vertices() const { return boundary_vertices_.size(); }

    // default ctor
    Partition() = default;
    // copy ctor
    Partition(const Partition &partition) = default;

private:
    std::unordered_set<const Vertex *> vertices_;
    std::unordered_set<const Vertex *> boundary_vertices_;
};

Vertex *Graph::vertex(const std::string &name) const {
    for (auto const &v : vertices_) {
        if (v->name == name) return v.get();
    }
    return nullptr;
}

Port *Graph::port(const std::string &vertex_name, const std::string &port_name) const {
    for (auto const &v : vertices_) {
        if (v->name == vertex_name) {
            if (v->ports.find(port_name) == v->ports.end())
                throw std::runtime_error(
                    ::format("Unable to find {0}.{1}", vertex_name, port_name));
            return v->ports.at(port_name);
        }
    }
    return nullptr;
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

bool has_path(const Port *from, const Port *to) {
    // depth first search
    std::queue<const Port *> working_set;
    std::unordered_set<const Port *> visited;

    working_set.emplace(from);

    while (!working_set.empty()) {
        auto const *port = working_set.front();
        working_set.pop();
        visited.emplace(from);
        auto const *v = port->vertex;
        for (auto const *e : v->edges_to) {
            auto const *p = e->to;
            if (p == to) return true;
            if (visited.find(p) == visited.end()) {
                working_set.emplace(p);
            }
        }
    }
    return false;
}