#include "graph.hh"

#include <fmt/format.h>

#include <cassert>
#include <queue>
#include <random>
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

std::vector<const Edge *> Graph::get_edges(
    const std::function<bool(const Edge *)> &predicate) const {
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
    (void)max_partition_size;

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

MultiGraph::MultiGraph(const Graph *graph, uint32_t target_wave) : target_wave_(target_wave) {
    auto edges = graph->get_edges([](const Edge *) { return true; });
    // copy the connections over
    // this is necessary because we don't want to make any changes to the original graph
    std::unordered_map<const Vertex *, SuperVertex *> vertex_map;

    for (auto const *edge : edges) {
        auto s_e = get_new_edge();
        auto const *from = edge->from->vertex;
        auto const *to = edge->to->vertex;
        if (vertex_map.find(from) == vertex_map.end()) {
            auto new_v = get_new_vertex();
            new_v->vertices.emplace(from);
            vertex_map.emplace(from, new_v);
        }

        if (vertex_map.find(to) == vertex_map.end()) {
            auto new_v = get_new_vertex();
            new_v->vertices.emplace(to);
            vertex_map.emplace(to, new_v);
        }

        auto s_from = vertex_map.at(from);
        auto s_to = vertex_map.at(to);
        s_e->from = s_from;
        s_e->to = s_to;
        s_e->edges.emplace_back(edge);
        s_from->edges_to.emplace(s_e);
        s_to->edges_from.emplace(s_e);
        s_e->wave_number = edge->wave_number;

        // need to keep track of the wave edge
        if (edge->wave_number == target_wave_) {
            wave_edges_.emplace(s_e);
        } else {
            non_wave_edges_.emplace(s_e);
        }
    }
}

SuperVertex *MultiGraph::get_new_vertex() {
    auto v = std::make_shared<SuperVertex>(this);
    vertices_.emplace(v.get(), v);
    return v.get();
}
SuperEdge *MultiGraph::get_new_edge() {
    auto e = std::make_shared<SuperEdge>();
    edges_.emplace(e.get(), e);
    return e.get();
}

void MultiGraph::delete_edge(SuperEdge *edge) {
    if (edge->wave_number == target_wave_) {
        wave_edges_.erase(edge);
    } else {
        non_wave_edges_.erase(edge);
    }
    edges_.erase(edge);
}

void MultiGraph::delete_vertex(SuperVertex *vertex) {
    if (vertices_.find(vertex) == vertices_.end())
        throw std::runtime_error("Invalid state to delete vertex");
    vertices_.erase(vertex);
}

SuperVertex *MultiGraph::find(const Vertex *vertex) const {
    for (auto const &iter : vertices_) {
        auto s_v = iter.first;
        if (s_v->vertices.find(vertex) != s_v->vertices.end()) {
            return s_v;
        }
    }
    throw std::runtime_error(
        ::format("Unable to find matching super vertex for {0}", vertex->name));
}

template <class T>
auto next(const T &set, uint32_t n) {
    auto it = std::begin(set);
    std::advance(it, n);
    return it;
}

void MultiGraph::merge(uint32_t seed) {
    std::mt19937 gen(seed);
    std::uniform_int_distribution<uint32_t> distrib(0);

    while (edges_.size() > 1) {
        // if we still have non-wave edges left, need to randomly pick one and choose them
        uint32_t index = distrib(gen);
        if (!non_wave_edges_.empty()) {
            // pick an edge from non-wave edges
            index = index % non_wave_edges_.size();
            auto edge_it = next(non_wave_edges_, index);
            auto edge = *edge_it;
            merge(edge);
        } else {
            index = index % wave_edges_.size();
            auto edge_it = next(wave_edges_, index);
            auto edge = *edge_it;
            merge(edge);
        }
    }
}

void MultiGraph::merge(SuperEdge *edge) {
    // merge the two vertices connected by the edge
    auto s_from = edge->from;
    auto s_to = edge->to;
    s_from->merge(s_to);
}

std::vector<SuperEdge *> MultiGraph::edges() const {
    std::vector<SuperEdge *> result;
    result.reserve(edge_size());
    for (auto const &it : edges_) result.emplace_back(it.first);
    return result;
}

std::vector<SuperVertex *> MultiGraph::vertices() const {
    std::vector<SuperVertex *> result;
    result.reserve(vertex_size());
    for (auto const &it : vertices_) result.emplace_back(it.first);
    return result;
}

void SuperVertex::merge(SuperVertex *vertex) {
    // this will delete the other vertex
    // simple way to merge, maybe use set union?
    for (auto const *v : vertex->vertices) vertices.emplace(v);
    // we remove internal edges
    std::unordered_set<SuperEdge *> edges_to_delete;
    for (auto *edge : vertex->edges_to) {
        if (edge->to != this) {
            edge->from = this;
            edges_to.emplace(edge);
        } else {
            // delete this edge
            // this will result in self loop
            edges_from.erase(edge);
            edges_to_delete.emplace(edge);
        }
    }
    for (auto *edge : vertex->edges_from) {
        if (edge->from != this) {
            edge->to = this;
            edges_from.emplace(edge);
        } else {
            // delete this edge
            // this will result in self loop
            edges_to.erase(edge);
            edges_to_delete.emplace(edge);
        }
    }
    // remove duplicated edges
    // we only need to keep one
    // maybe have a better data structure to speed up?
    auto clean_up_connection = [&edges_to_delete](std::unordered_set<SuperEdge *> &target_set,
                                                  bool index_by_to) {
        std::unordered_map<SuperVertex *, std::vector<SuperEdge *>> connections;
        // edges to first
        for (auto *edge : target_set) {
            if (index_by_to)
                connections[edge->to].emplace_back(edge);
            else
                connections[edge->from].emplace_back(edge);
        }
        for (auto const &iter : connections) {
            auto edges = iter.second;
            if (edges.size() > 1) {
                // need to condense it
                auto base_edge = edges[0];
                for (uint64_t i = 1; i < edges.size(); i++) {
                    auto *target_edge = edges[i];
                    target_set.erase(target_edge);
                    for (auto *e : target_edge->edges) base_edge->edges.emplace_back(e);

                    if (index_by_to) {
                        target_edge->to->edges_from.erase(target_edge);
                    } else {
                        target_edge->from->edges_to.erase(target_edge);
                    }
                    edges_to_delete.emplace(target_edge);
                }
            }
        }
    };

    clean_up_connection(edges_to, true);
    clean_up_connection(edges_from, false);

    // delete vertex
    graph_->delete_vertex(vertex);
    for (auto *e : edges_to_delete) {
        graph_->delete_edge(e);
    }
}

std::unordered_set<Port *> MultiGraph::edges_to_cut() const {
    std::unordered_set<Port *> result;
    for (auto iter : edges_) {
        for (auto const *edge : iter.first->edges) {
            result.emplace(edge->from);
        }
    }

    return result;
}

uint64_t MultiGraph::score() const {
    auto edges = edges_to_cut();
    return edges.size();
}