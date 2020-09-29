#include "graph.hh"

#include <fmt/format.h>

#include <cassert>
#include <iostream>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "pass.hh"

using fmt::format;

constexpr bool verify_algorithm = false;

Graph::Graph(const std::map<std::string, std::vector<std::pair<std::string, std::string>>> &netlist,
             const std::map<std::string, uint32_t> &track_mode) {
    // need to maintain some temp data structures for lookup
    std::map<std::pair<std::string, std::string>, Port *> port_to_ptr;
    std::map<std::string, Vertex *> vertex_to_ptr;

    for (auto const &[net_id, net] : netlist) {
        auto const &src = net[0];
        auto src_port = get_port(src, port_to_ptr, vertex_to_ptr);
        auto width = track_mode.at(net_id);
        src_port->width = width;

        for (uint64_t i = 1; i < net.size(); i++) {
            // convert the hyperedge into a digraph
            auto const &sink = net[i];
            auto sink_port = get_port(sink, port_to_ptr, vertex_to_ptr);
            sink_port->width = width;
            // connect the ports
            auto edge = connect(src_port, sink_port);
            edge->width = width;
        }
    }

    // compute inputs and outputs
    identify_io();
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
            continue;
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

// we hardcoded some coded some blk types here
const static std::unordered_set<char> io_vertex = {'i', 'I'};

void Graph::compute_data_wave() {
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

    // label vertex as well
    for (auto &v : vertices_) {
        if (v->edges_to.empty())
            v->wave_number = v->edges_from[0]->wave_number;
        else
            v->wave_number = v->edges_to[0]->wave_number;
    }

    // sanity check
    for (auto const &edge : edges_) {
        if (!edge->valid)
            throw std::runtime_error(
                ::format("Invalid edge: {0} -> {1}", edge->from->name, edge->to->name));
    }
}

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

void Graph::identify_io() {
    inputs_.clear();
    outputs_.clear();
    for (auto const &v : vertices_) {
        if (io_vertex.find(v->name[0]) == io_vertex.end()) continue;
        if (v->edges_from.empty()) {
            for (auto const &iter : v->ports) inputs_.emplace(iter.second);
        }
        if (v->edges_to.empty())
            for (auto const &iter : v->ports) outputs_.emplace(iter.second);
    }
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

bool has_path(const Port *from, const Port *to, const std::unordered_set<const Edge *> &null_set) {
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
            if (null_set.find(e) != null_set.end()) continue;
            auto const *p = e->to;
            if (p == to) return true;
            if (visited.find(p) == visited.end()) {
                working_set.emplace(p);
            }
        }
    }
    return false;
}

MultiGraph::MultiGraph(const Graph *graph, uint32_t target_wave)
    : target_wave_(target_wave), graph_(graph) {
    auto edges = graph->get_edges([](const Edge *) { return true; });
    // copy the connections over
    // this is necessary because we don't want to make any changes to the original graph
    std::unordered_map<const Vertex *, SuperVertex *> vertex_map;

    auto add_new_vertex = [this, &vertex_map](const Vertex *v) {
        auto new_v = get_new_vertex(v->wave_number);
        vertex_map.emplace(v, new_v);
        new_v->vertices.emplace(v);
    };

    for (auto const *edge : edges) {
        auto s_e = get_new_edge();
        auto const *from = edge->from->vertex;
        auto const *to = edge->to->vertex;
        if (vertex_map.find(from) == vertex_map.end()) {
            add_new_vertex(from);
        }

        if (vertex_map.find(to) == vertex_map.end()) {
            add_new_vertex(to);
        }

        auto s_from = vertex_map.at(from);
        auto s_to = vertex_map.at(to);
        s_e->from = s_from;
        s_e->to = s_to;
        s_e->edges.emplace(edge);
        s_from->edges_to.emplace(s_e);
        s_to->edges_from.emplace(s_e);

        // parent
        edge_find_[edge] = s_e;

        if (edge->to->vertex->wave_number == target_wave_ ||
            edge->from->vertex->wave_number == target_wave_) {
            wave_edges_.emplace_back(edge);
        } else {
            non_wave_edges_.emplace_back(edge);
            non_wave_edges_set_.emplace(edge);
        }
    }
}

SuperVertex *MultiGraph::get_new_vertex(uint32_t wave_number) {
    auto v = std::make_shared<SuperVertex>();
    if (wave_number == target_wave_)
        wave_vertices_.emplace(v.get());
    else
        non_wave_vertices_.emplace(v.get());
    v->wave_number = wave_number;
    vertices_.emplace(v.get(), v);
    return v.get();
}
SuperEdge *MultiGraph::get_new_edge() {
    auto e = std::make_shared<SuperEdge>();
    edges_.emplace(e.get(), e);
    return e.get();
}

void MultiGraph::delete_edge(SuperEdge *edge) { edges_.erase(edge); }

void MultiGraph::delete_vertex(SuperVertex *vertex) {
    if (vertices_.find(vertex) == vertices_.end())
        throw std::runtime_error("Invalid state to delete vertex");
    if (vertex->wave_number == target_wave_)
        wave_vertices_.erase(vertex);
    else
        non_wave_vertices_.erase(vertex);

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

    while (vertices_.size() > 2) {
        // if we still have non-wave edges left, need to randomly pick one and choose them
        if (!non_wave_edges_set_.empty()) {
            std::uniform_int_distribution<uint32_t> distrib(0, non_wave_edges_.size() - 1);
            // pick an edge from non-wave edges
            SuperEdge *target;
            do {
                uint32_t index = distrib(gen);
                auto edge = non_wave_edges_[index];
                target = edge_find_.at(edge);
            } while (target == nullptr);
            merge(target);
        } else {
            std::uniform_int_distribution<uint32_t> distrib(0, wave_edges_.size() - 1);
            SuperEdge *target;
            do {
                uint32_t index = distrib(gen);
                auto edge = wave_edges_[index];
                target = edge_find_.at(edge);
            } while (target == nullptr);
            merge(target);
        }
    }
}

void MultiGraph::merge(SuperEdge *edge) {
    // merge the two vertices connected by the edge
    auto s_from = edge->from;
    auto s_to = edge->to;
    merge(s_from, s_to);
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

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

void MultiGraph::merge(SuperVertex *a, SuperVertex *b) {
    // check before and after size
    uint64_t edge_count = 0;
    if constexpr (verify_algorithm) {  // NOLINT
        for (auto const &iter : edges_) {
            edge_count += iter.first->edges.size();
        }
    }

    // this will delete both vertex a and b
    // simple way to merge, maybe use set union?
    // brute force
    // figure the wave number
    // wave number is the maximum
    uint32_t wave_number = 0;
    for (auto const *v : a->vertices)
        if (v->wave_number > wave_number) wave_number = v->wave_number;
    for (auto const *v : b->vertices)
        if (v->wave_number > wave_number) wave_number = v->wave_number;

    auto new_vertex = get_new_vertex(wave_number);

    for (auto *v : a->vertices) new_vertex->vertices.emplace(v);
    for (auto *v : b->vertices) new_vertex->vertices.emplace(v);

    // only interested in the neighbor edges
    std::set<SuperEdge *> edges;
    for (auto *e : a->edges_to) edges.emplace(e);
    for (auto *e : a->edges_from) edges.emplace(e);
    for (auto *e : b->edges_to) edges.emplace(e);
    for (auto *e : b->edges_from) edges.emplace(e);

    for (auto &e : edges) {
        if (e->to == a) e->to = new_vertex;
        if (e->from == a) e->from = new_vertex;
        if (e->to == b) e->to = new_vertex;
        if (e->from == b) e->from = new_vertex;
    }
    // delete edges
    std::unordered_map<SuperEdge *, SuperEdge *> edges_to_remove;
    std::unordered_map<std::pair<SuperVertex *, SuperVertex *>, SuperEdge *, pair_hash>
        connection_map;
    for (auto &edge : edges) {
        auto from = edge->from;
        auto to = edge->to;
        auto p = std::make_pair(from, to);
        if (connection_map.find(p) == connection_map.end()) {
            connection_map.emplace(p, edge);
            if (from == new_vertex) new_vertex->edges_to.emplace(edge);
            if (to == new_vertex) new_vertex->edges_from.emplace(edge);
        } else {
            // merge the edge to the target
            auto target = connection_map.at(p);
            target->merge(edge);
            from->edges_to.erase(edge);
            to->edges_from.erase(edge);
            edges_to_remove.emplace(edge, target);
        }
    }
    // check backward loop
    std::unordered_map<std::pair<SuperVertex *, SuperVertex *>,
                       std::pair<SuperVertex *, SuperVertex *>, pair_hash>
        loop_map;
    for (auto &iter : connection_map) {
        auto &p = iter.first;
        auto [from, to] = p;
        auto pair = std::make_pair(to, from);
        if (connection_map.find(pair) != connection_map.end()) {
            // don't double count
            if (loop_map.find(pair) == loop_map.end()) {
                loop_map.emplace(p, pair);
            }
        }
    }
    for (auto &iter : loop_map) {
        auto target_pair = iter.first;
        auto base_pair = iter.second;
        auto target_e = connection_map.at(target_pair);
        auto base_e = connection_map.at(base_pair);
        target_e->merge(base_e);
        auto [from, to] = base_pair;
        from->edges_to.erase(base_e);
        to->edges_from.erase(base_e);
        edges_to_remove.emplace(base_e, nullptr);
    }

    // check self loop
    uint64_t num_self_loop = 0;
    auto loop = std::make_pair(new_vertex, new_vertex);
    if (connection_map.find(loop) != connection_map.end()) {
        auto edge = connection_map.at(loop);
        edges_to_remove.emplace(edge, nullptr);
        new_vertex->edges_from.erase(edge);
        new_vertex->edges_to.erase(edge);
        num_self_loop += edge->edges.size();
    }

    for (auto [e, parent] : edges_to_remove) {
        for (auto *edge : e->edges) {
            edge_find_[edge] = parent;
            if (non_wave_edges_set_.find(edge) != non_wave_edges_set_.end() && !parent)
                non_wave_edges_set_.erase(edge);
        }
        delete_edge(e);
    }

    // sanity check
    if constexpr (verify_algorithm) {  // NOLINT
        uint64_t new_edge_count = 0;
        for (auto const &iter : edges_) {
            new_edge_count += iter.first->edges.size();
        }
        assert((new_edge_count + num_self_loop) == edge_count);
        auto es = this->edges();
        for (auto *edge : es) {
            if (edge->to == b || edge->from == b) {
                throw std::runtime_error("invalid state");
            }
            if (edge->to == edge->from) {
                throw std::runtime_error("invalid state");
            }
        }
    }

    delete_vertex(a);
    delete_vertex(b);
}

void MultiGraph::print_graph() const {
    for (auto const &iter : vertices_) {
        auto *p = iter.first;
        std::cout << ::format("Vertex: 0x{0:X} ", (uint64_t)p);
        for (auto v : p->vertices) std::cout << " " << v->name;
        std::cout << std::endl;
        std::cout << "   Connections: ";
        for (auto e : p->edges_to) std::cout << ::format(" 0x{0:X}", (uint64_t)e->to);
        std::cout << std::endl;
    }

    std::cout << "---------------------" << std::endl;
}

CutResult::CutResult(MultiGraph *graph)
    : targeted_wave_(graph->target_wave_num()),
      inputs_(graph->inputs()),
      outputs_(graph->outputs()) {
    auto edges = graph->edges();
    for (auto const *e : edges) {
        for (auto const *edge : e->edges) {
            edges_.emplace(edge);
        }
    }
    auto vertices = graph->vertices();
    if (vertices.size() != 2) throw std::runtime_error("Graph not cut properly");
    separators_ = std::make_pair(vertices[0]->vertices, vertices[1]->vertices);
}

uint32_t CutResult::score() const {
    // we need to make sure that the cut will actually prevent the flow
    // we assume it's a connected graph where the inputs can always goes to the outputs
    uint32_t s = 0;
    for (auto *in : inputs_) {
        for (auto *out : outputs_) {
            if (in->width != out->width) continue;
            if (has_path(in, out, edges_)) {
                s += 100;
            }
        }
    }
    // we cannot cut not in the targeted wave
    for (auto *e : edges_) {
        if (e->wave_number != targeted_wave_ && e->to->vertex->wave_number != targeted_wave_)
            s += 100;
    }
    return s + static_cast<uint32_t>(get_ports().size());
}

std::unordered_set<Port *> CutResult::get_ports() const {
    std::unordered_set<Port *> ports;
    for (auto *e : edges_) ports.emplace(e->from);
    return ports;
}

CutResult Netlist::partition(uint32_t wave_number) {
    remove_reset(graph_.get());
    graph_->compute_data_wave();

    CutResult result;
    uint32_t score = 0xFFFFFFFF;
    for (uint32_t i = 0; i < graph_->vertex_count() * graph_->vertex_count() / 4; i++) {
        MultiGraph mg(graph_.get(), wave_number);
        mg.merge(i);
        CutResult r(&mg);
        auto s = r.score();
        if (s < score) {
            score = s;
            result = r;
        }
    }

    return result;
}