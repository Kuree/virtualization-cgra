#ifndef VIRTUALIZATION_GRAPH_HH
#define VIRTUALIZATION_GRAPH_HH

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

struct Edge;
struct Vertex;
struct Port;
class Graph;

class Netlist {
public:
    explicit Netlist(std::shared_ptr<Graph> graph) : graph_(std::move(graph)) {}

    [[nodiscard]] Graph *graph() const { return graph_.get(); }

private:
    std::shared_ptr<Graph> graph_;
};

class Graph {
public:
    // build graph
    Graph(const std::map<std::string, std::vector<std::pair<std::string, std::string>>> &netlist,
          const std::map<std::string, uint32_t> &track_mode);

    // partition
    std::vector<Edge *> partition(uint32_t max_partition_size);

    // these two functions are expensive, only used for debugging
    [[nodiscard]] Vertex *vertex(const std::string &name) const;
    [[nodiscard]] Port *port(const std::string &vertex_name, const std::string &port_name) const;

    void compute_data_wave();

    [[nodiscard]] std::string dump_dot_graph() const;

    std::vector<const Edge *> get_edges(const std::function<bool(const Edge *)> &predicate) const;
    [[maybe_unused]] std::vector<const Vertex *> get_vertices(
        const std::function<bool(const Vertex *)> &predicate);
    void remove_edges(const std::vector<const Edge *> &edges);
    void remove_vertices(const std::vector<const Vertex *> &vertices);

private:
    // memory storage
    std::vector<std::unique_ptr<Edge>> edges_;
    std::vector<std::unique_ptr<Vertex>> vertices_;
    std::vector<std::unique_ptr<Port>> ports_;

    Port *get_port(const std::pair<std::string, std::string> &name,
                   std::map<std::pair<std::string, std::string>, Port *> &port_to_ptr,
                   std::map<std::string, Vertex *> &vertex_to_ptr);
    Edge *connect(Port *from, Port *to);
};

struct Edge {
    Port *from = nullptr;
    Port *to = nullptr;
    uint32_t width = 0;

    // used for compute data wave
    uint32_t wave_number = 0;
    bool valid = false;
};

struct Vertex {
    std::string name;
    std::map<std::string, Port *> ports;

    std::vector<Edge *> edges_to;
    std::vector<Edge *> edges_from;
};

struct Port {
    std::string name;

    Vertex *vertex = nullptr;
};

class SuperVertex;
struct SuperEdge {
    std::vector<const Edge *> edges;
    SuperVertex *from;
    SuperVertex *to;

    uint32_t wave_number;
};

class MultiGraph {
public:
    MultiGraph(const Graph *graph, uint32_t target_wave);

    void inline delete_vertex(SuperVertex *vertex) { vertices_.erase(vertex); }
    void delete_edge(SuperEdge *edge);

    // an expensive method. only used for debugging
    [[nodiscard]] SuperVertex *find(const Vertex *vertex) const;

    void merge(uint32_t seed);
    void merge(SuperEdge *edge);
    [[nodiscard]] uint64_t edge_size() const { return edges_.size(); }

private:
    // memory holder
    std::unordered_map<SuperVertex *, std::shared_ptr<SuperVertex>> vertices_;
    std::unordered_map<SuperEdge *, std::shared_ptr<SuperEdge>> edges_;

    // these are designed to speed up the merge process
    std::unordered_set<SuperEdge*> non_wave_edges_;
    std::unordered_set<SuperEdge*> wave_edges_;

    uint32_t target_wave_;

    SuperVertex *get_new_vertex();
    SuperEdge *get_new_edge();
};

class SuperVertex {
public:
    std::unordered_set<const Vertex *> vertices;
    std::unordered_set<SuperEdge *> edges_to;
    std::unordered_set<SuperEdge *> edges_from;

    SuperVertex(MultiGraph *graph) : graph_(graph) {}

    void merge(SuperVertex *vertex);

private:
    MultiGraph *graph_;
};

// compute the partition size
std::vector<std::pair<uint32_t, int>> compute_cut_groups(const std::map<int, uint64_t> &edge_sizes,
                                                         int partition_size);

bool has_path(const Port *from, const Port *to);

#endif  // VIRTUALIZATION_GRAPH_HH
