#ifndef VIRTUALIZATION_GRAPH_HH
#define VIRTUALIZATION_GRAPH_HH

#include <map>
#include <memory>
#include <string>
#include <vector>

struct Edge;
struct Vertex;
struct Port;
class Graph;

class Netlist {
public:
    explicit Netlist(std::shared_ptr<Graph> graph) : graph_(std::move(graph)) {}

private:
    std::shared_ptr<Graph> graph_;
};

class Graph {
public:
    // build graph
    Graph(const std::map<std::string, std::vector<std::pair<std::string, std::string>>> &netlist,
          const std::map<std::string, uint32_t> &track_mode);

    // partition
    std::vector<Edge*> partition(uint32_t num_of_partition);

private:
    // memory storage
    std::vector<std::unique_ptr<Edge>> edges_;
    std::vector<std::unique_ptr<Vertex>> vertices_;
    std::vector<std::unique_ptr<Port>> ports_;

    Port *get_port(const std::pair<std::string, std::string> &name,
                   std::map<std::pair<std::string, std::string>, Port *> &port_to_ptr,
                   std::map<std::string, Vertex *> &vertex_to_ptr);
    Edge *connect(Port *from, Port *to);

    void compute_data_wave();
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

#endif  // VIRTUALIZATION_GRAPH_HH
