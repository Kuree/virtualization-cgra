#include "io.hh"

#include <algorithm>
#include <exception>
#include <fstream>

#define DELIMITER ": \t,()"

using std::map;
using std::runtime_error;
using std::string;
using std::vector;

// trim function copied from https://stackoverflow.com/a/217605
// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) { return !std::isspace(ch); }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) { return !std::isspace(ch); }).base(),
            s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}

inline uint32_t stou(const std::string &str) { return static_cast<uint32_t>(std::stoi(str)); }

inline bool exists(const std::string &filename) {
    std::ifstream in(filename);
    return in.good();
}

::vector<::string> get_tokens(const ::string &line) {
    ::vector<::string> tokens;
    size_t prev = 0, pos = 0;
    ::string token;
    // copied from https://stackoverflow.com/a/7621814
    while ((pos = line.find_first_of(DELIMITER, prev)) != ::string::npos) {
        if (pos > prev) {
            tokens.emplace_back(line.substr(prev, pos - prev));
        }
        prev = pos + 1;
    }
    if (prev < line.length()) tokens.emplace_back(line.substr(prev, std::string::npos));
    return tokens;
}

std::unique_ptr<Netlist> load_netlist(const std::string &filename) {
    if (!::exists(filename)) throw ::runtime_error(filename + " does not exist");
    std::ifstream in;
    in.open(filename);

    ::string line;
    bool in_netlist = false;
    bool in_bus = false;

    ::map<::string, ::vector<std::pair<::string, ::string>>> netlist;
    ::map<::string, uint32_t> track_mode;

    while (std::getline(in, line)) {
        // we are only interested in the packed netlist section
        trim(line);
        if (line[0] == '#') continue;
        if (in_netlist) {
            if (line.empty()) {
                in_netlist = false;
                continue;
            }
            const ::vector<::string> tokens = get_tokens(line);

            if (tokens.size() % 2 != 1) throw ::runtime_error("unable to process line " + line);
            const ::string &net_id = tokens[0];
            ::vector<std::pair<::string, ::string>> net;
            for (uint32_t i = 1; i < tokens.size(); i += 2) {
                ::string const &blk_id = tokens[i];
                ::string const &port = tokens[i + 1];
                net.emplace_back(make_pair(blk_id, port));
            }
            netlist.insert({net_id, net});
            // skip the rest of logic
            continue;
        }

        if (in_bus) {
            if (line.empty()) {
                in_bus = false;
                continue;
            }
            const ::vector<::string> tokens = get_tokens(line);
            if (tokens.size() != 2) throw ::runtime_error("unable to process line " + line);
            const ::string net_id = tokens[0];
            const ::string track_str = tokens[1];

            auto width = static_cast<uint32_t>(std::stoi(track_str));
            track_mode.insert({net_id, width});

            // skip the rest of logic
            continue;
        }

        // state control
        if (line == "Netlists:") {
            in_netlist = true;
            continue;
        } else if (line == "Netlist Bus:") {
            in_bus = true;
        } else {
            in_netlist = false;
            in_bus = false;
        }
    }

    if (netlist.size() != track_mode.size()) {
        throw ::runtime_error("netlist size doesn't match with netlist bus");
    }
    auto graph = std::make_shared<Graph>(netlist, track_mode);
    return std::make_unique<Netlist>(graph);
}
