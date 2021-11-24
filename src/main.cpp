#include <chrono>
#include <sstream>

#include "dijkstra.hpp"
#include "fileHandler.hpp"

int main([[maybe_unused]] int argc, char *argv[]) {
#if defined(__linux__)
    auto start = std::chrono::steady_clock::now();
#endif
    auto fileHandler = File(argv[1]);
    Graph graph{fileHandler.readInput()};
    std::vector solution = graph.dijkstra();
    fileHandler.writeOutput(solution);
#if defined(__linux__)
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
#endif
    return 0;
}
