#include "clock.hpp"
#include "dijkstra.hpp"
#include "fileHandler.hpp"

int main([[maybe_unused]] int argc, char *argv[]) {
#if defined(__linux__)
    Clock globalClock{};
#endif
    auto fileHandler = File(argv[1]);
    Graph graph{fileHandler.readInput()};
    std::vector solution = graph.dijkstra();
    fileHandler.writeOutput(solution);
#if defined(__linux__)
    globalClock.print("Total elapsed time: ");
#endif
    return 0;
}
