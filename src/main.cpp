#include "clock.hpp"
#include "dijkstra.hpp"
#include "fileHandler.hpp"

int main([[maybe_unused]] int argc, char *argv[]) {
    Clock globalClock{};
    auto fileHandler = File(argv[1]);
    Graph graph{fileHandler.readInput()};
    std::vector solution = graph.dijkstra();
    fileHandler.writeOutput(solution);
    globalClock.print("Total elapsed time: ");
    return 0;
}
