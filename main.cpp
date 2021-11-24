#include <sstream>
#include <chrono>

#include "dijkstra.hpp"

int main([[maybe_unused]] int argc, [[maybe_unused]] char * argv[]) {
    auto start = std::chrono::steady_clock::now();
    std::fstream fileStream;
    constexpr char const *inExtension = ".cav";
    constexpr char const *outExtension = ".csn";

    auto fileToOpen = std::string{argv[1]} + std::string{inExtension};
    auto fileToWrite = std::string{argv[1]} + std::string{outExtension};

    fileStream.open(fileToOpen, std::ios_base::in);
    std::ostringstream os;
    os << fileStream.rdbuf();
    std::string input = os.str();

    Graph graph{input};
    std::vector solution = graph.dijkstra();

    std::ofstream outFile(fileToWrite);
    for (auto const &s : solution) {
        outFile << s << " ";
    }
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
    return 0;
}
