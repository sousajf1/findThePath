#ifndef FINDTHEPATH_FILEHANDLER_HPP
#define FINDTHEPATH_FILEHANDLER_HPP

#include <fstream>
#include <functional>
#include <iterator>
#include <numeric>
#include <sstream>
#include <string>

struct File {
    static constexpr char const *inExtension = ".cav";
    static constexpr char const *outExtension = ".csn";

    [[nodiscard]] explicit File(std::string fileName) : fileName_(std::move(fileName)) {}

    [[nodiscard]] std::string readInput() const {
        std::fstream fileStream;
        auto fileToOpen = std::string{fileName_} + std::string{inExtension};
        fileStream.open(fileToOpen, std::ios_base::in);
        std::ostringstream os;
        os << fileStream.rdbuf();
        return os.str();
    }
    void writeOutput(std::vector<int> const &solution) const {
        auto fileToWrite = std::string{fileName_} + std::string{outExtension};
        std::ofstream outFile(fileToWrite);

        auto solutionAsString = std::accumulate(std::next(solution.begin()), solution.end(),
                                                std::to_string(solution[0]),
                                                [](const std::string &a, int b) {
                                                    return a + " " + std::to_string(b);
                                                });

        outFile << solutionAsString;
    }

    std::string fileName_;
};

#endif// FINDTHEPATH_FILEHANDLER_HPP
