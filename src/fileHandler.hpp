#ifndef FINDTHEPATH_FILEHANDLER_HPP
#define FINDTHEPATH_FILEHANDLER_HPP

#include <fstream>
#include <sstream>
#include <string>
#include <utility>

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
        for (auto const &s : solution) {
            outFile << s << " ";
        }
    }

    std::string fileName_;
};

#endif// FINDTHEPATH_FILEHANDLER_HPP
