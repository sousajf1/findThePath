#ifndef DIJKSTRA_HPP_DIJKSTRA_H
#define DIJKSTRA_HPP_DIJKSTRA_H

#include <cmath>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <vector>

#include "coordinates.hpp"

using Matrix = std::vector<std::vector<double>>;
constexpr auto infinity = std::numeric_limits<double>::max();

struct Graph {
    explicit Graph(std::string const &graphInput) {
        std::vector graphInfo = stringSplit(graphInput);
        // on the file provided the first value is the number of nodes
        numberOfNodes = graphInfo[0];
        nodes.reserve(numberOfNodes);
        // create coordinates (x,y)
        for (std::size_t i = 1; i < (numberOfNodes) * 2 + 1; i = i + 2) {
            nodes.push_back({graphInfo[i], graphInfo[i + 1]});
        }
        adjacencies.reserve(numberOfNodes);
        // adjancency matrix parse
        for (std::size_t j = 0; j < numberOfNodes; ++j) {
            std::vector<double> jAdjacencies{};// from j to k
            jAdjacencies.reserve(numberOfNodes);
            for (std::size_t k = 0; k < numberOfNodes; ++k) {
                bool connected = graphInfo[(numberOfNodes * 2) + (k * numberOfNodes) + j + 1];
                jAdjacencies.push_back(connected ? nodes[j].distanceTo(nodes[k]) : 0.0);
            }
            adjacencies.push_back(jAdjacencies);
        }
    }

    [[nodiscard]] std::vector<int> dijkstra() {
        auto const adjacenciesSize = adjacencies.size();
        std::vector<double> distanceFromSrc;
        distanceFromSrc.resize(adjacenciesSize, infinity);
        std::vector<int> previous;
        previous.resize(adjacenciesSize, -1);
        std::vector<int> vertexInShortestPath;
        vertexInShortestPath.resize(adjacenciesSize, 0);

        distanceFromSrc[src] = 0;
        previous[src] = src;

        for (std::size_t i = 0; i < adjacenciesSize - 1; i++) {
            int const minDistVertex = minimumDistance(distanceFromSrc, vertexInShortestPath);
            vertexInShortestPath[minDistVertex] = 1;
            for (std::size_t v = 0; v < adjacenciesSize; v++) {
                if (vertexInShortestPath[v]) continue;
                if (!adjacencies[minDistVertex][v]) continue;
                if (distanceFromSrc[minDistVertex] == infinity) continue;
                // no need to compute this twice for each iteration
                auto const distToCheck = distanceFromSrc[minDistVertex] + adjacencies[minDistVertex][v];
                if (distanceFromSrc[v] > distToCheck) {
                    distanceFromSrc[v] = distToCheck;
                    previous[v] = minDistVertex;
                }
            }
        }
        //create real solution (in the good order)
        createSolution(previous);
        return solution;
    }

     void createSolution(std::vector<int> const& previous) {
        if (previous[adjacencies.size() - 1] == -1) {
            solution.emplace_back(0);
            return;
        }
        std::stack<int> result;
        // from the last value we check which was the previous to print it
        for (std::size_t i = adjacencies.size() - 1; i != 0; i = previous[i]) {
            result.push(i + 1);
        }
        // when getting to 0 loop goes away
        solution.emplace_back(1);
        while (!result.empty()) {
            solution.emplace_back(result.top());
            result.pop();
        }
    }

    static std::vector<int> stringSplit(std::string s){
        std::vector<int> out{};
        std::string const delim = ",";

        size_t pos;
        std::string token;
        while ((pos = s.find(delim)) != std::string::npos) {
            token = s.substr(0, pos);
            s.erase(0, pos + delim.length());
            out.push_back(std::stoi(token));
        }
        out.push_back(std::stoi(s));
        return out;
    }

    static int minimumDistance(std::vector<double> const &distanceFromSrc, std::vector<int> &vertexInShortestPath) {
        int minIdx{};
        double min = infinity;
        auto const distanceFromSrcSize = distanceFromSrc.size();

        for (std::size_t v = 0; v < distanceFromSrcSize; v++) {
            if (!vertexInShortestPath[v] && distanceFromSrc[v] <= min) {
                min = distanceFromSrc[v];
                minIdx = v;
            }
        }
        return minIdx;
    }

    int numberOfNodes{};
    std::vector<Coordinates> nodes{};
    Matrix adjacencies{};
    int src{0};
    std::vector<int> solution{};
};
#endif // DIJKSTRA_HPP_DIJKSTRA_H
