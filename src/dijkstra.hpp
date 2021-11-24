#ifndef DIJKSTRA_HPP_DIJKSTRA_H
#define DIJKSTRA_HPP_DIJKSTRA_H

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <vector>

#include "clock.hpp"
#include "coordinates.hpp"

using Matrix = std::vector<std::vector<double>>;
constexpr auto infinity = std::numeric_limits<double>::max();

struct Graph {
    explicit Graph(std::string const &graphInput) {
        std::vector graphInfo = tokenize(graphInput);
        // on the file provided the first value is the number of nodes
        numberOfNodes = graphInfo[0];
        readNodes(graphInfo);
        readAdjancencies(graphInfo);
    }

    // 0 1 2
    // 3 4 5
    // 6 7 8
    [[nodiscard]] std::size_t indexFromMatrixCoordinates(std::size_t j, std::size_t k) const {
        return (k * numberOfNodes) + j;
    }
    void readAdjancencies(std::vector<int> const &graphInfo) {
        adjacencies.reserve(numberOfNodes);
        Clock timeAdjancyParseClock{};
        // adjancency matrix parse
        for (std::size_t j = 0; j < numberOfNodes; ++j) {
            std::vector<double> jAdjacencies{};// distance from j to k 0.0 means not connected
            jAdjacencies.reserve(numberOfNodes);
            for (std::size_t k = 0; k < numberOfNodes; ++k) {
                std::size_t whereStoppedReadingTheNodes = (numberOfNodes * 2) + 1;
                // numberOfNodes * 2) + 1  where we stopped reading the nodes
                // number of Columns = numberOfNodes
                std::size_t idx = whereStoppedReadingTheNodes + indexFromMatrixCoordinates(j, k);
                bool connected = graphInfo[idx];
                jAdjacencies.push_back(connected ? nodes[j].distanceTo(nodes[k]) : 0.0);
            }
            adjacencies.push_back(jAdjacencies);
        }
        timeAdjancyParseClock.print("Time elapsed parsing adjacencies: ");
    }

    void readNodes(std::vector<int> const &graphInfo) {
        nodes.reserve(numberOfNodes);
        // create coordinates (x,y)
        for (std::size_t i = 1; i < (numberOfNodes) *2 + 1; i = i + 2) {
            nodes.push_back({graphInfo[i], graphInfo[i + 1]});
        }
    }

    [[nodiscard]] std::vector<int> dijkstra() {
        auto const adjacenciesSize = adjacencies.size();

        std::vector<double> shortestDistanceFromSrc;
        shortestDistanceFromSrc.resize(adjacenciesSize, infinity);
        shortestDistanceFromSrc[src] = 0;

        std::vector<int> previousNodeInShortestPath;
        // -1 means not there yet
        previousNodeInShortestPath.resize(adjacenciesSize, -1);

        std::vector<int> visited;
        visited.resize(adjacenciesSize, 0);

        previousNodeInShortestPath[src] = src;

        // for all nodes
        for (std::size_t i = 0; i < adjacenciesSize - 1; i++) {
            std::size_t const closestUnvisitedNode_ = closestUnvisitedNode(shortestDistanceFromSrc, visited);
            visited[closestUnvisitedNode_] = 1;
            if (shortestDistanceFromSrc[closestUnvisitedNode_] == infinity) continue;// none of the unvisited nodes are connected to the visited nodes
            for (std::size_t v = 0; v < adjacenciesSize; v++) {
                if (visited[v]) continue;
                if (!adjacencies[closestUnvisitedNode_][v]) continue;
                // all of distance from src until here + distance from here to the next one
                auto const distanceFromSrcThruClosestUnvisitedNode = shortestDistanceFromSrc[closestUnvisitedNode_] + adjacencies[closestUnvisitedNode_][v];
                // if distance from sr until v is less than the min dist calculated before
                // this is the new min dist
                if (distanceFromSrcThruClosestUnvisitedNode < shortestDistanceFromSrc[v]) {
                    shortestDistanceFromSrc[v] = distanceFromSrcThruClosestUnvisitedNode;
                    previousNodeInShortestPath[v] = closestUnvisitedNode_;
                }
            }
        }
        //create real solution (in the good order)
        createSolution(previousNodeInShortestPath);
        return solution;
    }

    void createSolution(std::vector<int> const &previous) {
        Clock createSolutionClock{};
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
        createSolutionClock.print("Time elapsed creting solution: ");
    }
    static std::vector<int> tokenize(const std::string &str) {
        Clock tokenizeClock{};
        std::vector<int> out;
        constexpr char const delim = ',';
        std::size_t start = str.find_first_not_of(delim);
        std::size_t end = start;
        while (start != std::string::npos) {
            end = str.find(delim, start);
            out.push_back(std::stoi(str.substr(start, end - start)));
            start = str.find_first_not_of(delim, end);
        }
        tokenizeClock.print("Time elapsed tokenizing: ");
        return out;
    }

    static std::size_t closestUnvisitedNode(std::vector<double> const &shortestDistanceFromSrc, std::vector<int> &visited) {
        std::size_t closestIndex{};
        double minDistance = infinity;

        for (std::size_t v = 0; v < shortestDistanceFromSrc.size(); v++) {
            if (visited[v]) continue;
            if (shortestDistanceFromSrc[v] <= minDistance) {
                minDistance = shortestDistanceFromSrc[v];
                closestIndex = v;
            }
        }
        return closestIndex;
    }

    int numberOfNodes{};
    std::vector<Coordinates> nodes{};
    Matrix adjacencies{};
    int src{0};
    std::vector<int> solution{};
};
#endif// DIJKSTRA_HPP_DIJKSTRA_H
