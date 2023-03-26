#include "../include/kdtree.hpp"
#include <chrono>
#include <cstdint>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

using CoordinateType = double;
constexpr std::size_t NUMBER_OF_DIMENSIONS = 3;
constexpr std::size_t NUMBER_OF_POINTS = 100000;

void generateRandomPoints(std::vector<neighbour_search::Point<CoordinateType, NUMBER_OF_DIMENSIONS>> &points)
{
    points.clear();
    points.reserve(NUMBER_OF_POINTS);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<CoordinateType> dist(-10.0, 10.0);

    for (std::size_t i = 0; i < NUMBER_OF_POINTS; ++i)
    {
        neighbour_search::Point<CoordinateType, NUMBER_OF_DIMENSIONS> point{dist(gen), dist(gen), dist(gen)};
        points.push_back(point);
    }
}

std::int32_t main(std::int32_t argc, const char **const argv)
{
    try
    {
        std::vector<neighbour_search::Point<CoordinateType, NUMBER_OF_DIMENSIONS>> points;
        generateRandomPoints(points);

        auto t1 = std::chrono::high_resolution_clock::now();

        neighbour_search::KDTree kdtree(points, true);

        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "KDTree construction time (s): " << (t2 - t1).count() / 1.0e9 << std::endl;

        neighbour_search::Point<CoordinateType, NUMBER_OF_DIMENSIONS> target{0.2, -1.3, 2.5};
        auto t3 = std::chrono::high_resolution_clock::now();

        auto nearest_point = kdtree.findNearestNeighbour(target);

        auto t4 = std::chrono::high_resolution_clock::now();
        std::cout << "KDTree nearest neighbour search time (s): " << (t4 - t3).count() / 1.0e9 << std::endl;
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (...)
    {
        std::cerr << "Unknown exception: " << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}