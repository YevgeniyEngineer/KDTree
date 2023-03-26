#include "kdtree.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

int main()
{
    using namespace data_structure;

    std::size_t number_of_points = 10'000'000;
    constexpr std::size_t number_of_dimensions = 3;

    try
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(-10.0, 10.0);

        std::vector<KDTreePoint<number_of_dimensions>> points(number_of_points);
        for (auto i = 0; i < number_of_points; ++i)
        {
            double x = dist(gen);
            double y = dist(gen);
            double z = dist(gen);
            KDTreePoint<number_of_dimensions> point{x, y, z};
            points.emplace_back(point);
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        KdTree<number_of_dimensions> kdtree(points, ExecutionType::PARALLEL);
        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "Elapsed time (construction): " << (t2 - t1).count() / 1e9 << " seconds" << std::endl;

        KDTreePoint<number_of_dimensions> random_point{dist(gen), dist(gen), dist(gen)};
        auto t3 = std::chrono::high_resolution_clock::now();
        auto nearest_point = kdtree.findNearestNeighbor(random_point);
        auto t4 = std::chrono::high_resolution_clock::now();
        std::cout << "Elapsed time (query): " << (t4 - t3).count() / 1e9 << " seconds" << std::endl;

        std::cout << "( ";
        for (const auto &coordiate : nearest_point.coordinates)
        {
            std::cout << coordiate << " ";
        }
        std::cout << ") \n";
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Exception: " << ex.what();
        return EXIT_FAILURE;
    }
    catch (...)
    {
        std::cerr << "Unknown exception." << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}