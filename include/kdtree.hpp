#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <algorithm>   // std::sort, std::nth_element
#include <array>       // std::array
#include <chrono>      // std::chrono
#include <cmath>       // std::floor
#include <cstdint>     // std::size_t
#include <functional>  // std::ref
#include <future>      // std::async
#include <iostream>    // std::cout
#include <iterator>    // std::distance
#include <limits>      // std::numeric_limits
#include <stdexcept>   // std::runtime_error
#include <thread>      // std::thread
#include <type_traits> // std::enable_if_t
#include <vector>      // std::vector

namespace neighbour_search
{
const static auto DEFAULT_RECURSION_DEPTH =
    static_cast<std::uint32_t>(std::floor(std::log2(std::thread::hardware_concurrency())));

/// @brief Definition of the point struct
template <typename CoordinateType, std::size_t number_of_dimensions,
          typename = std::enable_if_t<(number_of_dimensions == 2) || (number_of_dimensions == 3)>>
using Point = std::array<CoordinateType, number_of_dimensions>;

template <typename CoordinateType, std::size_t number_of_dimensions> class KDTree final
{
    using PointType = Point<CoordinateType, number_of_dimensions>;
    using KDTreeType = KDTree<CoordinateType, number_of_dimensions>;

  public:
    KDTree &operator=(const KDTreeType &other) = delete;
    KDTree(const KDTreeType &other) = delete;
    KDTree &operator=(KDTreeType &&other) noexcept = default;
    KDTree(KDTreeType &&other) noexcept = default;
    KDTree() = delete;

    /// @brief Builds KDTree
    /// @param points List of points
    /// @param threaded Flag that specifies whether threaded execution should be enabled
    explicit KDTree(const std::vector<PointType> &points, bool threaded = true)
        : nodes_(points.begin(), points.end()), root_(nullptr)
    {
        if (threaded)
        {
            // Concurrent build
            root_ = buildTreeRecursivelyParallel(nodes_.begin(), nodes_.end(), 0UL);
        }
        else
        {
            // Sequential build
            root_ = buildTreeRecursively(nodes_.begin(), nodes_.end(), 0UL);
        }

        if (root_ == nullptr)
        {
            throw std::runtime_error("KDTree is empty.");
        }
    }

    /// @brief Find a single nearest neighbour
    /// @param target Point of interest
    /// @return Closest point to the target
    PointType findNearestNeighbour(const PointType &target) const
    {
        Node *nearest_node = nullptr;
        CoordinateType min_distance_squared = std::numeric_limits<CoordinateType>::max();

        findNearestNeighbourRecursively(root_, target, 0UL, min_distance_squared, nearest_node);

        return nearest_node->point;
    }

  private:
    /// @brief Node structure containing point, pointer to left and right subtrees
    struct Node final
    {
        explicit Node(const PointType &point) : point(point), left(nullptr), right(nullptr)
        {
        }
        PointType point;
        Node *left = nullptr;
        Node *right = nullptr;
    };
    Node *root_ = nullptr;
    std::vector<Node> nodes_;

    /// @brief Distance squared between two points
    /// @param p1 first point
    /// @param p2 second point
    /// @return distance squared between points
    constexpr inline CoordinateType distanceSquared(const PointType &p1, const PointType &p2) const noexcept
    {
        static_assert((number_of_dimensions == 2) || (number_of_dimensions == 3));
        if constexpr (number_of_dimensions == 2)
        {
            const auto dx = p1[0] - p2[0];
            const auto dy = p1[1] - p2[1];
            return (dx * dx) + (dy * dy);
        }
        else
        {
            const auto dx = p1[0] - p2[0];
            const auto dy = p1[1] - p2[1];
            const auto dz = p1[2] - p2[2];
            return (dx * dx) + (dy * dy) + (dz * dz);
        }
    }

    /// @brief Recursive sequential build of the KDTree
    /// @param begin begin iterator
    /// @param end end iterator
    /// @param index index between first and last
    /// @return root node
    Node *buildTreeRecursively(typename std::vector<Node>::iterator begin, typename std::vector<Node>::iterator end,
                               std::size_t index) const noexcept
    {
        if (begin >= end)
        {
            return nullptr;
        }

        auto middle = begin + std::distance(begin, end) / 2;

        std::nth_element(begin, middle, end, [&index](const Node &n1, const Node &n2) -> bool {
            return (n1.point[index] < n2.point[index]);
        });

        index = (index + 1) % number_of_dimensions;

        middle->left = buildTreeRecursively(begin, middle, index);
        middle->right = buildTreeRecursively(middle + 1, end, index);

        return &(*middle);
    }

    /// @brief Recursive parallel build of the KDTree
    /// @param begin begin iterator
    /// @param end end iterator
    /// @param index index between first and last
    /// @return root node
    Node *buildTreeRecursivelyParallel(typename std::vector<Node>::iterator begin,
                                       typename std::vector<Node>::iterator end, std::size_t index,
                                       std::uint32_t recursion_depth = 0U) const noexcept
    {
        if (recursion_depth > DEFAULT_RECURSION_DEPTH)
        {
            return buildTreeRecursively(begin, end, index);
        }
        else
        {
            if (begin >= end)
            {
                return nullptr;
            }

            auto middle = begin + std::distance(begin, end) / 2;

            std::nth_element(begin, middle, end, [&index](const Node &n1, const Node &n2) -> bool {
                return (n1.point[index] < n2.point[index]);
            });

            index = (index + 1) % number_of_dimensions;

            auto future = std::async(std::launch::async, &KDTreeType::buildTreeRecursivelyParallel, this,
                                     std::ref(begin), std::ref(middle), index, recursion_depth + 1);
            middle->right = buildTreeRecursivelyParallel(middle + 1, end, index, recursion_depth + 1);
            middle->left = future.get();

            return &(*middle);
        }
    }

    /// @brief Finds closest point to target
    /// @param node Pointer to the root node
    /// @param target Target point
    /// @param index Node index
    /// @param min_distance_squared Distance squared
    /// @param nearest_node Node pointer reference
    void findNearestNeighbourRecursively(const Node *node, const PointType &target, std::size_t index,
                                         CoordinateType &min_distance_squared, Node *&nearest_node) const noexcept
    {
        if (node == nullptr)
        {
            return;
        }

        const auto distance_squared = distanceSquared(target, node->point);
        if (distance_squared < min_distance_squared)
        {
            min_distance_squared = distance_squared;
            nearest_node = const_cast<Node *>(node);
        }

        const auto delta = node->point[index] - target[index];

        index = (index + 1) % number_of_dimensions;

        const bool is_delta_positive = (delta > 0);
        findNearestNeighbourRecursively(is_delta_positive ? node->left : node->right, target, index,
                                        min_distance_squared, nearest_node);

        if (delta * delta < min_distance_squared)
        {
            findNearestNeighbourRecursively(is_delta_positive ? node->right : node->left, target, index,
                                            min_distance_squared, nearest_node);
        }
    }
};
} // namespace neighbour_search

#endif // KDTREE_HPP