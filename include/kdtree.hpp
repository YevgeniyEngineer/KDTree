#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <algorithm>   // std::sort, std::nth_element
#include <array>       // std::array
#include <chrono>      // std::chrono
#include <cstdint>     // std::size_t
#include <iostream>    // std::cout
#include <stdexcept>   // std::runtime_error
#include <type_traits> // std::enable_if_t
#include <vector>      // std::vector

namespace neighbour_search
{
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

    explicit KDTree(const std::vector<PointType> &points, bool threaded = true)
        : nodes_(points.begin(), points.end()), root_(nullptr)
    {
        std::cout << "Number of nodes: " << nodes_.size() << std::endl;
        if (points.size() < 2)
        {
            throw std::runtime_error("KDTree expects at least 2 points.");
        }
        if (threaded)
        {
            // Concurrent build
        }
        else
        {
            // Sequential build
            root_ = buildTreeRecursively(0UL, nodes_.size(), 0UL);
        }
    }

    ~KDTree()
    {
        root_ = nullptr;
    }

  private:
    struct Node final
    {
        explicit Node(const PointType &point) : point(point), left(nullptr), right(nullptr)
        {
        }
        ~Node()
        {
            left = nullptr;
            right = nullptr;
        }
        PointType point;
        Node *left = nullptr;
        Node *right = nullptr;
    };
    Node *root_ = nullptr;
    std::vector<Node> nodes_;

    /// @brief Recursive sequential build of the KDTree
    /// @param begin first node index
    /// @param end last node index
    /// @param index index between first and last
    /// @return root node
    inline Node *buildTreeRecursively(const std::size_t begin, const std::size_t end, std::size_t index)
    {
        if (end <= begin)
        {
            return nullptr;
        }

        const std::size_t middle = begin + (end - begin) / 2;

        auto node_it = nodes_.begin();
        std::nth_element(node_it, node_it + middle, node_it + end, [&index](const Node &n1, const Node &n2) -> bool {
            return (n1.point[index] < n2.point[index]);
        });

        index = (index + 1) % number_of_dimensions;

        nodes_[middle].left = buildTreeRecursively(begin, middle, index);
        nodes_[middle].right = buildTreeRecursively(middle + 1, end, index);

        return &nodes_[middle];
    }
};
} // namespace neighbour_search

#endif // KDTREE_HPP