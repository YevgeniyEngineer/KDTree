#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <algorithm>   // std::sort, std::nth_element
#include <array>       // std::array
#include <cstdint>     // std::size_t
#include <memory>      // std::unique_ptr
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
    ~KDTree() noexcept
    {
        root_ = nullptr;
    }
    explicit KDTree(const std::vector<PointType> &points, bool threaded = true)
        : nodes_(points.begin(), points.end()), root_(nullptr)
    {
        if (threaded)
        {
            // Concurrent build
        }
        else
        {
            // Sequential build
        }
    }

  private:
    struct Node final
    {
        explicit Node(const PointType &point) : point(point), left(nullptr), right(nullptr)
        {
        }
        ~Node() noexcept
        {
            left = nullptr;
            right = nullptr;
        }
        PointType point;
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;
    };
    std::unique_ptr<Node> root_;
    std::vector<Node> nodes_;
};
} // namespace neighbour_search

#endif // KDTREE_HPP