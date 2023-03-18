#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <memory>
#include <thread>
#include <vector>

namespace data_structure
{
enum class ExecutionType : std::uint8_t
{
    SEQUENTIAL = 0,
    PARALLEL = 1
};

// Finds maximum recursion depth for threaded building of KD-Tree
const static std::uint8_t DEFAULT_RECURSION_DEPTH =
    static_cast<std::uint8_t>(std::floor(std::log2(std::thread::hardware_concurrency())));

/// @brief Point type used in the construction and nearest neighbour search of the KDTree.
/// @tparam dimensions Number of dimensions in the point.
template <std::size_t dimensions> struct KDTreePoint
{
    std::array<double, dimensions> coordinates;
    double &operator[](std::size_t index)
    {
        return coordinates[index];
    }
    const double &operator[](std::size_t index) const
    {
        return coordinates[index];
    }
};

/// @brief K-Dimensional tree for nearest neighbour search.
template <std::size_t dimensions> class KdTree final
{
  public:
    using PointType = KDTreePoint<dimensions>;

    /// @brief Constructor of the KDTree object.
    /// @param points Points used for construction of the tree.
    /// @param execution_type Select execution type. Default - sequential.
    explicit KdTree(const std::vector<PointType> &points, ExecutionType execution_type = ExecutionType::SEQUENTIAL)
        : points_(points), root_(nullptr)
    {
        switch (execution_type)
        {
        case (ExecutionType::PARALLEL): {
            buildTreeParallel(root_, points_.begin(), points_.end(), 0, 0);
            break;
        }
        default: {
            buildTree(root_, points_.begin(), points_.end(), 0);
            break;
        }
        }
    }

  private:
    struct Node
    {
        PointType point;
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;

        Node(const PointType &pt) : point(pt), left(nullptr), right(nullptr)
        {
        }
    };

    using NodePtr = std::unique_ptr<Node>;
    using PointIter = typename std::vector<PointType>::iterator;

    std::vector<PointType> points_;
    NodePtr root_;

    /// @brief Compare two points along the specified axis.
    /// @param a First point.
    /// @param b Second point.
    /// @param axis Axis along which points will be compared against each other.
    /// @return Return True if the first point is less than the second point along the specified axis.
    inline static bool comparePoints(const PointType &a, const PointType &b, std::size_t axis) noexcept
    {
        return (a[axis] < b[axis]);
    }

    /// @brief Finds the pivot point between left and right.
    /// @param left Left iterator.
    /// @param right Right iterator.
    /// @param axis Axis along which the the pivot point is searched.
    /// @return Pivot point.
    inline static PointIter findPivotPoint(const PointIter &left, const PointIter &right, std::size_t axis)
    {
        PointIter pivot = left + (std::distance(left, right) / 2);
        std::nth_element(left, pivot, right,
                         [&](const PointType &a, const PointType &b) { return comparePoints(a, b, axis); });
        return pivot;
    }

    /// @brief Attempts to find the median point along the specified axis.
    /// @param begin Begin iterator.
    /// @param end End iterator.
    /// @param axis Axis along which the median point is searched.
    /// @return Median point along the specified axis.
    inline static PointIter findMedian(const PointIter &begin, const PointIter &end, std::size_t axis)
    {
        auto length = std::distance(begin, end);
        auto mid = begin + length / 2;

        PointIter left = begin;
        PointIter right = end;

        while (true)
        {
            PointIter pivot = findPivotPoint(left, right, axis);
            if (pivot == mid)
            {
                return pivot;
            }
            else if (pivot < mid)
            {
                left = pivot + 1;
            }
            else
            {
                right = pivot;
            }
        }
    }

    /// @brief Recursively builds KD-Tree using sequential algorithm.
    /// @param node Current tree node.
    /// @param begin Left point iterator.
    /// @param end Right point iterator.
    /// @param axis_depth Dimension along which to split the tree.
    void buildTree(NodePtr &node, PointIter begin, PointIter end, std::size_t axis_depth)
    {
        // If points are equal
        if (begin == end)
        {
            return;
        }

        // Select median point
        std::size_t axis = axis_depth % dimensions;
        PointIter mid = findMedian(begin, end, axis);

        // Update the node and continue building the tree
        node = std::make_unique<Node>(*mid);
        buildTree(node->left, begin, mid, axis_depth + 1);
        buildTree(node->right, mid + 1, end, axis_depth + 1);
    }

    /// @brief Recursively builds KD-Tree using parallel algorithm.
    /// @param node Current tree node.
    /// @param begin Left point iterator.
    /// @param end Right point iterator.
    /// @param axis_depth Dimension along which to split the tree.
    /// @param recursion_depth Current recursion depth until the algorithm switches to sequential build strategy.
    void buildTreeParallel(NodePtr &node, PointIter begin, PointIter end, std::size_t axis_depth,
                           std::size_t recursion_depth = 0U)
    {
        // Sequential build
        if (recursion_depth > DEFAULT_RECURSION_DEPTH)
        {
            buildTree(node, begin, end, axis_depth);
        }
        // Parallel build
        else
        {
            // If points are equal
            if (begin == end)
            {
                return;
            }

            // Select median point
            std::size_t axis = axis_depth % dimensions;
            PointIter mid = findMedian(begin, end, axis);

            // Update the node and continue building the tree
            node = std::make_unique<Node>(*mid);

            std::thread left_executor(&KdTree::buildTreeParallel, this, std::ref(node->left), begin, mid,
                                      axis_depth + 1, recursion_depth + 1);

            buildTreeParallel(node->right, mid + 1, end, axis_depth + 1, recursion_depth + 1);

            left_executor.join();
        }
    }
};
} // namespace data_structure

#endif // KDTREE_HPP