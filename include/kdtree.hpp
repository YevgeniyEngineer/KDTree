#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
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
const static std::size_t DEFAULT_RECURSION_DEPTH =
    static_cast<std::size_t>(std::floor(std::log2(std::thread::hardware_concurrency())));

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

    PointType findNearestNeighbor(const PointType &query_point) const
    {
        if (!root_)
        {
            throw std::runtime_error("KDTree is empty. Build the tree before searching.");
        }
        return findNearestNeighborRecursively(root_, query_point, 0);
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
                           std::size_t recursion_depth)
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

    inline static bool isInvalidPoint(const PointType &point) noexcept
    {
        for (const auto &coordinate : point.coordinates)
        {
            if (coordinate == std::numeric_limits<double>::max())
            {
                return (true);
            }
        }
        return (false);
    }

    inline static double squaredDistance(const PointType &a, const PointType &b) noexcept
    {
        double sum = 0.0;
        for (std::size_t i = 0; i < dimensions; ++i)
        {
            double diff = a[i] - b[i];
            sum += diff * diff;
        }
        return sum;
    }

    PointType findNearestNeighborRecursively(const NodePtr &node, const PointType &query_point, size_t depth) const
    {
        if (!node)
        {
            return PointType{}; // Return an invalid point
        }

        size_t axis = depth % dimensions;
        bool search_left_subtree = query_point[axis] < node->point[axis];

        PointType current_best;
        if (search_left_subtree)
        {
            current_best = findNearestNeighborRecursively(node->left, query_point, depth + 1);
        }
        else
        {
            current_best = findNearestNeighborRecursively(node->right, query_point, depth + 1);
        }

        if (isInvalidPoint(current_best) ||
            squaredDistance(node->point, query_point) < squaredDistance(current_best, query_point))
        {
            current_best = node->point;
        }

        double axis_distance_squared =
            (query_point[axis] - node->point[axis]) * (query_point[axis] - node->point[axis]);
        if (axis_distance_squared < squaredDistance(current_best, query_point))
        {
            PointType oppositeBest = search_left_subtree
                                         ? findNearestNeighborRecursively(node->right, query_point, depth + 1)
                                         : findNearestNeighborRecursively(node->left, query_point, depth + 1);

            if (!isInvalidPoint(oppositeBest) &&
                squaredDistance(oppositeBest, query_point) < squaredDistance(current_best, query_point))
            {
                current_best = oppositeBest;
            }
        }

        return current_best;
    }
};
} // namespace data_structure

#endif // KDTREE_HPP