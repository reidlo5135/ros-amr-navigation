#ifndef AMR_SPATIAL_SEGMENTER__DISTANCE_MAP_HPP_
#define AMR_SPATIAL_SEGMENTER__DISTANCE_MAP_HPP_

/**
 * @file distance_map.hpp
 * @brief Approximate Euclidean obstacle distance map for occupancy grids.
 */

#include <vector>

namespace amr::spatial_segmenter
{

/// @brief Multi-source Dijkstra/chamfer distance map seeded from blocked cells.
class DistanceMap
{
public:
  /// @brief Compute distances in meters from the supplied blocked-cell mask.
  void compute(int width, int height, double resolution, const std::vector<bool> & blocked);
  /// @brief Return distance in meters for a row-major index.
  double at(int index) const;
  /// @brief Return all distances.
  const std::vector<double> & data() const;
  /// @brief Return true when dimensions and data are usable.
  bool valid() const;

private:
  int width_{0};
  int height_{0};
  double resolution_{0.0};
  std::vector<double> distances_;
};

}  // namespace amr::spatial_segmenter

#endif  // AMR_SPATIAL_SEGMENTER__DISTANCE_MAP_HPP_
