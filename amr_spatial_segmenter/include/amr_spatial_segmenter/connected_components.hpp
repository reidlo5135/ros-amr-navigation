#ifndef AMR_SPATIAL_SEGMENTER__CONNECTED_COMPONENTS_HPP_
#define AMR_SPATIAL_SEGMENTER__CONNECTED_COMPONENTS_HPP_

/**
 * @file connected_components.hpp
 * @brief Binary-grid connected component extraction.
 */

#include <vector>

#include "amr_spatial_segmenter/grid_types.hpp"

namespace amr::spatial_segmenter
{

/// @brief Extracts connected regions from a row-major binary mask.
class ConnectedComponents
{
public:
  /// @brief Return components from true cells in the mask.
  static std::vector<Component> extract(
    const std::vector<bool> & mask,
    int width,
    int height,
    int min_cells = 1);
};

}  // namespace amr::spatial_segmenter

#endif  // AMR_SPATIAL_SEGMENTER__CONNECTED_COMPONENTS_HPP_
