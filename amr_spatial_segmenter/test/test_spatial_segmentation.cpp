/**
 * @file test_spatial_segmentation.cpp
 * @brief Unit tests for OccupancyGrid spatial segmentation helpers.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <iterator>
#include <vector>

#include "amr_spatial_segmenter/connected_components.hpp"
#include "amr_spatial_segmenter/distance_map.hpp"
#include "amr_spatial_segmenter/occupancy_grid_view.hpp"
#include "amr_spatial_segmenter/segment_classifier.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid make_grid(const int width, const int height, const double resolution)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = static_cast<std::uint32_t>(width);
  grid.info.height = static_cast<std::uint32_t>(height);
  grid.info.resolution = static_cast<float>(resolution);
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width * height), 0);
  return grid;
}

amr::spatial_segmenter::SegmenterParams test_params()
{
  amr::spatial_segmenter::SegmenterParams params;
  params.footprint_radius_m = 0.0;
  params.safety_margin_m = 0.0;
  params.corridor_min_length_m = 1.0;
  params.corridor_min_width_m = 0.45;
  params.corridor_max_width_m = 2.2;
  params.corridor_detection_mode = "scanline_band";
  params.corridor_min_aspect_ratio = 2.2;
  params.corridor_min_free_ratio = 0.70;
  params.corridor_min_boundary_evidence_ratio = 0.35;
  params.corridor_axis_aligned_only = true;
  params.corridor_allow_diagonal = false;
  params.scanline_min_free_run_length_m = 0.8;
  params.scanline_min_rows_per_horizontal_band = 4;
  params.scanline_min_cols_per_vertical_band = 4;
  params.area_detection_mode = "cell_decomposition";
  params.area_min_area_m2 = 0.5;
  params.area_min_side_m = 0.6;
  params.area_min_width_m = 0.6;
  params.area_min_height_m = 0.6;
  params.area_min_confidence = 0.50;
  params.area_min_free_ratio = 0.70;
  params.area_max_aspect_ratio = 3.5;
  params.area_publish_local_windows = false;
  params.area_interior_validation_enabled = true;
  params.area_internal_barrier_enabled = true;
  params.area_split_on_internal_barrier = true;
  params.area_reject_tiny_ambiguous = true;
  params.cell_decomposition_enabled = true;
  params.door_min_width_m = 0.45;
  params.door_max_width_m = 1.2;
  params.door_min_confidence = 0.20;
  return params;
}

bool spans_x_range(
  const amr::spatial_segmenter::Segment & segment,
  const double min_x,
  const double max_x)
{
  return segment.bbox_min.x < min_x && segment.bbox_max.x > max_x;
}

void carve_free_rect(
  nav_msgs::msg::OccupancyGrid & grid,
  const int min_x,
  const int min_y,
  const int max_x,
  const int max_y)
{
  const int width = static_cast<int>(grid.info.width);
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      grid.data[static_cast<std::size_t>((y * width) + x)] = 0;
    }
  }
}

std::vector<amr::spatial_segmenter::Segment> open_areas(
  const amr::spatial_segmenter::SegmentationResult & result)
{
  std::vector<amr::spatial_segmenter::Segment> areas;
  std::copy_if(
    result.segments.begin(),
    result.segments.end(),
    std::back_inserter(areas),
    [](const auto & segment) {
      return segment.type == amr::spatial_segmenter::SegmentType::OpenArea;
    });
  return areas;
}

std::vector<amr::spatial_segmenter::Segment> corridors(
  const amr::spatial_segmenter::SegmentationResult & result)
{
  std::vector<amr::spatial_segmenter::Segment> corridor_segments;
  std::copy_if(
    result.segments.begin(),
    result.segments.end(),
    std::back_inserter(corridor_segments),
    [](const auto & segment) {
      return segment.type == amr::spatial_segmenter::SegmentType::Corridor;
    });
  return corridor_segments;
}

std::vector<bool> traversable_mask(const amr::spatial_segmenter::OccupancyGridView & view)
{
  std::vector<bool> mask(static_cast<std::size_t>(view.cell_count()), false);
  for (int index = 0; index < view.cell_count(); ++index) {
    mask[static_cast<std::size_t>(index)] = view.is_free(index);
  }
  return mask;
}

amr::spatial_segmenter::DistanceMap raw_obstacle_distance(
  const amr::spatial_segmenter::OccupancyGridView & view)
{
  amr::spatial_segmenter::DistanceMap distance_map;
  distance_map.compute(view.width(), view.height(), view.resolution(), view.raw_occupied_mask());
  return distance_map;
}

}  // namespace

TEST(OccupancyGridView, ClassifiesOccupancyValues)
{
  auto grid = make_grid(3, 1, 0.1);
  grid.data = {-1, 20, 50};
  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  ASSERT_TRUE(view.valid());
  EXPECT_TRUE(view.is_unknown(0));
  EXPECT_TRUE(view.is_free(1));
  EXPECT_TRUE(view.is_ambiguous(2));
  EXPECT_TRUE(view.is_occupied(2));
  EXPECT_FALSE(view.is_raw_occupied(2));
}

TEST(OccupancyGridView, RejectsInvalidMapSize)
{
  auto grid = make_grid(3, 3, 0.1);
  grid.data.pop_back();
  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  EXPECT_FALSE(view.valid());
}

TEST(ConnectedComponents, ExtractsSeparateRegions)
{
  std::vector<bool> mask(9, false);
  mask[0] = true;
  mask[1] = true;
  mask[8] = true;
  const auto components = amr::spatial_segmenter::ConnectedComponents::extract(mask, 3, 3);
  ASSERT_EQ(components.size(), 2U);
  EXPECT_EQ(components[0].cells.size(), 2U);
  EXPECT_EQ(components[1].cells.size(), 1U);
}

TEST(SegmentClassifier, DetectsSimpleCorridor)
{
  auto grid = make_grid(30, 8, 0.1);
  for (int x = 0; x < 30; ++x) {
    grid.data[static_cast<std::size_t>(x)] = 100;
    grid.data[static_cast<std::size_t>((6 * 30) + x)] = 100;
  }
  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(test_params());
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  const auto corridor = std::find_if(
    result.segments.begin(),
    result.segments.end(),
    [](const auto & segment) {
      return segment.type == amr::spatial_segmenter::SegmentType::Corridor;
    });
  ASSERT_NE(corridor, result.segments.end());
  EXPECT_GE(corridor->length_m, 1.0);
  EXPECT_EQ(corridor->centerline.size(), 2U);
  EXPECT_DOUBLE_EQ(corridor->centerline.front().y, corridor->centerline.back().y);
}

TEST(SegmentClassifier, RejectsDiagonalCorridor)
{
  auto grid = make_grid(30, 30, 0.1);
  grid.data.assign(static_cast<std::size_t>(30 * 30), 100);
  for (int x = 4; x < 26; ++x) {
    const int y = x;
    for (int offset = -2; offset <= 2; ++offset) {
      const int yy = y + offset;
      if (yy >= 0 && yy < 30) {
        grid.data[static_cast<std::size_t>((yy * 30) + x)] = 0;
      }
    }
  }

  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(test_params());
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  const auto corridor = std::find_if(
    result.segments.begin(),
    result.segments.end(),
    [](const auto & segment) {
      return segment.type == amr::spatial_segmenter::SegmentType::Corridor;
    });
  EXPECT_EQ(corridor, result.segments.end());
}

TEST(SegmentClassifier, DetectsDoorBottleneckOnAxisAlignedCorridor)
{
  auto grid = make_grid(42, 14, 0.1);
  grid.data.assign(static_cast<std::size_t>(42 * 14), 100);
  for (int x = 2; x < 40; ++x) {
    const int min_y = x >= 18 && x <= 23 ? 5 : 4;
    const int max_y = x >= 18 && x <= 23 ? 9 : 10;
    for (int y = min_y; y <= max_y; ++y) {
      grid.data[static_cast<std::size_t>((y * 42) + x)] = 0;
    }
  }

  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  auto params = test_params();
  params.corridor_detection_mode = "axis_aligned_scanline";
  params.area_detection_mode = "local_free_rectangles";
  params.area_publish_local_windows = true;
  const amr::spatial_segmenter::SegmentClassifier classifier(params);
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  const auto door = std::find_if(
    result.segments.begin(),
    result.segments.end(),
    [](const auto & segment) {
      return segment.type == amr::spatial_segmenter::SegmentType::DoorCandidate;
    });
  ASSERT_NE(door, result.segments.end());
  EXPECT_GE(door->width_m, 0.45);
  EXPECT_LE(door->width_m, 1.2);
}

TEST(SegmentClassifier, RejectMapSizedBbox)
{
  auto grid = make_grid(30, 30, 0.1);
  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(test_params());
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  EXPECT_TRUE(open_areas(result).empty());
  EXPECT_EQ(result.stats.raw_components, 1);
  EXPECT_EQ(result.stats.large_components, 1);
  EXPECT_EQ(result.stats.unresolved_components, 1);
  EXPECT_GE(result.stats.rejected_areas, 1);
}

TEST(SegmentClassifier, HorizontalBandMerge)
{
  auto grid = make_grid(70, 24, 0.1);
  grid.data.assign(static_cast<std::size_t>(70 * 24), 100);
  carve_free_rect(grid, 8, 9, 61, 13);

  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(test_params());
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  const auto corridor_segments = corridors(result);
  ASSERT_EQ(corridor_segments.size(), 1U);
  EXPECT_TRUE(open_areas(result).empty());
  EXPECT_EQ(result.stats.horizontal_bands, 1);
  EXPECT_EQ(result.stats.corridor_bands, 1);
  EXPECT_EQ(result.stats.local_windows, 0);
  EXPECT_GE(corridor_segments.front().length_m, 5.0);
  EXPECT_LE(corridor_segments.front().width_m, 0.6);
}

TEST(SegmentClassifier, RejectUnmergedLocalWindows)
{
  auto grid = make_grid(44, 44, 0.1);
  grid.data.assign(static_cast<std::size_t>(44 * 44), 100);
  carve_free_rect(grid, 12, 12, 31, 31);

  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(test_params());
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  const auto areas = open_areas(result);
  ASSERT_EQ(areas.size(), 1U);
  EXPECT_TRUE(corridors(result).empty());
  EXPECT_EQ(result.stats.local_windows, 0);
  EXPECT_GE(areas.front().area_m2, 3.5);
  EXPECT_LE(areas.front().area_m2, 4.1);
}

TEST(SegmentClassifier, RejectAreaWithInternalVerticalBarrier)
{
  auto grid = make_grid(60, 36, 0.1);
  grid.data.assign(static_cast<std::size_t>(60 * 36), 100);
  carve_free_rect(grid, 8, 8, 51, 27);
  for (int y = 8; y <= 27; ++y) {
    grid.data[static_cast<std::size_t>((y * 60) + 30)] = 100;
  }

  auto params = test_params();
  params.area_split_on_internal_barrier = false;
  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(params);
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  for (const auto & area : open_areas(result)) {
    EXPECT_FALSE(spans_x_range(area, 3.0, 3.1));
  }
}

TEST(SegmentClassifier, SplitAreaOnInternalBarrier)
{
  auto grid = make_grid(60, 36, 0.1);
  grid.data.assign(static_cast<std::size_t>(60 * 36), 100);
  carve_free_rect(grid, 8, 8, 51, 27);
  for (int y = 8; y <= 27; ++y) {
    grid.data[static_cast<std::size_t>((y * 60) + 30)] = 100;
  }

  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(test_params());
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  const auto areas = open_areas(result);
  ASSERT_GE(areas.size(), 1U);
  for (const auto & area : areas) {
    EXPECT_FALSE(spans_x_range(area, 3.0, 3.1));
  }
  EXPECT_GE(result.stats.split_cells, 1);
}

TEST(SegmentClassifier, RejectTinyAmbiguousArea)
{
  auto grid = make_grid(30, 30, 0.1);
  grid.data.assign(static_cast<std::size_t>(30 * 30), -1);
  carve_free_rect(grid, 12, 12, 17, 17);

  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(test_params());
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  EXPECT_TRUE(open_areas(result).empty());
}

TEST(SegmentClassifier, AcceptCoherentBoundedArea)
{
  auto grid = make_grid(44, 44, 0.1);
  grid.data.assign(static_cast<std::size_t>(44 * 44), 100);
  carve_free_rect(grid, 12, 12, 31, 31);

  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(test_params());
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  const auto areas = open_areas(result);
  ASSERT_EQ(areas.size(), 1U);
  EXPECT_GE(areas.front().confidence, 0.50);
  EXPECT_GE(areas.front().area_m2, 3.5);
}

TEST(SegmentClassifier, BoustrophedonCellDecompositionBasic)
{
  auto grid = make_grid(80, 44, 0.1);
  grid.data.assign(static_cast<std::size_t>(80 * 44), 100);
  carve_free_rect(grid, 8, 8, 28, 31);
  carve_free_rect(grid, 50, 8, 70, 31);
  carve_free_rect(grid, 29, 18, 49, 21);

  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(test_params());
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  for (const auto & area : open_areas(result)) {
    EXPECT_FALSE(area.bbox_min.x<2.9 && area.bbox_max.x>5.0);
  }
  EXPECT_GE(result.stats.raw_cells, 2);
}

TEST(SegmentClassifier, GreenLikeAreaPreference)
{
  auto grid = make_grid(70, 60, 0.1);
  grid.data.assign(static_cast<std::size_t>(70 * 60), 100);
  carve_free_rect(grid, 35, 32, 62, 53);
  carve_free_rect(grid, 8, 8, 14, 14);
  carve_free_rect(grid, 18, 10, 24, 15);

  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(test_params());
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  const auto areas = open_areas(result);
  ASSERT_FALSE(areas.empty());
  const auto best = std::max_element(
    areas.begin(),
    areas.end(),
    [](const auto & lhs, const auto & rhs) {
      return lhs.confidence < rhs.confidence;
    });
  ASSERT_NE(best, areas.end());
  EXPECT_GT(best->area_m2, 5.0);
  EXPECT_GT(best->bbox_min.x, 3.0);
  EXPECT_GT(best->bbox_min.y, 3.0);
}

TEST(SegmentClassifier, CorridorCenterlineFromBand)
{
  auto grid = make_grid(70, 24, 0.1);
  grid.data.assign(static_cast<std::size_t>(70 * 24), 100);
  carve_free_rect(grid, 8, 9, 61, 13);

  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(test_params());
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  const auto corridor_segments = corridors(result);
  ASSERT_EQ(corridor_segments.size(), 1U);
  ASSERT_EQ(corridor_segments.front().centerline.size(), 2U);
  EXPECT_DOUBLE_EQ(
    corridor_segments.front().centerline.front().y,
    corridor_segments.front().centerline.back().y);
  EXPECT_NEAR(corridor_segments.front().centerline.front().y, 1.15, 1.0e-6);
}

TEST(SegmentClassifier, NmsRemovesOverlappingBoxes)
{
  auto grid = make_grid(44, 44, 0.1);
  grid.data.assign(static_cast<std::size_t>(44 * 44), 100);
  carve_free_rect(grid, 12, 12, 31, 31);

  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(test_params());
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  EXPECT_EQ(result.stats.horizontal_bands, 1);
  EXPECT_EQ(result.stats.vertical_bands, 1);
  EXPECT_EQ(open_areas(result).size(), 1U);
}

TEST(SegmentClassifier, NoLocalWindowsInSemanticMarkers)
{
  auto grid = make_grid(70, 24, 0.1);
  grid.data.assign(static_cast<std::size_t>(70 * 24), 100);
  carve_free_rect(grid, 8, 9, 61, 13);

  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(test_params());
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  EXPECT_EQ(result.stats.local_windows, 0);
  EXPECT_EQ(result.segments.size(), 1U);
  EXPECT_EQ(corridors(result).size(), 1U);
  EXPECT_TRUE(open_areas(result).empty());
}

TEST(SegmentClassifier, NoAreaWhenOnlyHugeUnresolvedComponent)
{
  auto grid = make_grid(60, 60, 0.1);
  const amr::spatial_segmenter::OccupancyGridView view(grid, 25, 65);
  const amr::spatial_segmenter::SegmentClassifier classifier(test_params());
  const auto result = classifier.classify(
    view,
    view.raw_free_mask(),
    traversable_mask(view),
    raw_obstacle_distance(view));

  EXPECT_TRUE(open_areas(result).empty());
  EXPECT_EQ(result.stats.raw_components, 1);
  EXPECT_EQ(result.stats.large_components, 1);
  EXPECT_EQ(result.stats.unresolved_components, 1);
  EXPECT_EQ(result.stats.local_candidates, 0);
}
