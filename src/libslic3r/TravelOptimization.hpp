#ifndef slic3r_TravelOptimization_hpp_
#define slic3r_TravelOptimization_hpp_

///|/ Copyright (c) Prusa Research 2025
///|/ Copyright (c) preFlight / oozeBot, LLC 2025
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "Point.hpp"
#include "Polygon.hpp"
#include "Polyline.hpp"
#include "ExtrusionEntity.hpp"

namespace Slic3r {

// Forward declarations
struct ThickPolyline;

namespace TravelOptimization {

// =============================================================================
// SNAP-TO-VERTEX API
// =============================================================================
//
// These functions find the VERTEX (not arbitrary point) closest to a target.
// This is critical for travel optimization:
//
// 1. Splitting loops at vertices avoids creating artificial points
//    - A square has 4 corners; splitting at 3 o'clock creates a 5th point
//    - Splitting at a corner keeps 4 points
//
// 2. Starting at the optimal vertex minimizes travel distance
//    - After finishing perimeters, start infill at the nearest corner
//    - Not at a fixed position like "rear" or "3 o'clock"
// =============================================================================

/// Find the index of the vertex closest to the target point.
/// O(n) where n is the number of vertices.
size_t nearest_vertex_index(const Points &points, const Point &target);

inline size_t nearest_vertex_index(const Polygon &polygon, const Point &target) {
    return nearest_vertex_index(polygon.points, target);
}

inline size_t nearest_vertex_index(const Polyline &polyline, const Point &target) {
    return nearest_vertex_index(polyline.points, target);
}

/// Find the index of the vertex closest to target, excluding the last point
/// if it's a closing point (same as first). Use this for closed loops.
size_t nearest_vertex_index_closed(const Points &points, const Point &target);

// =============================================================================
// LOOP ROTATION API
// =============================================================================

/// Rotate a closed polygon to start at the vertex nearest to target.
size_t rotate_polygon_to_nearest_vertex(Polygon &polygon, const Point &target);

/// Rotate a closed ThickPolyline to start at the vertex nearest to target.
size_t rotate_thick_polyline_to_nearest_vertex(ThickPolyline &polyline, const Point &target);

// =============================================================================
// EXTRUSION LOOP API
// =============================================================================

/// Result of finding the nearest vertex in an ExtrusionLoop
struct LoopVertexLocation {
    size_t path_idx;     ///< Index of the path containing the vertex
    size_t vertex_idx;   ///< Index of the vertex within that path's polyline
    Point  vertex;       ///< The actual vertex point
    double distance_sq;  ///< Squared distance to target

    bool valid() const { return distance_sq < std::numeric_limits<double>::max(); }
};

/// Find the vertex in an ExtrusionLoop closest to the target point.
LoopVertexLocation find_nearest_vertex_in_loop(const ExtrusionLoop &loop, const Point &target);

// =============================================================================
// DISTANCE UTILITIES
// =============================================================================

inline double distance_squared(const Point &a, const Point &b) {
    return (a - b).cast<double>().squaredNorm();
}

inline double distance(const Point &a, const Point &b) {
    return std::sqrt(distance_squared(a, b));
}

// =============================================================================
// GEOMETRY SIMPLIFICATION API
// =============================================================================
//
// Remove unnecessary collinear points from paths.
// Fewer points = fewer G-code segments = faster processing on the printer.
// =============================================================================

/// Check if three points are collinear within a tolerance.
bool is_collinear(const Point &a, const Point &b, const Point &c, double tolerance_sq = 1.0);

/// Remove collinear points from a polygon (in place).
size_t remove_collinear_points(Polygon &polygon, double tolerance_sq = 1.0);

/// Remove collinear points from a polyline (in place).
size_t remove_collinear_points(Polyline &polyline, double tolerance_sq = 1.0);

/// Remove collinear points from a ThickPolyline (in place).
size_t remove_collinear_points(ThickPolyline &polyline, double tolerance_sq = 1.0);

/// Remove collinear points from a vector of points (in place).
size_t remove_collinear_points(Points &points, bool is_closed, double tolerance_sq = 1.0);

} // namespace TravelOptimization
} // namespace Slic3r

#endif // slic3r_TravelOptimization_hpp_
