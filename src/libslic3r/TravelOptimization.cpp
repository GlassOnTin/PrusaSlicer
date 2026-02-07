///|/ Copyright (c) Prusa Research 2025
///|/ Copyright (c) preFlight / oozeBot, LLC 2025
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "TravelOptimization.hpp"
#include "Polyline.hpp" // For ThickPolyline

#include <algorithm>
#include <limits>

namespace Slic3r {
namespace TravelOptimization {

size_t nearest_vertex_index(const Points &points, const Point &target) {
    if (points.empty())
        return 0;

    size_t best_idx     = 0;
    double best_dist_sq = std::numeric_limits<double>::max();

    for (size_t i = 0; i < points.size(); ++i) {
        double dist_sq = distance_squared(points[i], target);
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_idx     = i;
        }
    }

    return best_idx;
}

size_t nearest_vertex_index_closed(const Points &points, const Point &target) {
    if (points.size() < 2)
        return 0;

    // For closed loops, don't consider the last point (it's the same as first)
    bool   is_closed    = (points.front() == points.back());
    size_t search_limit = is_closed ? points.size() - 1 : points.size();

    size_t best_idx     = 0;
    double best_dist_sq = std::numeric_limits<double>::max();

    for (size_t i = 0; i < search_limit; ++i) {
        double dist_sq = distance_squared(points[i], target);
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_idx     = i;
        }
    }

    return best_idx;
}

size_t rotate_polygon_to_nearest_vertex(Polygon &polygon, const Point &target) {
    if (polygon.points.size() < 3)
        return 0;

    size_t idx = nearest_vertex_index(polygon.points, target);

    if (idx > 0 && idx < polygon.points.size()) {
        std::rotate(polygon.points.begin(), polygon.points.begin() + idx, polygon.points.end());
    }

    return idx;
}

size_t rotate_thick_polyline_to_nearest_vertex(ThickPolyline &polyline, const Point &target) {
    if (polyline.points.size() < 3)
        return 0;

    // ThickPolyline must be closed (front == back) for rotation to work
    if (polyline.points.front() != polyline.points.back())
        return 0;

    // Also need matching widths for proper rotation
    if (polyline.width.front() != polyline.width.back())
        return 0;

    // Find nearest vertex among unique vertices (exclude closing point)
    size_t idx = nearest_vertex_index_closed(polyline.points, target);

    if (idx > 0) {
        polyline.start_at_index(static_cast<int>(idx));
    }

    return idx;
}

LoopVertexLocation find_nearest_vertex_in_loop(const ExtrusionLoop &loop, const Point &target) {
    LoopVertexLocation result;
    result.path_idx    = 0;
    result.vertex_idx  = 0;
    result.vertex      = Point(0, 0);
    result.distance_sq = std::numeric_limits<double>::max();

    if (loop.paths.empty())
        return result;

    for (size_t path_idx = 0; path_idx < loop.paths.size(); ++path_idx) {
        const Polyline &polyline = loop.paths[path_idx].polyline;

        for (size_t vert_idx = 0; vert_idx < polyline.points.size(); ++vert_idx) {
            // For the last path, skip the last point if it matches the first path's first point
            // (to avoid counting the closing vertex twice)
            if (path_idx == loop.paths.size() - 1 &&
                vert_idx == polyline.points.size() - 1 &&
                polyline.points.back() == loop.paths.front().polyline.points.front()) {
                continue;
            }

            double dist_sq = distance_squared(polyline.points[vert_idx], target);
            if (dist_sq < result.distance_sq) {
                result.path_idx    = path_idx;
                result.vertex_idx  = vert_idx;
                result.vertex      = polyline.points[vert_idx];
                result.distance_sq = dist_sq;
            }
        }
    }

    return result;
}

// =============================================================================
// GEOMETRY SIMPLIFICATION IMPLEMENTATION
// =============================================================================

bool is_collinear(const Point &a, const Point &b, const Point &c, double tolerance_sq) {
    // Cross product (a-b) x (c-b) gives 2x the signed area of triangle abc
    Vec2d ab = (a - b).cast<double>();
    Vec2d cb = (c - b).cast<double>();

    double cross    = ab.x() * cb.y() - ab.y() * cb.x();
    double cross_sq = cross * cross;

    double len_ab_sq = ab.squaredNorm();
    double len_cb_sq = cb.squaredNorm();

    // Very short edges - consider collinear
    if (len_ab_sq < 1.0 || len_cb_sq < 1.0)
        return true;

    // height^2 = cross^2 / |ac|^2
    // We want height^2 < tolerance_sq
    Vec2d  ac       = (a - c).cast<double>();
    double len_ac_sq = ac.squaredNorm();

    if (len_ac_sq < 1.0)
        return true;

    return cross_sq < tolerance_sq * len_ac_sq;
}

size_t remove_collinear_points(Points &points, bool is_closed, double tolerance_sq) {
    if (points.size() < 3)
        return 0;

    size_t removed = 0;

    if (is_closed) {
        size_t n = points.size();

        bool   has_closing_point = (n > 0 && points.front() == points.back());
        size_t unique_points     = has_closing_point ? n - 1 : n;
        if (unique_points < 4) // Triangle or smaller
            return 0;

        std::vector<bool> to_remove(n, false);

        for (size_t i = 0; i < n; ++i) {
            if (unique_points - removed <= 3)
                break;

            // Skip the closing point
            if (i == n - 1 && points[i] == points[0])
                continue;

            size_t prev = (i == 0) ? n - 2 : i - 1;
            size_t next = (i == n - 2) ? 0 : i + 1;
            if (next == n - 1 && points[next] == points[0])
                next = 0;

            if (is_collinear(points[prev], points[i], points[next], tolerance_sq)) {
                to_remove[i] = true;
                ++removed;
            }
        }

        if (removed > 0) {
            Points new_points;
            new_points.reserve(n - removed);
            for (size_t i = 0; i < n; ++i) {
                if (!to_remove[i])
                    new_points.push_back(points[i]);
            }
            if (!new_points.empty() && new_points.front() != new_points.back())
                new_points.push_back(new_points.front());
            points = std::move(new_points);
        }
    } else {
        std::vector<bool> to_remove(points.size(), false);

        for (size_t i = 1; i < points.size() - 1; ++i) {
            if (is_collinear(points[i - 1], points[i], points[i + 1], tolerance_sq)) {
                to_remove[i] = true;
                ++removed;
            }
        }

        if (removed > 0) {
            Points new_points;
            new_points.reserve(points.size() - removed);
            for (size_t i = 0; i < points.size(); ++i) {
                if (!to_remove[i])
                    new_points.push_back(points[i]);
            }
            points = std::move(new_points);
        }
    }

    return removed;
}

size_t remove_collinear_points(Polygon &polygon, double tolerance_sq) {
    return remove_collinear_points(polygon.points, true, tolerance_sq);
}

size_t remove_collinear_points(Polyline &polyline, double tolerance_sq) {
    bool is_closed = !polyline.points.empty() && polyline.points.front() == polyline.points.back();
    return remove_collinear_points(polyline.points, is_closed, tolerance_sq);
}

size_t remove_collinear_points(ThickPolyline &polyline, double tolerance_sq) {
    if (polyline.points.size() < 3)
        return 0;

    bool   is_closed = polyline.points.front() == polyline.points.back();
    size_t n         = polyline.points.size();

    if (is_closed && n < 5) // 4 points = triangle
        return 0;

    std::vector<bool> to_remove(n, false);
    size_t            removed = 0;

    if (is_closed) {
        size_t unique_points = n - 1;
        for (size_t i = 0; i < n - 1; ++i) {
            if (unique_points - removed <= 3)
                break;

            size_t prev = (i == 0) ? n - 2 : i - 1;
            size_t next = (i == n - 2) ? 0 : i + 1;

            if (is_collinear(polyline.points[prev], polyline.points[i], polyline.points[next], tolerance_sq)) {
                to_remove[i] = true;
                ++removed;
            }
        }
        if (to_remove[0])
            to_remove[n - 1] = true;
    } else {
        for (size_t i = 1; i < n - 1; ++i) {
            if (is_collinear(polyline.points[i - 1], polyline.points[i], polyline.points[i + 1], tolerance_sq)) {
                to_remove[i] = true;
                ++removed;
            }
        }
    }

    if (removed == 0)
        return 0;

    Points                 new_points;
    std::vector<coordf_t>  new_widths;
    new_points.reserve(n - removed + (is_closed ? 1 : 0));

    size_t last_kept = SIZE_MAX;
    for (size_t i = 0; i < n; ++i) {
        if (!to_remove[i]) {
            new_points.push_back(polyline.points[i]);

            if (last_kept != SIZE_MAX && !new_widths.empty()) {
                size_t seg_start = last_kept;
                size_t seg_end   = i;

                if (2 * seg_start + 1 < polyline.width.size()) {
                    new_widths.push_back(polyline.width[2 * seg_start]);
                }
                if (seg_end > 0 && 2 * (seg_end - 1) + 1 < polyline.width.size()) {
                    new_widths.push_back(polyline.width[2 * (seg_end - 1) + 1]);
                }
            }
            last_kept = i;
        }
    }

    if (is_closed && !new_points.empty() && new_points.front() != new_points.back()) {
        if (last_kept != SIZE_MAX && last_kept != 0) {
            if (2 * last_kept < polyline.width.size())
                new_widths.push_back(polyline.width[2 * last_kept]);
            if (!polyline.width.empty())
                new_widths.push_back(polyline.width.back());
        }
        new_points.push_back(new_points.front());
    }

    polyline.points = std::move(new_points);

    size_t expected_widths = polyline.points.empty() ? 0 : 2 * (polyline.points.size() - 1);
    if (new_widths.size() == expected_widths) {
        polyline.width = std::move(new_widths);
    }

    return removed;
}

} // namespace TravelOptimization
} // namespace Slic3r
