#include "WipeTower.hpp"

#include <cassert>
#include <iostream>
#include <limits>
#include <vector>
#include <numeric>
#include <sstream>
#include <cstdlib>
#include <iomanip>
#include "GCodeProcessor.hpp"
#include "../GCodeWriter.hpp"
#include "BoundingBox.hpp"
#include "ClipperUtils.hpp"
#include "LocalesUtils.hpp"
#include "Triangulation.hpp"
#include "../MixedNozzleConfig.hpp"
#include "../Slicing.hpp"


namespace Slic3r
{
float         flat_iron_speed                = 10.f * 60.f;
static const double wipe_tower_wall_infill_overlap = 0.0;
static constexpr double WIPE_TOWER_RESOLUTION = 0.1;
// Orca: SCALING_FACTOR is a runtime variable (large-printer switch), so this cannot be constexpr
#define WT_SIMPLIFY_TOLERANCE_SCALED (0.001 / SCALING_FACTOR)
static constexpr int    arc_fit_size = 20;
#define SCALED_WIPE_TOWER_RESOLUTION (WIPE_TOWER_RESOLUTION / SCALING_FACTOR)
enum class LimitFlow { None, LimitPrintFlow, LimitRammingFlow, LimitRammingFlowNC};//nc:nozzle change
static const std::map<float, float> nozzle_diameter_to_nozzle_change_width{{0.2f, 0.5f}, {0.4f, 1.0f}, {0.6f, 1.2f}, {0.8f, 1.4f}};

// Same convention as ConfigOptionVector::get_at(): an index past the end takes the first value
// (a single entry applies to every filament); an empty vector reads as zero.
static float filament_vector_value(const std::vector<double> &values, size_t filament_id)
{
    if (values.empty())
        return 0.f;
    return float(filament_id < values.size() ? values[filament_id] : values.front());
}

inline float align_round(float value, float base)
{
    return std::round(value / base) * base;
}

inline float align_ceil(float value, float base)
{
    return std::ceil(value / base) * base;
}

inline float align_floor(float value, float base)
{
    return std::floor((value) / base) * base;
}

static bool is_valid_gcode(const std::string &gcode)
{
    int  str_size    = gcode.size();
    int  start_index = 0;
    int  end_index   = 0;
    bool is_valid    = false;
    while (end_index < str_size) {
        if (gcode[end_index] != '\n') {
            end_index++;
            continue;
        }

        if (end_index > start_index) {
            std::string line_str = gcode.substr(start_index, end_index - start_index);
            line_str.erase(0, line_str.find_first_not_of(" "));
            line_str.erase(line_str.find_last_not_of(" ") + 1);
            if (!line_str.empty() && line_str[0] != ';') {
                is_valid = true;
                break;
            }
        }

        start_index = end_index + 1;
        end_index   = start_index;
    }

    return is_valid;
}

Polygon chamfer_polygon(Polygon &polygon, double chamfer_dis = 2., double angle_tol = 30. / 180. * PI)
{
    if (polygon.points.size() < 3) return polygon;
    Polygon res;
    res.points.reserve(polygon.points.size() * 2);
    int    mod           = polygon.points.size();
    double cos_angle_tol = abs(std::cos(angle_tol));

    for (int i = 0; i < polygon.points.size(); i++) {
        Vec2d  a        = unscaled(polygon.points[(i - 1 + mod) % mod]);
        Vec2d  b        = unscaled(polygon.points[i]);
        Vec2d  c        = unscaled(polygon.points[(i + 1) % mod]);
        double ab_len   = (a - b).norm();
        double bc_len   = (b - c).norm();
        Vec2d  ab       = (b - a) / ab_len;
        Vec2d  bc       = (c - b) / bc_len;
        assert(ab_len != 0);
        assert(bc_len != 0);
        float  cosangle = ab.dot(bc);
        //std::cout << " angle " << acos(cosangle) << " cosangle " << cosangle << std::endl;
        //std::cout << " ab_len " << ab_len << " bc_len " << bc_len << std::endl;
        if (abs(cosangle) < cos_angle_tol) {
            float real_chamfer_dis = std::min({chamfer_dis, ab_len / 2.1, bc_len / 2.1}); // 2.1 to ensure the points do not coincide
            Vec2d left             = b - ab * real_chamfer_dis;
            Vec2d right            = b + bc * real_chamfer_dis;
            res.points.push_back(scaled(left));
            res.points.push_back(scaled(right));
        } else
            res.points.push_back(polygon.points[i]);
    }
    res.points.shrink_to_fit();
    return res;
}

Polygon WipeTower::rounding_polygon(Polygon &polygon, double rounding /*= 2.*/, double angle_tol/* = 30. / 180. * PI*/)
{
    if (polygon.points.size() < 3) return polygon;
    Polygon res;
    res.points.reserve(polygon.points.size() * 2);
    int    mod           = polygon.points.size();
    double cos_angle_tol = abs(std::cos(angle_tol));

    for (int i = 0; i < polygon.points.size(); i++) {
        Vec2d  a      = unscaled(polygon.points[(i - 1 + mod) % mod]);
        Vec2d  b      = unscaled(polygon.points[i]);
        Vec2d  c      = unscaled(polygon.points[(i + 1) % mod]);
        double ab_len = (a - b).norm();
        double bc_len = (b - c).norm();
        Vec2d  ab     = (b - a) / ab_len;
        Vec2d  bc     = (c - b) / bc_len;
        assert(ab_len != 0);
        assert(bc_len != 0);
        float cosangle = ab.dot(bc);
        cosangle       = std::clamp(cosangle, -1.f, 1.f);
        bool  is_ccw   = cross2(ab, bc) > 0;
        if (abs(cosangle) < cos_angle_tol) {
            float real_rounding_dis = std::min({rounding, ab_len / 2.1, bc_len / 2.1}); // 2.1 to ensure the points do not coincide
            Vec2d left              = b - ab * real_rounding_dis;
            Vec2d right             = b + bc * real_rounding_dis;
            //Point r_left            = scaled(left);
            //Point r_right            = scaled(right);
           // std::cout << " r_left  " << r_left[0] << " " << r_left[1] << std::endl;
            //std::cout << " r_right  " << r_right[0] << " " << r_right[1] << std::endl;
            {
                float half_angle = std::acos(cosangle)/2.f;
                //std::cout << " half_angle  " << cos(half_angle) << std::endl;

                Vec2d dir        = (right - left).normalized();
                dir              = Vec2d{-dir[1], dir[0]};
                dir                  = is_ccw ? dir : -dir;
                double dis       = real_rounding_dis / sin(half_angle);
                //std::cout << " dis  " << dis << std::endl;

                Vec2d  center    = b + dir * dis;
                double radius    = (left - center).norm();
                ArcSegment arc(scaled(center), scaled(radius), scaled(left), scaled(right), is_ccw ? ArcDirection::Arc_Dir_CCW : ArcDirection::Arc_Dir_CW);
                int        n            = arc_fit_size;
                //std::cout << "start  " << arc.start_point[0] << " " << arc.start_point[1] << std::endl;
                //std::cout << "end  " << arc.end_point[0] << " " << arc.end_point[1] << std::endl;
                //std::cout << "start angle   " << arc.polar_start_theta << " end angle " << arc.polar_end_theta << std::endl;
                for (int j = 0; j < n; j++) {
                    float cur_angle = arc.polar_start_theta + (float)j/n * arc.angle_radians ;
                    //std::cout << " cur_angle " << cur_angle << std::endl;
                    if (cur_angle > 2 * PI)
                        cur_angle -= 2 * PI;
                    else if (cur_angle < 0)
                        cur_angle += 2 * PI;
                    Point tmp = arc.center + Point{arc.radius * std::cos(cur_angle), arc.radius *std::sin(cur_angle)};
                    //std::cout << "j = " << j << std::endl;
                    //std::cout << "tmp  = " << tmp[0]<<" "<<tmp[1] << std::endl;
                    res.points.push_back(tmp);
                }
            }
            res.points.push_back(scaled(right));
        } else
            res.points.push_back(polygon.points[i]);
    }
    res.remove_duplicate_points();
    res.points.shrink_to_fit();
    return res;
}

Polygon rounding_rectangle(Polygon &polygon, double rounding = 2., double angle_tol = 30. / 180. * PI) {
    if (polygon.points.size() < 3) return polygon;
    Polygon res;
    res.points.reserve(polygon.points.size() * 2);
    int    mod           = polygon.points.size();
    double cos_angle_tol = abs(std::cos(angle_tol));

    for (int i = 0; i < polygon.points.size(); i++) {
        Vec2d  a      = unscaled(polygon.points[(i - 1 + mod) % mod]);
        Vec2d  b      = unscaled(polygon.points[i]);
        Vec2d  c      = unscaled(polygon.points[(i + 1) % mod]);
        double ab_len = (a - b).norm();
        double bc_len = (b - c).norm();
        Vec2d  ab     = (b - a) / ab_len;
        Vec2d  bc     = (c - b) / bc_len;
        assert(ab_len != 0);
        assert(bc_len != 0);
        float cosangle = ab.dot(bc);
        cosangle       = std::clamp(cosangle, -1.f, 1.f);
        bool is_ccw    = cross2(ab, bc) > 0;
        if (abs(cosangle) < cos_angle_tol) {
            float real_rounding_dis = std::min({rounding, ab_len / 2.1, bc_len / 2.1}); // 2.1 to ensure the points do not coincide
            Vec2d left              = b - ab * real_rounding_dis;
            Vec2d right             = b + bc * real_rounding_dis;
            //Point r_left            = scaled(left);
            //Point r_right           = scaled(right);
            // std::cout << " r_left  " << r_left[0] << " " << r_left[1] << std::endl;
            // std::cout << " r_right  " << r_right[0] << " " << r_right[1] << std::endl;
            {
                Vec2d      center = b;
                double     radius = real_rounding_dis;
                ArcSegment arc(scaled(center), scaled(radius), scaled(left), scaled(right), is_ccw ? ArcDirection::Arc_Dir_CCW : ArcDirection::Arc_Dir_CW);
                int        n            = arc_fit_size;
                // std::cout << "start  " << arc.start_point[0] << " " << arc.start_point[1] << std::endl;
                // std::cout << "end  " << arc.end_point[0] << " " << arc.end_point[1] << std::endl;
                // std::cout << "start angle   " << arc.polar_start_theta << " end angle " << arc.polar_end_theta << std::endl;
                for (int j = 0; j < n; j++) {
                    float cur_angle = arc.polar_start_theta + (float) j / n * arc.angle_radians;
                    // std::cout << " cur_angle " << cur_angle << std::endl;
                    if (cur_angle > 2 * PI)
                        cur_angle -= 2 * PI;
                    else if (cur_angle < 0)
                        cur_angle += 2 * PI;
                    Point tmp = arc.center + Point{arc.radius * std::cos(cur_angle), arc.radius * std::sin(cur_angle)};
                    // std::cout << "j = " << j << std::endl;
                    // std::cout << "tmp  = " << tmp[0]<<" "<<tmp[1] << std::endl;
                    res.points.push_back(tmp);
                }
            }
            res.points.push_back(scaled(right));
        } else
            res.points.push_back(polygon.points[i]);
    }
    res.points.shrink_to_fit();
    return res;
}

std::pair<bool, Vec2f> ray_intersetion_line(const Vec2f &a, const Vec2f &v1, const Vec2f &b, const Vec2f &c)
{
    const Vec2f v2    = c - b;
    double      denom = cross2(v1, v2);
    if (fabs(denom) < EPSILON) return {false, Vec2f(0, 0)};
    const Vec2f v12    = (a - b);
    double      nume_a = cross2(v2, v12);
    double      nume_b = cross2(v1, v12);
    double      t1     = nume_a / denom;
    double      t2     = nume_b / denom;
    if (t1 >= 0 && t2 >= 0 && t2 <= 1.) {
        // Get the intersection point.
        Vec2f res = a + t1 * v1;
        return std::pair<bool, Vec2f>(true, res);
    }
    return std::pair<bool, Vec2f>(false, Vec2f{0, 0});
}
Polygon scale_polygon(const std::vector<Vec2f> &points) {
    Polygon res;
    for (const auto &p : points) res.points.push_back(scaled(p));
    return res;
}
std::vector<Vec2f> unscale_polygon(const Polygon& polygon)
{
    std::vector<Vec2f> res;
    for (const auto &p : polygon.points) res.push_back(unscaled<float>(p));
    return res;
}

Polygon generate_rectange(const Line &line, coord_t offset)
{
    Point p1 = line.a;
    Point p2 = line.b;

    double dx = p2.x() - p1.x();
    double dy = p2.y() - p1.y();

    double length = std::sqrt(dx * dx + dy * dy);

    double ux = dx / length;
    double uy = dy / length;

    double vx = -uy;
    double vy = ux;

    double ox = vx * offset;
    double oy = vy * offset;

    Points rect;
    rect.resize(4);
    rect[0] = {p1.x() + ox, p1.y() + oy};
    rect[1] = {p1.x() - ox, p1.y() - oy};
    rect[2] = {p2.x() - ox, p2.y() - oy};
    rect[3] = {p2.x() + ox, p2.y() + oy};
    Polygon poly(rect);
    return poly;
};

struct Segment
{
    Vec2f start;
    Vec2f end;
    bool  is_arc = false;
    ArcSegment arcsegment;
    Segment(const Vec2f &s, const Vec2f &e) : start(s), end(e) {}
    bool is_valid() const { return start.y() < end.y(); }
};

std::vector<Segment> remove_points_from_segment(const Segment &segment, const std::vector<Vec2f> &skip_points, double range)
{
    std::vector<Segment> result;
    result.push_back(segment);
    float x = segment.start.x();

    for (const Vec2f &point : skip_points) {
        std::vector<Segment> newResult;
        for (const auto &seg : result) {
            if (point.y() + range <= seg.start.y() || point.y() - range >= seg.end.y()) {
                newResult.push_back(seg);
            } else {
                if (point.y() - range > seg.start.y()) { newResult.push_back(Segment(Vec2f(x, seg.start.y()), Vec2f(x, point.y() - range))); }
                if (point.y() + range < seg.end.y()) { newResult.push_back(Segment(Vec2f(x, point.y() + range), Vec2f(x, seg.end.y()))); }
            }
        }

        result = newResult;
    }

    result.erase(std::remove_if(result.begin(), result.end(), [](const Segment &seg) { return !seg.is_valid(); }), result.end());
    return result;
}

struct IntersectionInfo
{
    Vec2f pos;
    int   idx;
    int   pair_idx; // gap_pair idx
    float dis_from_idx;
    bool  is_forward;
};

struct PointWithFlag
{
    Vec2f pos;
    int   pair_idx; // gap_pair idx
    bool  is_forward;
};
IntersectionInfo move_point_along_polygon(const std::vector<Vec2f> &points, const Vec2f &startPoint, int startIdx, float offset, bool forward, int pair_idx)
{
    float            remainingDistance = offset;
    IntersectionInfo res;
    int  mod = points.size();
    if (forward) {
        int next = (startIdx + 1) % mod;
        remainingDistance -= (points[next] - startPoint).norm();
        if (remainingDistance <= 0) {
            res.idx          = startIdx;
            res.pos          = startPoint + (points[next] - startPoint).normalized() * offset;
            res.pair_idx     = pair_idx;
            res.dis_from_idx = (points[startIdx] - res.pos).norm();
            return res;
        } else {
            for (int i = (startIdx + 1) % mod; i != startIdx; i = (i + 1) % mod) {
                float segmentLength = (points[(i + 1) % mod] - points[i]).norm();
                if (remainingDistance <= segmentLength) {
                    float ratio      = remainingDistance / segmentLength;
                    res.idx          = i;
                    res.pos          = points[i] + ratio * (points[(i + 1) % mod] - points[i]);
                    res.dis_from_idx = remainingDistance;
                    res.pair_idx     = pair_idx;
                    return res;
                }
                remainingDistance -= segmentLength;
            }
            res.idx          = (startIdx - 1 + mod) % mod;
            res.pos          = points[startIdx];
            res.pair_idx     = pair_idx;
            res.dis_from_idx = (res.pos - points[res.idx]).norm();
        }
    } else {
        int next = (startIdx + 1) % mod;
        remainingDistance -= (points[startIdx] - startPoint).norm();
        if (remainingDistance <= 0) {
            res.idx          = startIdx;
            res.pos          = startPoint - (points[next] - points[startIdx]).normalized() * offset;
            res.dis_from_idx = (res.pos - points[startIdx]).norm();
            res.pair_idx     = pair_idx;
            return res;
        }
        for (int i = (startIdx - 1 + mod) % mod; i != startIdx; i = (i - 1 + mod) % mod) {
            float segmentLength = (points[(i + 1) % mod] - points[i]).norm();
            if (remainingDistance <= segmentLength) {
                float ratio      = remainingDistance / segmentLength;
                res.idx          = i;
                res.pos          = points[(i + 1) % mod] - ratio * (points[(i + 1) % mod] - points[i]);
                res.dis_from_idx = segmentLength - remainingDistance;
                res.pair_idx     = pair_idx;
                return res;
            }
            remainingDistance -= segmentLength;
        }
        res.idx          = startIdx;
        res.pos          = points[res.idx];
        res.pair_idx     = pair_idx;
        res.dis_from_idx = 0;
    }
    return res;
};

void insert_points(std::vector<PointWithFlag> &pl, int idx, Vec2f pos, int pair_idx, bool is_forward)
{
    int   next = (idx + 1) % pl.size();
    Vec2f pos1 = pl[idx].pos;
    Vec2f pos2 = pl[next].pos;
    if ((pos - pos1).squaredNorm() < EPSILON) {
        pl[idx].pair_idx   = pair_idx;
        pl[idx].is_forward = is_forward;
    } else if ((pos - pos2).squaredNorm() < EPSILON) {
        pl[next].pair_idx   = pair_idx;
        pl[next].is_forward = is_forward;
    } else {
        pl.insert(pl.begin() + idx + 1, PointWithFlag{pos, pair_idx, is_forward});
    }
}

// For skip_point
// TODO: Optimize the skip_point algorithm itself instead of adding guards here
Polygon add_extra_point(const Polygon &polygon, int scale_range)
{
    Polygon res;
    if (polygon.size() < 2) return polygon;

    // Compute bounding box of the polygon
    auto polygon_box = get_extents(polygon);

    // Anchor point: X at bbox center, Y at bbox bottom
    Vec2f anchor_point(float(polygon_box.center()[0]), float(polygon_box.min[1]));

    // Find the edge whose midpoint is closest to the anchor point
    size_t closest_edge_idx = 0;
    float  min_dist_sq      = std::numeric_limits<float>::max();

    for (size_t i = 0; i < polygon.size(); ++i) {
        const Point &a_i = polygon[i];
        const Point &b_i = polygon[(i + 1) % polygon.size()];

        Vec2f a(float(a_i.x()), float(a_i.y()));
        Vec2f b(float(b_i.x()), float(b_i.y()));
        Vec2f mid = (a + b) * 0.5f;

        float dist_sq = (anchor_point - mid).squaredNorm();
        if (dist_sq < min_dist_sq) {
            min_dist_sq      = dist_sq;
            closest_edge_idx = i;
        }
    }

    // Edge endpoints (integer space)
    const Point &a_i = polygon[closest_edge_idx];
    const Point &b_i = polygon[(closest_edge_idx + 1) % polygon.size()];

    // Convert to float for geometric computation
    Vec2f a(float(a_i.x()), float(a_i.y()));
    Vec2f b(float(b_i.x()), float(b_i.y()));

    Vec2f mid = (a + b) * 0.5f;

    // Direction vectors from midpoint towards A and B
    Vec2f dir_to_a = a - mid;
    Vec2f dir_to_b = b - mid;

    float len_a = dir_to_a.norm();
    float len_b = dir_to_b.norm();

    // Guard against degenerated edges
    if (len_a < EPSILON || len_b < EPSILON) return polygon;

    dir_to_a /= len_a;
    dir_to_b /= len_b;

    // Clamp range to avoid overshooting the edge
    float max_range = std::min(len_a, len_b) * 0.9f;
    float range     = std::min(float(scale_range), max_range);

    // Offset points (float space)
    Vec2f offset_to_a_f = mid + dir_to_a * range;
    Vec2f offset_to_b_f = mid + dir_to_b * range;

    // Safe cast back to scaled integer Point
    auto to_int_point = [](const Vec2f &p) {
        auto clamp = [](float v) -> coord_t {
            constexpr float kMin = float(std::numeric_limits<coord_t>::min());
            constexpr float kMax = float(std::numeric_limits<coord_t>::max());
            v                    = std::clamp(v, kMin, kMax);
            return static_cast<coord_t>(std::lround(v));
        };
        return Point(clamp(p.x()), clamp(p.y()));
    };

    Point mid_i         = to_int_point(mid);
    Point offset_to_a_i = to_int_point(offset_to_a_f);
    Point offset_to_b_i = to_int_point(offset_to_b_f);

    // Rebuild polygon with inserted points
    for (size_t i = 0; i < polygon.size(); ++i) {
        res.points.push_back(polygon[i]);

        // Insert points right after the selected edge start vertex
        if (i == closest_edge_idx) {
            res.points.push_back(offset_to_a_i);
            res.points.push_back(mid_i);
            res.points.push_back(offset_to_b_i);
        }
    }

    return res;
}



Polylines remove_points_from_polygon(const Polygon &polygon_ori, const std::vector<Vec2f> &skip_points, double range, float wt_width, Polygon &insert_skip_pg)
{
    Polygon polygon = add_extra_point(polygon_ori, scale_(range));
    if (polygon.size() < 2) return Polylines{to_polyline(polygon)};
    Polylines                     result;
    std::vector<PointWithFlag>    new_pl; // add intersection points for gaps, where bool indicates whether it's a gap point.
    std::vector<IntersectionInfo> inter_info;
    auto                          polygon_box  = get_extents(polygon);
    //Point                         anchor_point = /*is_left ? Point{polygon_box.max[0], polygon_box.min[1]} :*/ polygon_box.min; // rd:ld
    Point              anchor_point = Point{polygon_box.center()[0], polygon_box.min[1]}; // for next reconnect
    std::vector<Vec2f>            points;
    {
        points.reserve(polygon.points.size());
        // Like closest_point_index(), but exact ties (the ring is symmetric about the anchor) are
        // broken by coordinates so the result does not depend on the ring's vertex order.
        int idx = -1;
        double nearest = std::numeric_limits<double>::max();
        for (size_t at = 0; at < polygon.points.size(); ++ at) {
            const Point &here = polygon.points[at];
            const double distance = (here - anchor_point).cast<double>().squaredNorm();
            bool take = idx < 0 || distance < nearest;
            if (!take && distance == nearest) {
                const Point &held = polygon.points[idx];
                take = here.x() < held.x() || (here.x() == held.x() && here.y() < held.y());
            }
            if (take) {
                nearest = distance;
                idx     = int(at);
            }
        }
        Polyline tmp_poly = polygon.split_at_index(idx);
        for (auto &p : tmp_poly) points.push_back(unscale(p).cast<float>());
        points.pop_back();
    }

    for (int i = 0; i < skip_points.size(); i++) {
        bool  is_left = abs(skip_points[i].x()) < wt_width / 2.f;
        Vec2f ray = is_left ? Vec2f(-1, 0) : Vec2f(1, 0);
        for (int j = 0; j < points.size(); j++) {
            Vec2f& p1                   = points[j];
            Vec2f& p2                   = points[(j + 1) % points.size()];
            auto [is_inter, inter_pos] = ray_intersetion_line(skip_points[i], ray, p1, p2);
            if (is_inter) {
                IntersectionInfo forward  = move_point_along_polygon(points, inter_pos, j, range, true, i);
                IntersectionInfo backward = move_point_along_polygon(points, inter_pos, j, range, false, i);
                backward.is_forward       = false;
                forward.is_forward        = true;
                inter_info.push_back(backward);
                inter_info.push_back(forward);
                break;
            }
        }
    }

    // insert point to new_pl
    for (const auto &p : points) new_pl.push_back({p, -1});
    std::sort(inter_info.begin(), inter_info.end(), [](const IntersectionInfo &lhs, const IntersectionInfo &rhs) {
        if (rhs.idx == lhs.idx) return lhs.dis_from_idx < rhs.dis_from_idx;
        return lhs.idx < rhs.idx;
    });
    for (int i = inter_info.size() - 1; i >= 0; i--) { insert_points(new_pl, inter_info[i].idx, inter_info[i].pos, inter_info[i].pair_idx, inter_info[i].is_forward); }

    {
        //set insert_pg for wipe_path
        for (auto &p : new_pl) insert_skip_pg.points.push_back(scaled(p.pos));
    }

    int beg = 0;
    bool skip = true;
    int  i    = beg;
    Polyline pl;

    do {
        if (skip || new_pl[i].pair_idx == -1) {
            pl.points.push_back(scaled(new_pl[i].pos));
            i    = (i + 1) % new_pl.size();
            skip = false;
        } else {
            if (!pl.points.empty()) {
                pl.points.push_back(scaled(new_pl[i].pos));
                result.push_back(pl);
                pl.points.clear();
            }
            int left = new_pl[i].pair_idx;
            int j    = (i + 1) % new_pl.size();
            while (j != beg && new_pl[j].pair_idx != left) {
                if (new_pl[j].pair_idx != -1 && !new_pl[j].is_forward) left = new_pl[j].pair_idx;
                j = (j + 1) % new_pl.size();
            }
            i    = j;
            skip = true;
        }
    } while (i != beg);

    if (!pl.points.empty()) {
        if (new_pl[i].pair_idx==-1) pl.points.push_back(scaled(new_pl[i].pos));
        result.push_back(pl);
    }
    return result;
}

Polylines construct_gap_for_skip_points(const Polygon &polygon, const std::vector<Vec2f> & skip_points ,float wt_width,float gap_length,Polygon& insert_skip_polygon)
{
    if (skip_points.empty()) {
        insert_skip_polygon = polygon;
        return Polylines{to_polyline(polygon)};
    }
    //bool is_left  = false;
    //const auto &pt      = skip_points.front();
    //if (abs(pt.x()) < wt_width/2.f) {
    //    is_left = true;
    //}
    return remove_points_from_polygon(polygon, skip_points, gap_length, wt_width, insert_skip_polygon);

};

Polygon generate_rectange_polygon(const Vec2f &wt_box_min ,const Vec2f & wt_box_max) {
    Polygon res;
    res.points.push_back(scaled(wt_box_min));
    res.points.push_back(scaled(Vec2f{wt_box_max[0], wt_box_min[1]}));
    res.points.push_back(scaled(wt_box_max));
    res.points.push_back(scaled(Vec2f{wt_box_min[0], wt_box_max[1]}));
    return res;
}

const char* flush_planner_queue_command(GCodeFlavor flavor)
{
    return flavor == gcfKlipper ? "M400\n" : "G4 S0\n";
}

std::string wait_command(GCodeFlavor flavor, float seconds)
{
    if (flavor == gcfKlipper)
        return "G4 P" + std::to_string(std::lround(seconds * 1000.f)) + "\n";
    return "G4 S" + Slic3r::float_to_string_decimal_point(seconds, 3) + "\n";
}

class WipeTowerWriter
{
public:
	// travel_speed (mm/s) is what travel() uses when called with f=0.
	// emit_block_z makes set_z() emit the block's own Z (lagging tower only).
	WipeTowerWriter(float layer_height, float line_width, GCodeFlavor flavor, const std::vector<WipeTower::FilamentParameters>& filament_parameters, bool enable_arc_fitting, float travel_speed = 0.f, bool emit_block_z = false) :
		m_current_z(0.f),
		m_current_feedrate(0.f),
		m_layer_height(layer_height),
		m_travel_speed(travel_speed),
		m_extrusion_flow(0.f),
		m_preview_suppressed(false),
		m_elapsed_time(0.f),
#if ENABLE_GCODE_VIEWER_DATA_CHECKING
        m_default_analyzer_line_width(line_width),
#endif // ENABLE_GCODE_VIEWER_DATA_CHECKING
        m_gcode_flavor(flavor),
        m_enable_arc_fitting(enable_arc_fitting),
        m_filpar(filament_parameters),
        m_emit_block_z(emit_block_z)
        {
            // ORCA: This class is only used by BBL printers, so set the parameter appropriately.
            // This fixes an issue where the wipe tower was using BBL tags resulting in statistics for purging in the purge tower not being displayed.
            GCodeProcessor::s_IsBBLPrinter = true;
            // adds tag for analyzer:
            std::ostringstream str;
            str << ";" << GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) << std::to_string(m_layer_height) << "\n"; // don't rely on GCodeAnalyzer knowing the layer height - it knows nothing at priming
            str << ";" << GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role) << ExtrusionEntity::role_to_string(erWipeTower) << "\n";
            m_gcode += str.str();
            change_analyzer_line_width(line_width);
    }

    // Open a tower bracket and repeat the height tag inside it: a compact tower block can be taller
    // than the part layer, and a reader starting at the marker would not see the earlier tag.
    WipeTowerWriter& append_wipe_tower_start() {
        m_gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_Start) + "\n";
        m_gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + std::to_string(m_layer_height) + "\n";
        return *this;
    }

    WipeTowerWriter& change_analyzer_line_width(float line_width) {
        // adds tag for analyzer:
        std::stringstream str;
        str << ";" << GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Width) << std::to_string(line_width) << "\n";
        m_gcode += str.str();
        return *this;
    }

#if ENABLE_GCODE_VIEWER_DATA_CHECKING
    WipeTowerWriter& change_analyzer_mm3_per_mm(float len, float e) {
        static const float area = float(M_PI) * 1.75f * 1.75f / 4.f;
        float mm3_per_mm = (len == 0.f ? 0.f : area * e / len);
        // adds tag for processor:
        std::stringstream str;
        str << ";" << GCodeProcessor::Mm3_Per_Mm_Tag << mm3_per_mm << "\n";
        m_gcode += str.str();
        return *this;
    }
#endif // ENABLE_GCODE_VIEWER_DATA_CHECKING

	WipeTowerWriter& 			 set_initial_position(const Vec2f &pos, float width = 0.f, float depth = 0.f, float internal_angle = 0.f) {
        m_wipe_tower_width = width;
        m_wipe_tower_depth = depth;
        m_internal_angle = internal_angle;
		m_start_pos = this->rotate(pos);
		m_current_pos = pos;
		return *this;
	}

    WipeTowerWriter&				 set_initial_tool(size_t tool) { m_current_tool = tool; return *this; }

	WipeTowerWriter&				 set_z(float z)
	{
		m_current_z = z;
		// A lagging visit stacks blocks at several Z inside one tool change, so each block
		// emits its own height.
		if (m_emit_block_z) {
			m_gcode += std::string("G1") + set_format_Z(z);
			if (m_travel_speed > 0.f) {
				m_gcode += set_format_F(m_travel_speed * 60.f);
				m_feed_from_travel = true;
			}
			m_gcode += "\n";
		}
		return *this;
	}

	// Move to z and always emit it, for a block that sits off its level's height (for example
	// ramming held within the outgoing tool's layer-height limits).
	WipeTowerWriter&				 move_z(float z)
	{
		m_current_z = z;
		m_gcode += std::string("G1") + set_format_Z(z);
		if (m_travel_speed > 0.f) {
			m_gcode += set_format_F(m_travel_speed * 60.f);
			m_feed_from_travel = true;
		}
		m_gcode += "\n";
		return *this;
	}

	WipeTowerWriter& 			 set_extrusion_flow(float flow)
		{ m_extrusion_flow = flow; return *this; }

	WipeTowerWriter&				 set_y_shift(float shift) {
        m_current_pos.y() -= shift-m_y_shift;
        m_y_shift = shift;
        return (*this);
    }

    WipeTowerWriter&            disable_linear_advance() {
        if (m_gcode_flavor == gcfKlipper)
            m_gcode += "SET_PRESSURE_ADVANCE ADVANCE=0\n";
        else if (m_gcode_flavor == gcfRepRapFirmware)
            m_gcode += std::string("M572 D") + std::to_string(m_current_tool) + " S0\n";
        else
            m_gcode += "M900 K0\n";

        return *this;
    }

	// Suppress / resume G-code preview in Slic3r. Slic3r will have difficulty to differentiate the various
	// filament loading and cooling moves from normal extrusion moves. Therefore the writer
	// is asked to suppres output of some lines, which look like extrusions.
#if ENABLE_GCODE_VIEWER_DATA_CHECKING
    WipeTowerWriter& suppress_preview() { change_analyzer_line_width(0.f); m_preview_suppressed = true; return *this; }
    WipeTowerWriter& resume_preview() { change_analyzer_line_width(m_default_analyzer_line_width); m_preview_suppressed = false; return *this; }
#else
    WipeTowerWriter& 			 suppress_preview() { m_preview_suppressed = true; return *this; }
	WipeTowerWriter& 			 resume_preview()   { m_preview_suppressed = false; return *this; }
#endif // ENABLE_GCODE_VIEWER_DATA_CHECKING

	WipeTowerWriter& 			 feedrate(float f)
	{
        if (f != m_current_feedrate) {
			m_gcode += "G1" + set_format_F(f) + "\n";
            m_current_feedrate = f;
        }
        m_print_feedrate    = f;
        m_feed_from_travel  = false;
		return *this;
	}

	const std::string&   gcode() const { return m_gcode; }
	const std::vector<WipeTower::Extrusion>& extrusions() const { return m_extrusions; }
	float                x()     const { return m_current_pos.x(); }
	float                y()     const { return m_current_pos.y(); }
	const Vec2f& 		 pos()   const { return m_current_pos; }
	const Vec2f	 		 start_pos_rotated() const { return m_start_pos; }
	const Vec2f  		 pos_rotated() const { return this->rotate(m_current_pos); }
	float 				 elapsed_time() const { return m_elapsed_time; }
    float                get_and_reset_used_filament_length() { float temp = m_used_filament_length; m_used_filament_length = 0.f; return temp; }

    // Orca: the feedrate this move may run at without exceeding the tool's volumetric ceiling
    // (max_e_speed, FLT_MAX when unset). Evaluated against the effective feedrate, so a move that
    // reuses the current F is limited too; a reduced value makes the caller emit a new F.
    float feedrate_within_volumetric_ceiling(float f, float e, float len, LimitFlow limit_flow) const
    {
        if (limit_flow == LimitFlow::None || e <= 0.f)
            return f;
        const float effective = (f != 0.f) ? f : m_current_feedrate;
        if (effective <= 0.f)
            return f;
        float ceiling = m_filpar[m_current_tool].max_e_speed;
        if (limit_flow == LimitFlow::LimitRammingFlow)
            ceiling = m_filpar[m_current_tool].max_e_ramming_speed.first;
        else if (limit_flow == LimitFlow::LimitRammingFlowNC)
            ceiling = m_filpar[m_current_tool].max_e_ramming_speed.second;
        if (! (ceiling > 0.f))
            return f;
        const float travelled = (len == 0.f) ? std::abs(e) : len;
        if (travelled <= 0.f)
            return f;
        const float e_speed = e / (travelled / effective * 60.f);
        const float limited = effective / std::max(1.f, e_speed / ceiling);
        if (! (limited < effective))
            return f;
        // set_format_F rounds to nearest, which could exceed the ceiling, so floor here. Never
        // return 0: that means "no feedrate named" to this writer.
        return std::max(1.f, std::floor(limited));
    }

    // Feedrate for a move that names none before its block has set one, so it does not inherit
    // the part's last speed. Travels use the tower travel speed; extrusions are also capped by the
    // tower print cap and the volumetric ceiling. With no travel speed a slow explicit rate is used.
    static constexpr float first_move_feedrate_floor = 1800.f; // mm/min, 30 mm/s
    static constexpr float first_move_extrusion_cap  = 5400.f; // mm/min, the tower's own cap
    float feedrate_for_first_move(float e, float len, LimitFlow limit_flow) const
    {
        const float travel = m_travel_speed > 0.f ? m_travel_speed * 60.f : first_move_feedrate_floor;
        if (e <= 0.f)
            return travel;
        const float capped = std::min(travel, first_move_extrusion_cap);
        float limited = feedrate_within_volumetric_ceiling(capped, e, len, limit_flow);
        if (! (limited > 0.f))
            limited = capped;
        // A ramming class with an unusable zero ceiling still answers to the print ceiling.
        const float by_print = feedrate_within_volumetric_ceiling(limited, e, len, LimitFlow::LimitPrintFlow);
        return by_print > 0.f ? by_print : limited;
    }

    // An extrusion that names no feedrate runs at the last extrusion feedrate, never at the speed of
    // an in-tower travel or Z move that came between (the tower's own travel speed, up to F60000).
    // Before any extrusion feedrate is known it takes the capped first-move feedrate.
    float feedrate_after_travel(float f, float e, float len, LimitFlow limit_flow) const
    {
        if (f != 0.f || e <= 0.f || ! m_feed_from_travel)
            return f;
        return m_print_feedrate > 0.f ? m_print_feedrate : feedrate_for_first_move(e, len, limit_flow);
    }

    // Extrude with an explicitely provided amount of extrusion.
    WipeTowerWriter &extrude_explicit(float x, float y, float e, float f = 0.f, bool record_length = false ,LimitFlow limit_flow = LimitFlow::LimitPrintFlow)
	{
        if ((std::abs(x - m_current_pos.x()) <= (float)EPSILON) && (std::abs(y - m_current_pos.y()) < (float)EPSILON) && e == 0.f && (f == 0.f || f == m_current_feedrate))
			// Neither extrusion nor a travel move.
			return *this;

		float dx = x - m_current_pos.x();
		float dy = y - m_current_pos.y();
        float len = std::sqrt(dx*dx+dy*dy);
        if (record_length)
            m_used_filament_length += e;

		// Now do the "internal rotation" with respect to the wipe tower center
		Vec2f rotated_current_pos(this->pos_rotated());
		Vec2f rot(this->rotate(Vec2f(x,y)));                               // this is where we want to go

        if (! m_preview_suppressed && e > 0.f && len > 0.f) {
#if ENABLE_GCODE_VIEWER_DATA_CHECKING
            change_analyzer_mm3_per_mm(len, e);
#endif // ENABLE_GCODE_VIEWER_DATA_CHECKING
            // Width of a squished extrusion, corrected for the roundings of the squished extrusions.
			// This is left zero if it is a travel move.
            float width = e * m_filpar[0].filament_area / (len * m_layer_height);
			// Correct for the roundings of a squished extrusion.
			width += m_layer_height * float(1. - M_PI / 4.);
			if (m_extrusions.empty() || m_extrusions.back().pos != rotated_current_pos)
				m_extrusions.emplace_back(WipeTower::Extrusion(rotated_current_pos, 0, m_current_tool));
			m_extrusions.emplace_back(WipeTower::Extrusion(rot, width, m_current_tool));
		}


        if (e == 0.f) {
            m_gcode += set_travel_acceleration();
        } else {
            m_gcode += set_normal_acceleration();
        }

		m_gcode += "G1";
        const bool carries_x = std::abs(rot.x() - rotated_current_pos.x()) > (float)EPSILON;
        const bool carries_y = std::abs(rot.y() - rotated_current_pos.y()) > (float)EPSILON;
        if (carries_x)
			m_gcode += set_format_X(rot.x());

        if (carries_y)
			m_gcode += set_format_Y(rot.y());


		if (e != 0.f)
			m_gcode += set_format_E(e);

        f = feedrate_after_travel(f, e, len, limit_flow);
        // Upstream limits only moves that name a new feedrate; Off mode keeps that.
        if (m_mixed_nozzle_slicing || (f != 0.f && f != m_current_feedrate))
            f = feedrate_within_volumetric_ceiling(f, e, len, limit_flow);
        // An XY move before the block has named any feedrate gets one of its own.
        if (f == 0.f && m_current_feedrate <= 0.f && (carries_x || carries_y)) {
            f = feedrate_for_first_move(e, len, limit_flow);
            // A travel that takes the tower travel speed this way does not set the print feedrate.
            if (e == 0.f)
                m_feed_from_travel = true;
        }
		if (f != 0.f && f != m_current_feedrate) {
			m_gcode += set_format_F(f);
        }
        if (e > 0.f) {
            m_print_feedrate   = m_current_feedrate;
            m_feed_from_travel = false;
        }

        m_current_pos.x() = x;
        m_current_pos.y() = y;

		// Update the elapsed time with a rough estimate. A move with no feedrate in force yet
		// counts as taking no time rather than dividing by zero.
		if (m_current_feedrate > 0.f)
			m_elapsed_time += ((len == 0.f) ? std::abs(e) : len) / m_current_feedrate * 60.f;
		m_gcode += "\n";
		return *this;
	}

    	// Extrude with an explicitely provided amount of extrusion.
    WipeTowerWriter &extrude_arc_explicit(ArcSegment &arc, float f = 0.f, bool record_length = false, LimitFlow limit_flow = LimitFlow::LimitPrintFlow)
    {
        float x   = (float)unscale(arc.end_point).x();
        float y   = (float)unscale(arc.end_point).y();
        float len = unscaled<float>(arc.length);
        float e   = len * m_extrusion_flow;
        if (len < (float) EPSILON && e == 0.f && (f == 0.f || f == m_current_feedrate))
            // Neither extrusion nor a travel move.
            return *this;
        if (record_length) m_used_filament_length += e;

        // Now do the "internal rotation" with respect to the wipe tower center
        Vec2f rotated_current_pos(this->pos_rotated());
        Vec2f rot(this->rotate(Vec2f(x, y))); // this is where we want to go

        if (!m_preview_suppressed && e > 0.f && len > 0.f) {
#if ENABLE_GCODE_VIEWER_DATA_CHECKING
            change_analyzer_mm3_per_mm(len, e);
#endif // ENABLE_GCODE_VIEWER_DATA_CHECKING
       // Width of a squished extrusion, corrected for the roundings of the squished extrusions.
       // This is left zero if it is a travel move.
            float width = e * m_filpar[0].filament_area / (len * m_layer_height);
            // Correct for the roundings of a squished extrusion.
            width += m_layer_height * float(1. - M_PI / 4.);
            if (m_extrusions.empty() || m_extrusions.back().pos != rotated_current_pos) m_extrusions.emplace_back(WipeTower::Extrusion(rotated_current_pos, 0, m_current_tool));
            {
                int   n            = arc_fit_size;
                for (int j = 0; j < n; j++) {
                    float cur_angle = arc.polar_start_theta + (float) j / n * arc.angle_radians;
                    if (cur_angle > 2 * PI)
                        cur_angle -= 2 * PI;
                    else if (cur_angle < 0)
                        cur_angle += 2 * PI;
                    Point tmp = arc.center + Point{arc.radius * std::cos(cur_angle), arc.radius * std::sin(cur_angle)};
                    m_extrusions.emplace_back(WipeTower::Extrusion(this->rotate(unscaled<float>(tmp)), width, m_current_tool));
                }
                m_extrusions.emplace_back(WipeTower::Extrusion(rot, width, m_current_tool));
            }

        }


        if (e == 0.f) {
            m_gcode += set_travel_acceleration();
        } else {
            m_gcode += set_normal_acceleration();
        }

        m_gcode += arc.direction == ArcDirection::Arc_Dir_CCW ? "G3" : "G2";
        const Vec2f center_offset = this->rotate(unscaled<float>(arc.center)) - rotated_current_pos;
        m_gcode += set_format_X(rot.x());
        m_gcode += set_format_Y(rot.y());
        m_gcode += set_format_I(center_offset.x());
        m_gcode += set_format_J(center_offset.y());

        if (e != 0.f) m_gcode += set_format_E(e);

        f = feedrate_after_travel(f, e, len, limit_flow);
        // Upstream's placement of the ceiling in Off mode, as in extrude_explicit() above.
        if (m_mixed_nozzle_slicing || (f != 0.f && f != m_current_feedrate))
            f = feedrate_within_volumetric_ceiling(f, e, len, limit_flow);
        // An arc always carries X and Y; see extrude_explicit() above.
        if (f == 0.f && m_current_feedrate <= 0.f) {
            f = feedrate_for_first_move(e, len, limit_flow);
            if (e == 0.f)
                m_feed_from_travel = true;
        }
        if (f != 0.f && f != m_current_feedrate) {
            m_gcode += set_format_F(f);
        }
        if (e > 0.f) {
            m_print_feedrate   = m_current_feedrate;
            m_feed_from_travel = false;
        }

        m_current_pos.x() = x;
        m_current_pos.y() = y;

        // Update the elapsed time with a rough estimate; see extrude_explicit() above for why a
        // move with no feedrate in force yet is counted as taking no time.
        if (m_current_feedrate > 0.f)
            m_elapsed_time += ((len == 0.f) ? std::abs(e) : len) / m_current_feedrate * 60.f;
        m_gcode += "\n";
        return *this;
    }

	WipeTowerWriter &extrude_explicit(const Vec2f &dest, float e, float f = 0.f, bool record_length = false, LimitFlow limit_flow = LimitFlow::LimitPrintFlow)
    {
        return extrude_explicit(dest.x(), dest.y(), e, f, record_length, limit_flow);
    }

	// Travel to a new XY position. In mixed-nozzle modes f=0 means the tower's own travel speed
	// rather than an unrelated inherited feedrate (for example the retraction feedrate); the next
	// extrusion that names no feedrate then goes back to the print feedrate (feedrate_after_travel).
	// Off mode keeps upstream's writer: f=0 travels at the current feedrate.
	WipeTowerWriter& travel(float x, float y, float f = 0.f)
	{
		const bool substituted = f == 0.f && m_mixed_nozzle_slicing && m_travel_speed > 0.f;
		extrude_explicit(x, y, 0.f, substituted ? m_travel_speed * 60.f : f);
		if (substituted)
			m_feed_from_travel = true;
		return *this;
	}

	WipeTowerWriter& travel(const Vec2f &dest, float f = 0.f)
		{ return travel(dest.x(), dest.y(), f); }

	// Extrude a line from current position to x, y with the extrusion amount given by m_extrusion_flow.
    WipeTowerWriter &extrude(float x, float y, float f = 0.f, LimitFlow limit_flow = LimitFlow::LimitPrintFlow)
	{
		float dx = x - m_current_pos.x();
		float dy = y - m_current_pos.y();
        return extrude_explicit(x, y, std::sqrt(dx * dx + dy * dy) * m_extrusion_flow, f, false, limit_flow);
	}
    WipeTowerWriter &extrude_arc(ArcSegment &arc, float f = 0.f, LimitFlow limit_flow = LimitFlow::LimitPrintFlow)
    {
        return extrude_arc_explicit(arc, f, false , limit_flow);
    }

	WipeTowerWriter& extrude(const Vec2f &dest, const float f = 0.f)
		{ return extrude(dest.x(), dest.y(), f); }

    WipeTowerWriter& rectangle(const Vec2f& ld,float width,float height,const float f = 0.f)
    {
        Vec2f corners[4];
        corners[0] = ld;
        corners[1] = ld + Vec2f(width,0.f);
        corners[2] = ld + Vec2f(width,height);
        corners[3] = ld + Vec2f(0.f,height);
        int index_of_closest = 0;
        if (x()-ld.x() > ld.x()+width-x())    // closer to the right
            index_of_closest = 1;
        if (y()-ld.y() > ld.y()+height-y())   // closer to the top
            index_of_closest = (index_of_closest==0 ? 3 : 2);

        travel(corners[index_of_closest].x(), y());      // travel to the closest corner
        travel(x(),corners[index_of_closest].y());

        int i = index_of_closest;
        do {
            ++i;
            if (i==4) i=0;
            extrude(corners[i], f);
        } while (i != index_of_closest);
        return (*this);
    }

    WipeTowerWriter &rectangle_fill_box(const WipeTower* wipe_tower, const Vec2f &ld, float width, float height, const float f = 0.f)
    {
        bool need_change_flow = wipe_tower->need_thick_bridge_flow(ld.y());

        Vec2f corners[4];
        corners[0]           = ld;
        corners[1]           = ld + Vec2f(width, 0.f);
        corners[2]           = ld + Vec2f(width, height);
        corners[3]           = ld + Vec2f(0.f, height);
        int index_of_closest = 0;
        if (x() - ld.x() > ld.x() + width - x()) // closer to the right
            index_of_closest = 1;
        if (y() - ld.y() > ld.y() + height - y()) // closer to the top
            index_of_closest = (index_of_closest == 0 ? 3 : 2);

        travel(corners[index_of_closest].x(), y()); // travel to the closest corner
        travel(x(), corners[index_of_closest].y());

        int i = index_of_closest;
        bool flow_changed = false;
        do {
            ++i;
            if (i == 4) i = 0;
            if (need_change_flow) {
                if (i == 1) {
                    // using bridge flow in bridge area, and add notes for gcode-check when flow changed
                    set_extrusion_flow(wipe_tower->extrusion_flow(0.2));
                    append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + std::to_string(0.2) + "\n");
                    flow_changed = true;
                } else if (i == 2 && flow_changed) {
                    set_extrusion_flow(wipe_tower->get_extrusion_flow());
                    append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + std::to_string(m_layer_height) + "\n");
                }
            }
            extrude(corners[i], f);
        } while (i != index_of_closest);
        return (*this);
    }
    WipeTowerWriter &line(const WipeTower *wipe_tower, Vec2f p0, Vec2f p1,const float f = 0.f)
    {
        bool need_change_flow = wipe_tower->need_thick_bridge_flow(p0.y());
        if (need_change_flow) {
            set_extrusion_flow(wipe_tower->extrusion_flow(0.2));
            append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + std::to_string(0.2) + "\n");
        }
        if (abs(x() - p0.x()) > abs(x() - p1.x())) std::swap(p0, p1);
        travel(p0.x(), y());
        travel(x(), p0.y());
        extrude(p1, f);
        if (need_change_flow) {
            set_extrusion_flow(wipe_tower->get_extrusion_flow());
            append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + std::to_string(m_layer_height) + "\n");
        }
        return (*this);
    }

    WipeTowerWriter &rectangle_fill_box(const WipeTower *wipe_tower, const WipeTower::box_coordinates &fill_box, std::vector<Vec2f> &finish_rect_wipe_path, const float f = 0.f)
    {
        float width  = fill_box.rd.x() - fill_box.ld.x();
        float height = fill_box.ru.y() - fill_box.rd.y();
        if (height > wipe_tower->m_perimeter_width - wipe_tower->WT_EPSILON) {
            rectangle_fill_box(wipe_tower, fill_box.ld, width, height, f);
            Vec2f target = (pos() == fill_box.ld ? fill_box.rd : (pos() == fill_box.rd ? fill_box.ru : (pos() == fill_box.ru ? fill_box.lu : fill_box.ld)));
            finish_rect_wipe_path.emplace_back(pos());
            finish_rect_wipe_path.emplace_back(target);
        } else if (height > wipe_tower->WT_EPSILON) {
            line(wipe_tower, fill_box.ld, fill_box.rd);
            Vec2f target = (pos() == fill_box.ld ? fill_box.rd : fill_box.ld);
            finish_rect_wipe_path.emplace_back(pos());
            finish_rect_wipe_path.emplace_back(target);
        }
        return (*this);
    }
    WipeTowerWriter& rectangle(const WipeTower::box_coordinates& box, const float f = 0.f)
    {
        rectangle(Vec2f(box.ld.x(), box.ld.y()),
                  box.ru.x() - box.lu.x(),
                  box.ru.y() - box.rd.y(), f);
        return (*this);
    }
    WipeTowerWriter &polygon(const Polygon &wall_polygon, const float f = 0.f)
    {
        Polyline    pl = to_polyline(wall_polygon);
        pl.simplify(WT_SIMPLIFY_TOLERANCE_SCALED);
        if (m_enable_arc_fitting) {
            pl.simplify_by_fitting_arc(SCALED_WIPE_TOWER_RESOLUTION);
        } else {
            pl.simplify(SCALED_WIPE_TOWER_RESOLUTION);
            pl.reset_to_linear_move();
        }

        // Nearest corner, with exact ties broken by smaller x then smaller y, so the entry corner
        // does not depend on the ring's starting vertex (symmetric rib tips produce ties).
        auto get_closet_idx = [this](std::vector<Segment> &corners) -> int {
            Vec2f anchor{this->m_current_pos.x(), this->m_current_pos.y()};
            int   closestIndex = -1;
            float minDistance  = std::numeric_limits<float>::max();
            for (int i = 0; i < corners.size(); ++i) {
                const float distance = (corners[i].start - anchor).squaredNorm();
                bool take = closestIndex < 0 || distance < minDistance;
                if (!take && distance == minDistance) {
                    const Vec2f &held = corners[closestIndex].start;
                    const Vec2f &here = corners[i].start;
                    take = here.x() < held.x() || (here.x() == held.x() && here.y() < held.y());
                }
                if (take) {
                    minDistance  = distance;
                    closestIndex = i;
                }
            }
            return closestIndex;
        };
        std::vector<Segment> segments;
        for (int i = 0; i < pl.fitting_result.size(); i++) {
            if (pl.fitting_result[i].path_type == EMovePathType::Linear_move) {
                for (int j = pl.fitting_result[i].start_point_index; j < pl.fitting_result[i].end_point_index; j++)
                    segments.push_back({unscaled<float>(pl.points[j]), unscaled<float>(pl.points[j + 1])});
            } else {
                int beg = pl.fitting_result[i].start_point_index;
                int end = pl.fitting_result[i].end_point_index;
                segments.push_back({unscaled<float>(pl.points[beg]), unscaled<float>(pl.points[end])});
                segments.back().is_arc     = true;
                segments.back().arcsegment = pl.fitting_result[i].arc_data;
            }
        }

        if (segments.empty())
            return (*this);

        int index_of_closest = get_closet_idx(segments);
        int i                = index_of_closest;
        travel(segments[i].start); // travel to the closest points
        segments[i].is_arc ? extrude_arc(segments[i].arcsegment, f) : extrude(segments[i].end, f);
        do {
            i = (i + 1) % segments.size();
            if (i == index_of_closest) break;
            segments[i].is_arc ? extrude_arc(segments[i].arcsegment, f) : extrude(segments[i].end, f);
        } while (1);
        return (*this);
    }

	WipeTowerWriter& load(float e, float f = 0.f)
	{
		if (e == 0.f && (f == 0.f || f == m_current_feedrate))
			return *this;
		m_gcode += "G1";
		if (e != 0.f)
			m_gcode += set_format_E(e);
		if (f != 0.f && f != m_current_feedrate)
			m_gcode += set_format_F(f);
		m_gcode += "\n";
		return *this;
	}

	WipeTowerWriter& retract(float e, float f = 0.f)
		{ return load(-e, f); }

// Loads filament while also moving towards given points in x-axis (x feedrate is limited by cutting the distance short if necessary)
    WipeTowerWriter& load_move_x_advanced(float farthest_x, float loading_dist, float loading_speed, float max_x_speed = 50.f)
    {
        float time = std::abs(loading_dist / loading_speed); // time that the move must take
        float x_distance = std::abs(farthest_x - x());       // max x-distance that we can travel
        float x_speed = x_distance / time;                   // x-speed to do it in that time

        if (x_speed > max_x_speed) {
            // Necessary x_speed is too high - we must shorten the distance to achieve max_x_speed and still respect the time.
            x_distance = max_x_speed * time;
            x_speed = max_x_speed;
        }

        float end_point = x() + (farthest_x > x() ? 1.f : -1.f) * x_distance;
        return extrude_explicit(end_point, y(), loading_dist, x_speed * 60.f, false, LimitFlow::None);
    }

	// Elevate the extruder head above the current print_z position.
	WipeTowerWriter& z_hop(float hop, float f = 0.f)
	{
		m_gcode += std::string("G1") + set_format_Z(m_current_z + hop);
		if (f != 0 && f != m_current_feedrate)
			m_gcode += set_format_F(f);
		m_gcode += "\n";
		return *this;
	}

	// Lower the extruder head back to the current print_z position.
	WipeTowerWriter& z_hop_reset(float f = 0.f)
		{ return z_hop(0, f); }

	// Move to x1, +y_increment,
	// extrude quickly amount e to x2 with feed f.
	WipeTowerWriter& ram(float x1, float x2, float dy, float e0, float e, float f)
	{
        extrude_explicit(x1, m_current_pos.y() + dy, e0, f, true, LimitFlow::None);
        extrude_explicit(x2, m_current_pos.y(), e, 0.f, true, LimitFlow::None);
		return *this;
	}

	// Let the end of the pulled out filament cool down in the cooling tube
	// by moving up and down and moving the print head left / right
	// at the current Y position to spread the leaking material.
	WipeTowerWriter& cool(float x1, float x2, float e1, float e2, float f)
	{
		extrude_explicit(x1, m_current_pos.y(), e1, f, false, LimitFlow::None);
        extrude_explicit(x2, m_current_pos.y(), e2, 0.f, false, LimitFlow::None);
		return *this;
	}

    WipeTowerWriter& set_tool(size_t tool)
	{
		m_current_tool = tool;
		return *this;
	}

	// Set extruder temperature, don't wait by default.
	WipeTowerWriter& set_extruder_temp(int temperature, bool wait = false)
	{
        m_gcode += "M" + std::to_string(wait ? 109 : 104) + " S" + std::to_string(temperature) + "\n";
        return *this;
    }

    // Wait for a period of time (seconds).
	WipeTowerWriter& wait(float time)
	{
        if (time==0.f)
            return *this;
        m_gcode += wait_command(m_gcode_flavor, time);
		return *this;
    }

	// Set speed factor override percentage.
	WipeTowerWriter& speed_override(int speed)
	{
        m_gcode += "M220 S" + std::to_string(speed) + "\n";
		return *this;
    }

    // Let the firmware back up the active speed override value.
    WipeTowerWriter& speed_override_backup()
    {
        // BBS: BBL machine don't support speed backup
        if (m_gcode_flavor == gcfMarlinLegacy || m_gcode_flavor == gcfMarlinFirmware)
            m_gcode += "M220 B\n";
        return *this;
    }

    // Let the firmware restore the active speed override value.
    WipeTowerWriter& speed_override_restore()
    {
        // BBS: BBL machine don't support speed restore
        if (m_gcode_flavor == gcfMarlinLegacy || m_gcode_flavor == gcfMarlinFirmware)
            m_gcode += "M220 R\n";
        return *this;
    }

	// Set digital trimpot motor
	WipeTowerWriter& set_extruder_trimpot(int current)
	{
        // BBS: don't control trimpot
#if 0
        if (m_gcode_flavor == gcfRepRapSprinter || m_gcode_flavor == gcfRepRapFirmware)
            m_gcode += "M906 E";
        else
            m_gcode += "M907 E";
        m_gcode += std::to_string(current) + "\n";
#endif
		return *this;
    }

	WipeTowerWriter& flush_planner_queue()
	{
		m_gcode += flush_planner_queue_command(m_gcode_flavor);
		return *this;
	}

	// Reset internal extruder counter.
	WipeTowerWriter& reset_extruder()
	{
		m_gcode += "G92 E0\n";
		return *this;
	}

	WipeTowerWriter& comment_with_value(const char *comment, int value)
    {
        m_gcode += std::string(";") + comment + std::to_string(value) + "\n";
		return *this;
    }


    WipeTowerWriter& set_fan(unsigned speed)
	{
		if (speed == m_last_fan_speed)
			return *this;
		if (speed == 0)
			m_gcode += "M107\n";
        else
            m_gcode += "M106 S" + std::to_string(unsigned(255.0 * speed / 100.0)) + "\n";
		m_last_fan_speed = speed;
		return *this;
	}

	WipeTowerWriter& append(const std::string& text) { m_gcode += text; return *this; }

    const std::vector<Vec2f>& wipe_path() const
    {
        return m_wipe_path;
    }

    WipeTowerWriter& add_wipe_point(const Vec2f& pt)
    {
        m_wipe_path.push_back(rotate(pt));
        return *this;
    }

    WipeTowerWriter& add_wipe_point(float x, float y)
    {
        return add_wipe_point(Vec2f(x, y));
    }

    WipeTowerWriter &add_wipe_path(const Polygon & polygon,double wipe_dist)
    {
        int closest_idx = polygon.closest_point_index(scaled(m_current_pos));
        Polyline wipe_path   = polygon.split_at_index(closest_idx);
        wipe_path.reverse();
        for (int i = 0; i < wipe_path.size(); ++i) {
            if (wipe_dist < EPSILON) break;
            add_wipe_point(unscaled<float>(wipe_path[i]));
            if (i != 0) wipe_dist -= (unscaled(wipe_path[i]) - unscaled(wipe_path[i - 1])).norm();
        }
        return *this;
    }
    void generate_path(Polylines &pls, float feedrate, float retract_length, float retract_speed, bool used_fillet)
    {
        // Nearest corner, with exact ties broken by smaller x then smaller y, so the entry corner
        // does not depend on the ring's starting vertex (symmetric rib tips produce ties).
        auto get_closet_idx = [this](std::vector<Segment> &corners) -> int {
            Vec2f anchor{this->m_current_pos.x(), this->m_current_pos.y()};
            int   closestIndex = -1;
            float minDistance  = std::numeric_limits<float>::max();
            for (int i = 0; i < corners.size(); ++i) {
                const float distance = (corners[i].start - anchor).squaredNorm();
                bool take = closestIndex < 0 || distance < minDistance;
                if (!take && distance == minDistance) {
                    const Vec2f &held = corners[closestIndex].start;
                    const Vec2f &here = corners[i].start;
                    take = here.x() < held.x() || (here.x() == held.x() && here.y() < held.y());
                }
                if (take) {
                    minDistance  = distance;
                    closestIndex = i;
                }
            }
            return closestIndex;
        };
        if (m_enable_arc_fitting) {
            for (auto &pl : pls) pl.simplify_by_fitting_arc(SCALED_WIPE_TOWER_RESOLUTION);
        } else {
            for (auto &pl : pls) {
                pl.simplify(SCALED_WIPE_TOWER_RESOLUTION);
                pl.reset_to_linear_move();
            }
        }

        std::vector<Segment> segments;
        for (const auto &pl : pls) {
            if (pl.points.size()<2) continue;
            for (int i = 0; i < pl.fitting_result.size(); i++) {
                if (pl.fitting_result[i].path_type == EMovePathType::Linear_move) {
                    for (int j = pl.fitting_result[i].start_point_index; j < pl.fitting_result[i].end_point_index; j++)
                        segments.push_back({unscaled<float>(pl.points[j]), unscaled<float>(pl.points[j + 1])});
                } else {
                    int beg = pl.fitting_result[i].start_point_index;
                    int end = pl.fitting_result[i].end_point_index;
                    segments.push_back({unscaled<float>(pl.points[beg]), unscaled<float>(pl.points[end])});
                    segments.back().is_arc = true;
                    segments.back().arcsegment = pl.fitting_result[i].arc_data;
                }
            }
        }
        if (segments.empty())
            return;

        int index_of_closest = get_closet_idx(segments);
        int i = index_of_closest;
        travel(segments[i].start); // travel to the closest points
        segments[i].is_arc? extrude_arc(segments[i].arcsegment,feedrate) : extrude(segments[i].end, feedrate);
        do {
            i         = (i + 1) % segments.size();
            if (i == index_of_closest) break;
            float dx  = segments[i].start.x() - m_current_pos.x();
            float dy  = segments[i].start.y() - m_current_pos.y();
            float len = std::sqrt(dx * dx + dy * dy);
            if (len > EPSILON) {
                retract(retract_length, retract_speed);
                travel(segments[i].start, 600.);
                retract(-retract_length, retract_speed);
            }
            segments[i].is_arc ? extrude_arc(segments[i].arcsegment, feedrate) : extrude(segments[i].end, feedrate);
        } while (1);
    }
    void spiral_flat_ironing(const Vec2f &center, float area, float step_length, float feedrate)
    {
        float edge_length = std::sqrt(area);
        Vec2f box_max     = center + Vec2f{step_length, step_length};
        Vec2f box_min     = center - Vec2f{step_length, step_length};
        int   n           = std::ceil(edge_length / step_length / 2.f);
        if (n <= 0) return;
        while (n--) {
            travel(box_max.x(), m_current_pos.y(), feedrate);
            travel(m_current_pos.x(), box_max.y(), feedrate);
            travel(box_min.x(), m_current_pos.y(), feedrate);
            travel(m_current_pos.x(), box_min.y(), feedrate);

            box_max += Vec2f{step_length, step_length};
            box_min -= Vec2f{step_length, step_length};
        }
    }

    WipeTowerWriter &format_line_M104(int target_temp, int target_extruder, bool wait_for_moves = true, const std::string &comment = std::string())
    {
        std::string buffer;
        if (wait_for_moves)
            // Not flush_planner_queue_command(): this BBL precool path wants M400, which every
            // flavor it reaches understands, not the zero dwell the other flavors flush with.
            buffer += "M400\n";
        buffer += "M104";
        if (target_extruder != -1)
            buffer += (" T" + std::to_string(physical_tool(target_extruder)));
        buffer += " S" + std::to_string(target_temp) + " N0"; // N0 means the gcode is generated by slicer
        if (!comment.empty()) buffer += " ;" + comment;
        buffer += '\n';
        append(buffer);
        return *this;
    }

    WipeTowerWriter &format_line_M109(int target_temp, int target_extruder, const std::string &comment = std::string())
    {
        std::string buffer = "M109";
        if (target_extruder != -1)
            buffer += (" T" + std::to_string(physical_tool(target_extruder)));
        buffer += " S" + std::to_string(target_temp) + " N0"; // N0 means the gcode is generated by slicer
        if (!comment.empty()) buffer += " ;" + comment;
        buffer += '\n';
        append(buffer);
        return *this;
    };

    // The physical tool a temperature line names. Bambu profiles map every extruder; the other
    // vendors' profiles carry a one-entry map, and there the extruder is the tool.
    int physical_tool(int extruder) const
    {
        return extruder >= 0 && size_t(extruder) < m_physical_extruder_map.size() ? m_physical_extruder_map[extruder] : extruder;
    }

    void set_first_layer(bool is_first_layer) { m_is_first_layer = is_first_layer; }
    void set_normal_acceleration(const std::vector<unsigned int> &accelerations) { m_normal_accelerations = accelerations; };
    void set_first_layer_normal_acceleration(const std::vector<unsigned int> &accelerations) { m_first_layer_normal_accelerations = accelerations; };
    void set_travel_acceleration(const std::vector<unsigned int> &accelerations) { m_travel_accelerations = accelerations; };
    void set_first_layer_travel_acceleration(const std::vector<unsigned int> &accelerations) { m_first_layer_travel_accelerations = accelerations; };
    void set_max_acceleration(unsigned int acceleration) { m_max_acceleration = acceleration; };
    void set_accel_to_decel_enable(bool enable) { m_accel_to_decel_enable = enable; }
    void set_accel_to_decel_factor(float factor) { m_accel_to_decel_factor = factor; }
    void set_layer_id(int layer_id) { m_layer_id = layer_id; }
    void set_multi_nozzle_group_result(const MultiNozzleUtils::LayeredNozzleGroupResult *multi_nozzle_group_result) { m_multi_nozzle_group_result = multi_nozzle_group_result; }
    void set_physical_extruder_map(const std::vector<int> &physical_extruder_map) { m_physical_extruder_map = physical_extruder_map; }
    // In a mixed-nozzle mode, moves that reuse the current feedrate are also held to the
    // volumetric ceiling.
    void set_mixed_nozzle_slicing(bool enabled) { m_mixed_nozzle_slicing = enabled; }

private:
    std::string set_normal_acceleration() {
        std::vector<unsigned int> accelerations = m_is_first_layer ? m_first_layer_normal_accelerations : m_normal_accelerations;
        if (accelerations.empty() || !m_multi_nozzle_group_result)
            return std::string();
        int extruder_id = m_multi_nozzle_group_result->get_extruder_id(m_current_tool, m_layer_id);
        // Orca: get_extruder_id returns -1 when the filament is not covered by the map
        // (reachable with a stale manual filament map); skip instead of indexing out of bounds.
        if (extruder_id < 0 || extruder_id >= (int) accelerations.size())
            return std::string();
        unsigned int acc         = accelerations[extruder_id];
        return set_acceleration_impl(acc);
    }
    std::string set_travel_acceleration()
    {
        std::vector<unsigned int> accelerations = m_is_first_layer ? m_first_layer_travel_accelerations : m_travel_accelerations;
        if (accelerations.empty() || !m_multi_nozzle_group_result)
            return std::string();
        int extruder_id = m_multi_nozzle_group_result->get_extruder_id(m_current_tool, m_layer_id);
        // Orca: get_extruder_id returns -1 when the filament is not covered by the map
        // (reachable with a stale manual filament map); skip instead of indexing out of bounds.
        if (extruder_id < 0 || extruder_id >= (int) accelerations.size())
            return std::string();
        unsigned int acc = accelerations[extruder_id];
        return set_acceleration_impl(acc);
    }
    std::string set_acceleration_impl(unsigned int acceleration) {
        // Clamp the acceleration to the allowed maximum.
        if (m_max_acceleration > 0 && acceleration > m_max_acceleration)
            acceleration = m_max_acceleration;

        if (acceleration == 0 || acceleration == m_last_acceleration)
            return std::string();

        m_last_acceleration = acceleration;

        std::ostringstream gcode;
        if (m_gcode_flavor == gcfRepetier) {
            // M201: Set max printing acceleration
            gcode << "M201 X" << acceleration << " Y" << acceleration;
            gcode << "\n";
            // M202: Set max travel acceleration
            gcode << "M202 X" << acceleration << " Y" << acceleration;
        } else if (m_gcode_flavor == gcfRepRapFirmware) {
            // M204: Set default acceleration
            gcode << "M204 P" << acceleration;
        } else if (m_gcode_flavor == gcfMarlinFirmware) {
            // This is new MarlinFirmware with separated print/retraction/travel acceleration.
            // Use M204 P, we don't want to override travel acc by M204 S (which is deprecated anyway).
            gcode << "M204 P" << acceleration;
        }
        else if (m_gcode_flavor == gcfKlipper && m_accel_to_decel_enable) {
            gcode << "SET_VELOCITY_LIMIT ACCEL_TO_DECEL=" << acceleration * m_accel_to_decel_factor / 100;
            gcode << "\nM204 S" << acceleration;
        }
        else {
            // M204: Set default acceleration
            gcode << "M204 S" << acceleration;
        }
        gcode << "\n";
        return gcode.str();
    }
    std::vector<unsigned int> m_normal_accelerations;
    std::vector<unsigned int> m_first_layer_normal_accelerations;
    std::vector<unsigned int> m_travel_accelerations;
    std::vector<unsigned int> m_first_layer_travel_accelerations;
    bool                      m_is_first_layer{false};
    unsigned int              m_max_acceleration{0};
    unsigned int              m_last_acceleration{0};
    bool                      m_accel_to_decel_enable;
    float                     m_accel_to_decel_factor;
    const MultiNozzleUtils::LayeredNozzleGroupResult *m_multi_nozzle_group_result{nullptr};
    int                       m_layer_id = -1;
    std::vector<int>          m_physical_extruder_map;
    bool                      m_mixed_nozzle_slicing{false};

private:
	// Eigen does not zero-initialise, and the final purge never calls set_initial_position(),
	// so both positions start from an explicit "unknown" value rather than garbage.
	static constexpr float unknown_position = std::numeric_limits<float>::max();
	Vec2f         m_start_pos { unknown_position, unknown_position };
	Vec2f         m_current_pos { unknown_position, unknown_position };
    std::vector<Vec2f>  m_wipe_path;
	float    	  m_current_z;
	float 	  	  m_current_feedrate;
	// Feedrate of the last extrusion move, and whether a travel or Z move has changed F since.
	float         m_print_feedrate { 0.f };
	bool          m_feed_from_travel { false };
    size_t        m_current_tool;
	float 		  m_layer_height;
	// mm/s, used by travel() when f=0.
	float 		  m_travel_speed;
	float 	  	  m_extrusion_flow;
	bool		  m_preview_suppressed;
	std::string   m_gcode;
	std::vector<WipeTower::Extrusion> m_extrusions;
	float         m_elapsed_time;
	float   	  m_internal_angle = 0.f;
	float		  m_y_shift = 0.f;
	float 		  m_wipe_tower_width = 0.f;
	float		  m_wipe_tower_depth = 0.f;
    unsigned      m_last_fan_speed = 0;
    int           current_temp = -1;
#if ENABLE_GCODE_VIEWER_DATA_CHECKING
    const float   m_default_analyzer_line_width;
#endif // ENABLE_GCODE_VIEWER_DATA_CHECKING
    float         m_used_filament_length = 0.f;
    GCodeFlavor   m_gcode_flavor;
    bool          m_enable_arc_fitting = true;
    const std::vector<WipeTower::FilamentParameters>& m_filpar;
    bool          m_emit_block_z = false;

	std::string   set_format_X(float x)
    {
        m_current_pos.x() = x;
        return " X" + Slic3r::float_to_string_decimal_point(x, 3);
	}

	std::string   set_format_Y(float y) {
        m_current_pos.y() = y;
        return " Y" + Slic3r::float_to_string_decimal_point(y, 3);
	}

	std::string   set_format_Z(float z) {
        return " Z" + Slic3r::float_to_string_decimal_point(z, 3);
	}

	std::string   set_format_E(float e) {
        // Orca: export E at the same precision as GCodeWriter; 4 digits was coarse enough to skew
        // the width read back from short tower rows.
        return " E" + Slic3r::float_to_string_decimal_point(e, GCodeFormatter::E_EXPORT_DIGITS);
	}

	std::string   set_format_F(float f) {
        char buf[64];
        sprintf(buf, " F%d", int(floor(f + 0.5f)));
        m_current_feedrate = f;
        return buf;
	}
    std::string set_format_I(float i) { return " I" + Slic3r::float_to_string_decimal_point(i, 3); }
    std::string set_format_J(float j) { return " J" + Slic3r::float_to_string_decimal_point(j, 3); }

	WipeTowerWriter& operator=(const WipeTowerWriter &rhs);

	// Rotate the point around center of the wipe tower about given angle (in degrees)
	Vec2f rotate(Vec2f pt) const
	{
		pt.x() -= m_wipe_tower_width / 2.f;
		pt.y() += m_y_shift - m_wipe_tower_depth / 2.f;
	    double angle = m_internal_angle * float(M_PI/180.);
	    double c = cos(angle);
	    double s = sin(angle);
	    return Vec2f(float(pt.x() * c - pt.y() * s) + m_wipe_tower_width / 2.f, float(pt.x() * s + pt.y() * c) + m_wipe_tower_depth / 2.f);
	}

}; // class WipeTowerWriter



static std::vector<WipeTower::StructuralEmission> make_structural_emissions(
    const std::vector<WipeTower::Extrusion> &extrusions, float z, float height,
    WipeTower::StructuralRole role, unsigned int support_domain_override,
    size_t first = 0, size_t last = std::numeric_limits<size_t>::max())
{
    if (role == WipeTower::StructuralRole::None)
        return {};
    const unsigned int role_support_domain = [&]() {
        switch (role) {
        case WipeTower::StructuralRole::TowerPerimeter: return 1u;
        case WipeTower::StructuralRole::TowerBlock:      return 2u;
        case WipeTower::StructuralRole::IntentionalBridge: return 3u;
        case WipeTower::StructuralRole::InteriorDeposit: break;
        case WipeTower::StructuralRole::None:            break;
        }
        return 0u;
    }();
    const unsigned int support_domain = support_domain_override != 0 ? support_domain_override : role_support_domain;

    std::vector<WipeTower::StructuralEmission> result;
    Vec2f min_pos(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f max_pos(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    unsigned int tool = 0;
    bool has_positive = false;
    auto flush = [&]() {
        if (has_positive)
            result.emplace_back(WipeTower::StructuralEmission{
                z, height, int(tool), role, min_pos, max_pos, support_domain});
        has_positive = false;
        min_pos = Vec2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        max_pos = Vec2f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    };

    for (size_t idx = first; idx < std::min(last, extrusions.size()); ++idx) {
        const WipeTower::Extrusion &extrusion = extrusions[idx];
        if (extrusion.width <= 0.f || !std::isfinite(extrusion.width))
            continue;
        if (!has_positive || extrusion.tool != tool) {
            flush();
            tool = extrusion.tool;
            has_positive = true;
        }

        // Each positive record is the end of a segment. Include its actual start as well;
        // it is commonly the preceding zero-width travel record and is otherwise absent from
        // the preview geometry bounds.
        if (idx > 0 && std::isfinite(extrusions[idx - 1].pos.x()) &&
            std::isfinite(extrusions[idx - 1].pos.y())) {
            min_pos = min_pos.cwiseMin(extrusions[idx - 1].pos);
            max_pos = max_pos.cwiseMax(extrusions[idx - 1].pos);
        }
        if (std::isfinite(extrusion.pos.x()) && std::isfinite(extrusion.pos.y())) {
            min_pos = min_pos.cwiseMin(extrusion.pos);
            max_pos = max_pos.cwiseMax(extrusion.pos);
        }
    }
    flush();
    return result;
}

WipeTower::ToolChangeResult WipeTower::construct_tcr(WipeTowerWriter& writer,
                                                     bool priming,
                                                     size_t old_tool,
                                                     bool is_finish,
                                                     bool is_tool_change,
                                                     float purge_volume,
                                                     bool is_contact,
                                                     StructuralRole structural_role) const
{
    ToolChangeResult result;
    result.priming      = priming;
    result.initial_tool = int(old_tool);
    result.new_tool     = int(m_current_tool);
    // Report the part layer the block belongs to; structural records keep the tower's own height.
    result.print_z      = m_part_z_pos > 0.f ? m_part_z_pos : m_z_pos;
    result.tower_z_start = m_z_pos;
    result.layer_height = m_layer_height;
    result.elapsed_time = writer.elapsed_time() +
        (m_nozzle_change_result.gcode.empty() ? 0.f : m_nozzle_change_result.elapsed_time);
    result.start_pos    = writer.start_pos_rotated();
    result.end_pos      = priming ? writer.pos() : writer.pos_rotated();
    result.gcode        = std::move(writer.gcode());
    result.extrusions   = std::move(writer.extrusions());
    result.structural_emissions = make_structural_emissions(result.extrusions, m_z_pos,
                                                             result.layer_height, structural_role, 0);
    // Ramming is generated by a separate writer and is carried by the nozzle-change result.
    // Merge its block-local records here exactly once with the enclosing TCR.
    result.structural_emissions.insert(result.structural_emissions.end(),
                                      m_nozzle_change_result.structural_emissions.begin(),
                                      m_nozzle_change_result.structural_emissions.end());
    result.wipe_path    = std::move(writer.wipe_path());
    result.is_finish_first = is_finish;
    result.nozzle_change_result = m_nozzle_change_result;
    result.is_tool_change       = is_tool_change;
    result.tool_change_start_pos = is_tool_change ? result.start_pos : Vec2f(0, 0);
    result.is_contact            = is_contact;
    // BBS
    result.purge_volume = purge_volume;
    return result;
}

WipeTower::ToolChangeResult WipeTower::construct_block_tcr(WipeTowerWriter &writer, bool priming,
                                                           size_t filament_id, bool is_finish,
                                                           float purge_volume,
                                                           StructuralRole structural_role,
                                                           float structural_height,
                                                           unsigned int support_domain) const
{
    ToolChangeResult result;
    result.priming              = priming;
    result.initial_tool         = int(filament_id);
    result.new_tool             = int(filament_id);
    result.print_z              = m_part_z_pos > 0.f ? m_part_z_pos : m_z_pos;
    result.tower_z_start        = m_z_pos;
    result.layer_height         = m_layer_height;
    result.elapsed_time         = writer.elapsed_time();
    result.start_pos            = writer.start_pos_rotated();
    result.end_pos              = priming ? writer.pos() : writer.pos_rotated();
    result.gcode                = std::move(writer.gcode());
    result.extrusions           = std::move(writer.extrusions());
    result.structural_emissions = make_structural_emissions(result.extrusions, m_z_pos,
                                                             structural_height > 0.f ? structural_height : result.layer_height,
                                                             structural_role, support_domain);
    result.wipe_path            = std::move(writer.wipe_path());
    result.is_finish_first      = is_finish;
    result.is_tool_change       = false;
    result.tool_change_start_pos = Vec2f(0, 0);
    // BBS
    result.purge_volume = purge_volume;
    return result;
}

// BBS
const double wrapping_wipe_tower_depth = 10;

// BBS
const std::map<float, float> WipeTower::min_depth_per_height = {
    {5.f,5.f}, {100.f, 20.f}, {250.f, 40.f}, {350.f, 60.f}
};

float WipeTower::get_limit_depth_by_height(float max_height)
{
    float min_wipe_tower_depth = 0.f;
    auto  iter                 = WipeTower::min_depth_per_height.begin();
    while (iter != WipeTower::min_depth_per_height.end()) {
        auto curr_height_to_depth = *iter;

        // This is the case that wipe tower height is lower than the first min_depth_to_height member.
        if (curr_height_to_depth.first >= max_height) {
            min_wipe_tower_depth = curr_height_to_depth.second;
            break;
        }

        iter++;

        // If curr_height_to_depth is the last member, use its min_depth.
        if (iter == WipeTower::min_depth_per_height.end()) {
            min_wipe_tower_depth = curr_height_to_depth.second;
            break;
        }

        // If wipe tower height is between the current and next member, set the min_depth as linear interpolation between them
        auto next_height_to_depth = *iter;
        if (next_height_to_depth.first > max_height) {
            float height_base    = curr_height_to_depth.first;
            float height_diff    = next_height_to_depth.first - curr_height_to_depth.first;
            float min_depth_base = curr_height_to_depth.second;
            float depth_diff     = next_height_to_depth.second - curr_height_to_depth.second;

            min_wipe_tower_depth = min_depth_base + (max_height - curr_height_to_depth.first) / height_diff * depth_diff;
            break;
        }
    }
    return min_wipe_tower_depth;
}

float WipeTower::get_auto_brim_by_height(float max_height) {
    if (max_height < 100) return max_height/100.f * 8.f;
    return 8.f;
}

float WipeTower::tower_footprint_floor() const
{
    // Mixed-nozzle towers only. The height is the planned tower height from generate_new().
    if (!m_mixed_nozzle_slicing)
        return 0.f;
    if (!std::isfinite(m_wipe_tower_height) || m_wipe_tower_height <= 0.f)
        return 0.f;
    // The tier floors the printed pad, rib wall included; convert it to the nominal rectangle
    // the planner works in. The stock height/depth rule stays in min_wipe_tower_depth.
    return rect_floor_for_pad_extent(mixed_nozzle_tower_footprint_floor(m_wipe_tower_height));
}

float WipeTower::rect_floor_for_pad_extent(float pad_extent_floor) const
{
    if (!std::isfinite(pad_extent_floor) || pad_extent_floor <= 0.f)
        return 0.f;
    if (!m_use_rib_wall)
        return pad_extent_floor;
    // Start from the closed form, then correct against rib_section(), since a fillet pulls the
    // rib tips back. The reach barely changes with side length, so this converges in one pass.
    float side = mixed_nozzle_tower_rect_floor(pad_extent_floor, m_rib_width, true);
    for (int pass = 0; pass < 4 && side > float(EPSILON); ++pass) {
        const float rib_width = std::min(m_rib_width, side / 2.f);
        const Polygon section = rib_section(side, side, side * std::sqrt(2.f), rib_width, m_used_fillet);
        if (section.points.size() < 3)
            break;
        const BoundingBox box = section.bounding_box();
        const float reach = std::min(unscaled<float>(box.max.x() - box.min.x()),
                                     unscaled<float>(box.max.y() - box.min.y())) - side;
        // A rib that reaches nowhere leaves the rectangle carrying the whole floor.
        const float want = std::min(pad_extent_floor, pad_extent_floor - reach);
        if (std::abs(want - side) <= float(EPSILON))
            break;
        side = want;
    }
    return std::max(0.f, std::min(side, pad_extent_floor));
}

void WipeTower::update_tower_base_extent()
{
    // Top Z of the solid tower base, from the final plan. Zero when not a mixed-nozzle tower.
    m_tower_base_top_z = 0.f;
    if (!m_mixed_nozzle_slicing || m_plan.empty())
        return;
    if (!std::isfinite(m_wipe_tower_height) || m_wipe_tower_height <= 0.f)
        return;
    const float required = mixed_nozzle_tower_base_height(m_wipe_tower_height);
    if (required <= 0.f)
        return;
    const size_t first = (m_first_layer_idx == size_t(-1) || m_first_layer_idx >= m_plan.size())
                             ? 0 : m_first_layer_idx;
    // Take whole layers until the base is at least as thick as required.
    float top = m_plan[first].z;
    for (size_t i = first; i < m_plan.size(); ++i) {
        // Idle or prime-only levels (lagging plans only) do not raise the tower top.
        if (m_plan[i].idle || m_plan[i].prime_only)
            continue;
        top = m_plan[i].z;
        if (top + float(EPSILON) >= required)
            break;
    }
    m_tower_base_top_z = top;
}

bool WipeTower::is_tower_base_layer() const
{
    if (m_tower_base_top_z <= 0.f || m_plan.empty() || m_layer_info == m_plan.end())
        return false;
    return m_layer_info->z <= m_tower_base_top_z + float(EPSILON);
}

float WipeTower::base_flow_ratio_for_width(int tool, float road_width) const
{
    const float ratio = first_layer_flow_ratio();
    // The road-width cap applies in mixed-nozzle modes only.
    if (! m_mixed_nozzle_slicing)
        return ratio;
    if (ratio <= 1.f || !std::isfinite(road_width) || road_width <= 0.f)
        return ratio;
    if (tool < 0 || size_t(tool) >= m_filpar.size())
        return 1.f;
    const float nozzle = m_filpar[size_t(tool)].nozzle_diameter;
    if (!std::isfinite(nozzle) || nozzle <= 0.f)
        return 1.f;
    const float cap = nozzle * Width_To_Nozzle_Ratio;
    return std::min(ratio, std::max(1.f, cap / road_width));
}

// The purge row the fine nozzle wipes on before it leaves: a whole row from the middle of this
// visit's rows, skipping short pieces and row steps. In the writer's rotated frame, like wipe_path.
static std::vector<Vec2f> middle_purge_row(const std::vector<WipeTower::Extrusion> &extrusions, size_t begin)
{
    std::vector<std::pair<Vec2f, Vec2f>> rows;
    float longest = 0.f;
    for (size_t i = std::max<size_t>(begin, 1); i < extrusions.size(); ++i)
        if (extrusions[i].width > 0.f)
            longest = std::max(longest, (extrusions[i].pos - extrusions[i - 1].pos).norm());
    if (longest <= 0.f)
        return {};
    for (size_t i = std::max<size_t>(begin, 1); i < extrusions.size(); ++i)
        if (extrusions[i].width > 0.f && (extrusions[i].pos - extrusions[i - 1].pos).norm() >= 0.5f * longest)
            rows.emplace_back(extrusions[i - 1].pos, extrusions[i].pos);
    if (rows.empty())
        return {};
    const auto &row = rows[rows.size() / 2];
    return { row.first, row.second };
}

bool WipeTower::is_fine_return(size_t old_tool, size_t new_tool) const
{
    // The fine tool is the one with the smaller nozzle.
    if (old_tool >= m_filpar.size() || new_tool >= m_filpar.size())
        return false;
    const float departing = m_filpar[old_tool].nozzle_diameter;
    const float arriving  = m_filpar[new_tool].nozzle_diameter;
    if (!std::isfinite(departing) || !std::isfinite(arriving))
        return false;
    return arriving + float(EPSILON) < departing;
}

Vec2f WipeTower::move_box_inside_polygon(const BoundingBox &box, const Polygons &polygons, coord_t offset)
{
    if (polygons.empty()) return Vec2f{0.f, 0.f};

    const BoundingBox bed = get_extents(polygons);
    // No position fits the footprint.
    if (box.size().x() >= bed.size().x() - 2 * offset || box.size().y() >= bed.size().y() - 2 * offset)
        return Vec2f{0.f, 0.f};

    // Clamp against the bounding box first, moving only along the axis that is violated so a dragged
    // prime tower slides along the bed edge instead of jumping inwards.
    Point shift(0, 0);
    for (int axis = 0; axis < 2; ++axis) {
        if (box.max[axis] > bed.max[axis] - offset)
            shift[axis] = (bed.max[axis] - offset) - box.max[axis];
        else if (box.min[axis] < bed.min[axis] + offset)
            shift[axis] = (bed.min[axis] + offset) - box.min[axis];
    }

    // A bed that fills its own bounding box is fully clamped by that, so every rectangular bed — all
    // but the delta-style profiles — stops here and keeps its historic placement, including when a
    // negative margin lets the footprint hang over the edge. The tolerance is relative because an
    // exact rectangle loses a few ulps once the areas are squared world coordinates.
    double area = 0.;
    for (const Polygon &poly : polygons) area += std::abs(poly.area());
    const double bed_area = double(bed.size().x()) * double(bed.size().y());
    if (area >= bed_area * (1. - EPSILON)) return unscaled<float>(shift);

    // Clamp a negative margin (an auto brim width that has not been resolved yet) to zero: padding by
    // it would shrink the footprint and hand back a position the validation still rejects. The
    // epsilon lets the move's round trip through millimeters land on the outline without counting as
    // a violation.
    BoundingBox padded = box.inflated(std::max<coord_t>(offset, 0) - SCALED_EPSILON);
    padded.translate(shift);
    auto fits = [&padded, &polygons](const Point &move) {
        BoundingBox moved = padded;
        moved.translate(move);
        return diff(Polygons{moved.polygon()}, polygons).empty();
    };
    if (fits(Point(0, 0))) return unscaled<float>(shift);

    // Walk towards the middle of the bed. On every non-rectangular bed we ship, the fitting positions
    // form a convex region around it, so bisecting stops just inside the outline.
    Point lo(0, 0), hi = bed.center() - padded.center();
    if (!fits(hi)) return unscaled<float>(shift);
    for (int i = 0; i < 12; ++i) {
        const Point mid = (lo + hi) / 2;
        if (fits(mid)) hi = mid; else lo = mid;
    }
    return unscaled<float>(Point(shift + hi));
}

Polygon WipeTower::rib_section(float width, float depth, float rib_length, float rib_width,bool fillet_wall)
{
    Polygon res;
    res.points.resize(16);
    float              theta     = std::atan(width / depth);
    float              costheta  = std::cos(theta);
    float              sintheta  = std::sin(theta);
    float              w         = rib_width / 2.f;
    float              diag      = std::sqrt(width * width + depth * depth);
    float              l         = (rib_length - diag) / 2;
    Vec2f              diag_dir1 = Vec2f{width, depth}.normalized();
    Vec2f              diag_dir1_perp{-diag_dir1[1], diag_dir1[0]};
    Vec2f              diag_dir2 = Vec2f{-width, depth}.normalized();
    Vec2f              diag_dir2_perp{-diag_dir2[1], diag_dir2[0]};
    std::vector<Vec2f> p{{0, 0}, {width, 0}, {width, depth}, {0, depth}};
    Polyline           p_render;
    for (auto &x : p) p_render.points.push_back(scaled(x));
    res.points[0] = scaled(Vec2f{p[0].x(), p[0].y() + w / sintheta});
    res.points[1] = scaled(Vec2f{p[0] - diag_dir1 * l + diag_dir1_perp * w});
    res.points[2] = scaled(Vec2f{p[0] - diag_dir1 * l - diag_dir1_perp * w});
    res.points[3] = scaled(Vec2f{p[0].x() + w / costheta, p[0].y()});

    res.points[4] = scaled(Vec2f{p[1].x() - w / costheta, p[1].y()});
    res.points[5] = scaled(Vec2f{p[1] - diag_dir2 * l + diag_dir2_perp * w});
    res.points[6] = scaled(Vec2f{p[1] - diag_dir2 * l - diag_dir2_perp * w});
    res.points[7] = scaled(Vec2f{p[1].x(), p[1].y() + w / sintheta});

    res.points[8]  = scaled(Vec2f{p[2].x(), p[2].y() - w / sintheta});
    res.points[9]  = scaled(Vec2f{p[2] + diag_dir1 * l - diag_dir1_perp * w});
    res.points[10] = scaled(Vec2f{p[2] + diag_dir1 * l + diag_dir1_perp * w});
    res.points[11] = scaled(Vec2f{p[2].x() - w / costheta, p[2].y()});

    res.points[12] = scaled(Vec2f{p[3].x() + w / costheta, p[3].y()});
    res.points[13] = scaled(Vec2f{p[3] + diag_dir2 * l - diag_dir2_perp * w});
    res.points[14] = scaled(Vec2f{p[3] + diag_dir2 * l + diag_dir2_perp * w});
    res.points[15] = scaled(Vec2f{p[3].x(), p[3].y() - w / sintheta});
    res.remove_duplicate_points();
    if (fillet_wall) { res = rounding_polygon(res); }
    res.points.shrink_to_fit();
    return res;
}

TriangleMesh WipeTower::its_make_rib_tower(float width, float depth, float height, float rib_length, float rib_width, bool fillet_wall)
{
    TriangleMesh res;
    Polygon      bottom = rib_section(width, depth, rib_length, rib_width, fillet_wall);
    Polygon      top    = rib_section(width, depth, std::sqrt(width * width + depth * depth), rib_width, fillet_wall);
    if (fillet_wall)
        assert(bottom.points.size() == top.points.size());
    int     offset       = bottom.points.size();
    res.its.vertices.reserve(offset * 2);
    if (bottom.area() < scaled(EPSILON) || top.area() < scaled(EPSILON) || bottom.points.size() != top.points.size()) return res;
    auto    faces_bottom = Triangulation::triangulate(bottom);
    auto    faces_top    = Triangulation::triangulate(top);
    res.its.indices.reserve(offset * 2 + faces_bottom.size() + faces_top.size());
    for (auto &t : faces_bottom) res.its.indices.push_back({t[1], t[0], t[2]});
    for (auto &t : faces_top) res.its.indices.push_back({t[0] + offset, t[1] + offset, t[2] + offset});

    for (int i = 0; i < bottom.size(); i++) res.its.vertices.push_back({unscaled<float>(bottom[i][0]), unscaled<float>(bottom[i][1]), 0});
    for (int i = 0; i < top.size(); i++) res.its.vertices.push_back({unscaled<float>(top[i][0]), unscaled<float>(top[i][1]), height});

    for (int i = 0; i < offset; i++) {
        int a = i;
        int b = (i + 1) % offset;
        int c = i + offset;
        int d = b + offset;
        res.its.indices.push_back({a, b, c});
        res.its.indices.push_back({d, c, b});
    }
    return res;
}

TriangleMesh WipeTower::its_make_rib_brim(const Polygon& brim, float layer_height) {
    TriangleMesh res;
    if (brim.area() < scaled(EPSILON))return res;
    int          offset = brim.size();
    res.its.vertices.reserve(brim.size() * 2);
    auto    faces= Triangulation::triangulate(brim);
    res.its.indices.reserve(brim.size() * 2  + 2 * faces.size());
    for (auto &t : faces) res.its.indices.push_back({t[1], t[0], t[2]});
    for (auto &t : faces) res.its.indices.push_back({t[0] + offset, t[1] + offset, t[2] + offset});

    for (int i = 0; i < brim.size(); i++) res.its.vertices.push_back({unscaled<float>(brim[i][0]), unscaled<float>(brim[i][1]), 0});
    for (int i = 0; i < brim.size(); i++) res.its.vertices.push_back({unscaled<float>(brim[i][0]), unscaled<float>(brim[i][1]), layer_height});

    for (int i = 0; i < offset; i++) {
        int a = i;
        int b = (i + 1) % offset;
        int c = i + offset;
        int d = b + offset;
        res.its.indices.push_back({a, b, c});
        res.its.indices.push_back({d, c, b});
    }
    return res;
}


WipeTower::WipeTower(const PrintConfig& config, int plate_idx, Vec3d plate_origin, size_t initial_tool,
                     const float wipe_tower_height, const std::vector<unsigned int>& slice_used_filaments,
                     float nominal_width_override) :
    m_semm(config.single_extruder_multi_material.value),
    m_wipe_tower_pos(config.wipe_tower_x.get_at(plate_idx), config.wipe_tower_y.get_at(plate_idx)),
    m_wipe_tower_width(nominal_width_override > 0.f ? nominal_width_override : float(config.prime_tower_width)),
    // BBS
    m_wipe_tower_height(wipe_tower_height),
    m_wipe_tower_rotation_angle(float(config.wipe_tower_rotation_angle)),
    m_wipe_tower_brim_width(float(config.prime_tower_brim_width)),
    m_y_shift(0.f),
    m_z_pos(0.f),
    //m_bridging(float(config.wipe_tower_bridging)),
    m_bridging(10.f),
    // Feature Split always uses the compact tower; other modes read the option.
    m_no_sparse_layers(mixed_nozzle_compact_tower(config)),
    m_gcode_flavor(config.gcode_flavor),
    m_travel_speed(config.travel_speed.get_at(get_extruder_index(config, (unsigned int)initial_tool))),
    m_current_tool(initial_tool),
    //wipe_volumes(flush_matrix)
    m_enable_timelapse_print(config.timelapse_type.value == TimelapseType::tlSmooth),
    m_enable_wrapping_detection(config.enable_wrapping_detection),
    m_wrapping_detection_layers(config.wrapping_detection_layers.value && (config.wrapping_exclude_area.values.size() > 2)),
    m_slice_used_filaments(slice_used_filaments.size()),
    m_is_multi_extruder(config.nozzle_diameter.size() > 1),
    m_use_gap_wall(config.prime_tower_skip_points.value),
    // Orca: rib-wall options live under wipe_tower_* names and the wall type is an enum
    m_use_rib_wall(config.wipe_tower_wall_type.value == WipeTowerWallType::wtwRib),
    m_extra_rib_length((float)config.wipe_tower_extra_rib_length.value),
    m_rib_width((float)config.wipe_tower_rib_width.value),
    m_used_fillet(config.wipe_tower_fillet_wall.value),
    m_extra_spacing((float)config.prime_tower_infill_gap.value/100.f),
    m_tower_framework(config.prime_tower_enable_framework.value),
    // Orca: prime_tower_max_speed is named wipe_tower_max_purge_speed (same default/min)
    m_max_speed((float)config.wipe_tower_max_purge_speed.value*60.f),
    m_accel_to_decel_enable(config.accel_to_decel_enable.value),
    m_accel_to_decel_factor(config.accel_to_decel_factor.value),
    m_printable_height(config.extruder_printable_height.values),
    m_flat_ironing(config.prime_tower_flat_ironing.value),
    m_enable_tower_interface_features(config.enable_tower_interface_features.value),
    m_physical_extruder_map(config.physical_extruder_map.values),
    m_enable_arc_fitting(config.enable_arc_fitting.value)
    // Orca: has_filament_switcher is a device-set dynamic key, not a PrintConfig member;
    // it is pushed in from Print via set_has_filament_switcher() instead of read here.
{
    m_mixed_nozzle_slicing           = is_mixed_nozzle_slicing_enabled(config);
    m_contact_speed                  = 20 * 60.f;
    // No in-class default; a replay or test tower never gets Print's configure step, so start at
    // the preset default rather than uninitialised memory.
    m_first_layer_flow_ratio         = 1.f;
    m_filaments_change_length.first = config.filament_change_length.values;
    m_filaments_change_length.second = config.filament_change_length_nc.values;
    m_hotend_heating_rate            = config.hotend_heating_rate.values;
    m_hotend_cooling_rate            = config.hotend_cooling_rate.values;
    m_flat_ironing = (m_flat_ironing && m_use_gap_wall);
    // Orca: default/initial-layer/travel acceleration are object-scope options here (PrintConfig
    // members in BBS), so Print pushes the resolved columns in via set_accelerations() instead of
    // the ctor reading them from config.
    m_max_accels = config.machine_max_acceleration_extruding.values.front();

    // Read absolute value of first layer speed, if given as percentage,
    // it is taken over following default. Speeds from config are not
    // easily accessible here.
    const float default_speed = 60.f;
    m_first_layer_speed       = config.initial_layer_speed.get_at(get_extruder_index(config, (unsigned int) initial_tool));
    if (m_first_layer_speed == 0.f) // just to make sure autospeed doesn't break it.
        m_first_layer_speed = default_speed / 2.f;
    // If this is a single extruder MM printer, we will use all the SE-specific config values.
    // Otherwise, the defaults will be used to turn off the SE stuff.
    // BBS: remove useless config
#if 0
    if (m_semm) {
        m_cooling_tube_retraction = float(config.cooling_tube_retraction);
        m_cooling_tube_length     = float(config.cooling_tube_length);
        m_parking_pos_retraction  = float(config.parking_pos_retraction);
        m_extra_loading_move      = float(config.extra_loading_move);
        m_set_extruder_trimpot    = config.high_current_on_filament_swap;
    }
#endif
    // Calculate where the priming lines should be - very naive test not detecting parallelograms etc.
    const std::vector<Vec2d>& bed_points = config.printable_area.values;
    BoundingBoxf bb(bed_points);
    m_bed_width = float(bb.size().x());
    m_bed_shape = (bed_points.size() == 4 ? RectangularBed : CircularBed);

    if (m_bed_shape == CircularBed) {
        // this may still be a custom bed, check that the points are roughly on a circle
        double r2 = std::pow(m_bed_width/2., 2.);
        double lim2 = std::pow(m_bed_width/10., 2.);
        Vec2d center = bb.center();
        for (const Vec2d& pt : bed_points)
            if (std::abs(std::pow(pt.x()-center.x(), 2.) + std::pow(pt.y()-center.y(), 2.) - r2) > lim2) {
                m_bed_shape = CustomBed;
                break;
            }
    }

    m_bed_bottom_left = m_bed_shape == RectangularBed
                  ? Vec2f(bed_points.front().x(), bed_points.front().y())
                  : Vec2f::Zero();
    m_last_layer_id.resize(config.nozzle_diameter.size(), -1);
    m_origin  = {plate_origin[0], plate_origin[1]};
    m_is_multiple_nozzle = std::any_of(config.extruder_max_nozzle_count.values.begin(), config.extruder_max_nozzle_count.values.end(), [](auto &elem) { return elem > 1; });
}


void WipeTower::set_extruder(size_t idx, const PrintConfig& config)
{
    //while (m_filpar.size() < idx+1)   // makes sure the required element is in the vector
    m_filpar.push_back(FilamentParameters());

    m_filpar[idx].material = config.filament_type.get_at(idx);
    // Orca: wipe_tower_filament (issue #10971) forces a specific filament to print the tower wall by
    // marking every other filament as "soluble"; 0 keeps the plain per-filament soluble flag.
    m_filpar[idx].is_soluble = config.wipe_tower_filament == 0 ? config.filament_soluble.get_at(idx) : (idx != size_t(config.wipe_tower_filament - 1));
    // BBS
    m_filpar[idx].is_support = config.filament_is_support.get_at(idx);
    m_filpar[idx].nozzle_temperature = config.nozzle_temperature.get_at(idx);
    m_filpar[idx].nozzle_temperature_initial_layer = config.nozzle_temperature_initial_layer.get_at(idx);
    m_filpar[idx].category = config.filament_adhesiveness_category.get_at(idx);
    m_filpar[idx].flat_iron_area = config.filament_tower_ironing_area.get_at(idx);

    // If this is a single extruder MM printer, we will use all the SE-specific config values.
    // Otherwise, the defaults will be used to turn off the SE stuff.
    // BBS: remove useless config
#if 0
    if (m_semm) {
        m_filpar[idx].loading_speed           = float(config.filament_loading_speed.get_at(idx));
        m_filpar[idx].loading_speed_start     = float(config.filament_loading_speed_start.get_at(idx));
        m_filpar[idx].unloading_speed         = float(config.filament_unloading_speed.get_at(idx));
        m_filpar[idx].unloading_speed_start   = float(config.filament_unloading_speed_start.get_at(idx));
        m_filpar[idx].delay                   = float(config.filament_toolchange_delay.get_at(idx));
        m_filpar[idx].cooling_moves           = config.filament_cooling_moves.get_at(idx);
        m_filpar[idx].cooling_initial_speed   = float(config.filament_cooling_initial_speed.get_at(idx));
        m_filpar[idx].cooling_final_speed     = float(config.filament_cooling_final_speed.get_at(idx));
    }
#endif

    m_filpar[idx].filament_area = float((M_PI/4.f) * pow(config.filament_diameter.get_at(idx), 2)); // all extruders are assumed to have the same filament diameter at this point
    // idx is a logical filament, but in mixed-nozzle modes nozzle_diameter is indexed by physical
    // tool, so resolve the tool that actually prints this filament. Other modes keep the stock lookup.
    const bool mixed_nozzle = is_mixed_nozzle_body_split(config) || is_mixed_nozzle_feature_split(config);
    const MixedNozzleToolResolution resolved = mixed_nozzle ?
        resolve_mixed_nozzle_tool(config, idx, MixedNozzleResolveScope::PhysicalToolOnly) : MixedNozzleToolResolution{};
    const float nozzle_diameter = resolved.tool.has_value() ?
        float(resolved.tool->nozzle_diameter) : float(config.nozzle_diameter.get_at(idx));
    m_filpar[idx].nozzle_diameter = nozzle_diameter; // to be used in future with (non-single) multiextruder MM
    // Layer-height limits of this filament's nozzle, as the structural audit reads them. Infinity
    // where no nozzle resolves.
    m_filpar[idx].max_layer_height = std::numeric_limits<float>::infinity();
    // Minimum, read only by a mixed-nozzle tower (ramming_height()); zero otherwise.
    m_filpar[idx].min_layer_height = 0.f;
    if (idx < config.filament_map.values.size()) {
        const int mapped_nozzle = config.filament_map.values[idx] - 1;
        if (mapped_nozzle >= 0 && size_t(mapped_nozzle) < config.nozzle_diameter.values.size()) {
            const double resolved_max = resolved_max_layer_height(config, size_t(mapped_nozzle));
            if (std::isfinite(resolved_max) && resolved_max > 0.)
                m_filpar[idx].max_layer_height = float(resolved_max);
            const double resolved_min = resolved_min_layer_height(config, size_t(mapped_nozzle));
            if (mixed_nozzle && std::isfinite(resolved_min) && resolved_min > 0.)
                m_filpar[idx].min_layer_height = float(resolved_min);
        }
    }

    float max_vol_speed = float(config.filament_max_volumetric_speed.get_at(idx));
    if (max_vol_speed!= 0.f)
        m_filpar[idx].max_e_speed = (max_vol_speed / filament_area());

    //set extruder change and nozzle change ramming speed
    {
        float ramming_vol_speed = float(config.filament_ramming_volumetric_speed.get_at(idx));
        if (config.filament_ramming_volumetric_speed.is_nil(idx) || is_approx(config.filament_ramming_volumetric_speed.get_at(idx), -1.)) ramming_vol_speed = max_vol_speed;
        m_filpar[idx].max_e_ramming_speed.first = (ramming_vol_speed / filament_area());

        float ramming_vol_speed_nc = float(config.filament_ramming_volumetric_speed_nc.get_at(idx));
        if (config.filament_ramming_volumetric_speed_nc.is_nil(idx) || is_approx(config.filament_ramming_volumetric_speed_nc.get_at(idx), -1.))
            ramming_vol_speed_nc = max_vol_speed;
        m_filpar[idx].max_e_ramming_speed.second = (ramming_vol_speed_nc / filament_area());
    }

    //set precooling time/precooling target temp during extruder change and nozzle change
    {
        int extruder_count = m_multi_nozzle_group_result->get_extruder_count();
        m_filpar[idx].precool_t.first.resize(extruder_count, 0.f);
        m_filpar[idx].precool_t_first_layer.first.resize(extruder_count, 0.f);
        m_filpar[idx].precool_t.second.resize(extruder_count, 0.f);
        m_filpar[idx].precool_t_first_layer.second.resize(extruder_count, 0.f);
        m_filpar[idx].precool_target_temp.first     = 0;
        m_filpar[idx].precool_target_temp.second     = 0;
        float nozzle_temp_first_layer = config.nozzle_temperature_initial_layer.is_nil(idx) ? -1.f : float(config.nozzle_temperature_initial_layer.get_at(idx));
        float nozzle_temp_other_layer = config.nozzle_temperature.is_nil(idx) ? -1.f : float(config.nozzle_temperature.get_at(idx));
        std::vector<double> hotend_cooling_rates    = config.hotend_cooling_rate.values;
        auto  is_need_precooling      = [&](bool extruder_change) -> bool
        {
            bool res = config.enable_pre_heating.value; 
            if (extruder_change) return res &&!config.filament_pre_cooling_temperature.is_nil(idx) && config.filament_pre_cooling_temperature.get_at(idx) != 0;
            return res &&!config.filament_pre_cooling_temperature_nc.is_nil(idx) && config.filament_pre_cooling_temperature_nc.get_at(idx) != 0;
        };
        if (is_need_precooling(true)) {
            for (int i = 0; i < m_filpar[idx].precool_t.first.size(); i++) {
                if (config.hotend_cooling_rate.is_nil(i)) continue;
                m_filpar[idx].precool_t.first[i] = std::max(0.f, nozzle_temp_other_layer - float(config.filament_pre_cooling_temperature.get_at(idx))) / float(hotend_cooling_rates[i]);
                m_filpar[idx].precool_t_first_layer.first[i] = std::max(0.f, nozzle_temp_first_layer -float(config.filament_pre_cooling_temperature.get_at(idx))) /float(hotend_cooling_rates[i]);
            }
            m_filpar[idx].precool_target_temp.first = config.filament_pre_cooling_temperature.get_at(idx);
        }

        if (is_need_precooling(false)) {
            for (int i = 0; i < m_filpar[idx].precool_t.second.size(); i++) {
                if (config.hotend_cooling_rate.is_nil(i)) continue;
                m_filpar[idx].precool_t.second[i] = std::max(0.f, nozzle_temp_other_layer - float(config.filament_pre_cooling_temperature_nc.get_at(idx))) / float(hotend_cooling_rates[i]);
                m_filpar[idx].precool_t_first_layer.second[i] = std::max(0.f, nozzle_temp_first_layer -float(config.filament_pre_cooling_temperature_nc.get_at(idx))) /float(hotend_cooling_rates[i]);
            }
            m_filpar[idx].precool_target_temp.second = config.filament_pre_cooling_temperature_nc.get_at(idx);
        }

    }
    //set ramming reverse travel time during extruder change and nozzle change
    {
        m_filpar[idx].ramming_travel_time = {0, 0};
        if (!config.filament_ramming_travel_time.is_nil(idx)) m_filpar[idx].ramming_travel_time.first = float(config.filament_ramming_travel_time.get_at(idx));
        if (!config.filament_ramming_travel_time_nc.is_nil(idx)) m_filpar[idx].ramming_travel_time.second = float(config.filament_ramming_travel_time_nc.get_at(idx));
    }
    // Orca: these widths are shared and overwritten per call, so the last filament would win. In
    // mixed-nozzle modes use the smallest installed nozzle, the one width every tool can draw,
    // independent of call order.
    const bool  shared_min_nozzle_width = (is_mixed_nozzle_body_split(config) || is_mixed_nozzle_feature_split(config)) &&
        ! config.nozzle_diameter.values.empty();
    const float tower_nozzle_diameter   = shared_min_nozzle_width ?
        float(*std::min_element(config.nozzle_diameter.values.begin(), config.nozzle_diameter.values.end())) : nozzle_diameter;
    m_perimeter_width = tower_nozzle_diameter * Width_To_Nozzle_Ratio; // all extruders are now assumed to have the same diameter
    // The nozzle-change line follows the same minimum: it is drawn by the outgoing tool, so only
    // the smallest nozzle is valid for both directions.
    // Orca: custom presets may use nozzle diameters outside the BBS table; fall back to the
    // previous 2*perimeter_width rule (identical to the table for 0.4) instead of throwing.
    {
        auto nc_width_it = nozzle_diameter_to_nozzle_change_width.find(tower_nozzle_diameter);
        m_nozzle_change_perimeter_width = nc_width_it != nozzle_diameter_to_nozzle_change_width.end() ? nc_width_it->second : 2.f * m_perimeter_width;
    }
    // BBS: remove useless config
#if 0
    if (m_semm) {
        std::istringstream stream{config.filament_ramming_parameters.get_at(idx)};
        float speed = 0.f;
        stream >> m_filpar[idx].ramming_line_width_multiplicator >> m_filpar[idx].ramming_step_multiplicator;
        m_filpar[idx].ramming_line_width_multiplicator /= 100;
        m_filpar[idx].ramming_step_multiplicator /= 100;
        while (stream >> speed)
            m_filpar[idx].ramming_speed.push_back(speed);
    }
#endif

    m_used_filament_length.resize(std::max(m_used_filament_length.size(), idx + 1)); // makes sure that the vector is big enough so we don't have to check later

    m_filpar[idx].retract_length = config.retraction_length.get_at(idx);
    m_filpar[idx].retract_speed  = config.retraction_speed.get_at(idx);
    m_filpar[idx].wipe_dist      = config.wipe_distance.get_at(idx);
    m_filpar[idx].filament_cooling_before_tower = config.filament_cooling_before_tower.get_at(idx);
    m_filpar[idx].filament_petg_pre_extrusion_offset_dist = config.filament_tower_interface_pre_extrusion_dist.get_at(idx);
    if (config.enable_tower_interface_features.value) {
        m_filpar[idx].filament_tower_interface_print_temp = config.filament_tower_interface_print_temp.get_at(idx) == -1 ? config.nozzle_temperature_range_high.get_at(idx) :
                                                                                                                           config.filament_tower_interface_print_temp.get_at(idx);
        m_filpar[idx].filament_tower_interface_pre_extrusion_dist = config.filament_tower_interface_pre_extrusion_dist.get_at(idx);
        m_filpar[idx].filament_tower_interface_pre_extrusion_length = config.filament_tower_interface_pre_extrusion_length.get_at(idx);
    } else {
        m_filpar[idx].filament_tower_interface_print_temp = config.nozzle_temperature.get_at(idx);
        m_filpar[idx].filament_tower_interface_pre_extrusion_dist = 0.f;
        m_filpar[idx].filament_tower_interface_pre_extrusion_length = 0.f;
    }
}



// Returns gcode to prime the nozzles at the front edge of the print bed.
std::vector<WipeTower::ToolChangeResult> WipeTower::prime(
	// print_z of the first layer.
	float 						initial_layer_print_height,
	// Extruder indices, in the order to be primed. The last extruder will later print the wipe tower brim, print brim and the object.
	const std::vector<unsigned int> &tools,
	// If true, the last priming are will be the same as the other priming areas, and the rest of the wipe will be performed inside the wipe tower.
	// If false, the last priming are will be large enough to wipe the last extruder sufficiently.
    bool 						/*last_wipe_inside_wipe_tower*/)
{
    return std::vector<ToolChangeResult>();
}

Vec2f WipeTower::get_next_pos(const WipeTower::box_coordinates &cleaning_box, float wipe_length, bool solid_toolchange)
{
    const float &xl = cleaning_box.ld.x();
    const float &xr = cleaning_box.rd.x();
    int line_count = wipe_length / (xr - xl);

    // In mixed-nozzle modes the predicted pitch must match what toolchange_wipe_new() steps by
    // (purge_width, no extra_spacing on base layers or solid tool changes). Off mode keeps upstream.
    float dy = m_mixed_nozzle_slicing
        ? (is_base_layer() ? purge_width(int(m_current_tool))
                           : m_layer_info->extra_spacing * purge_width(int(m_current_tool)))
        : m_layer_info->extra_spacing * get_block_gap_width(int(m_current_tool), false);
    if (m_mixed_nozzle_slicing && solid_toolchange)
        dy = purge_width(int(m_current_tool));
    float y_offset = float(line_count) * dy;
    const Vec2f pos_offset = Vec2f(0.f, m_depth_traversed);

    Vec2f res;
    int   index = m_cur_layer_id % 4;
    //Vec2f offset = m_use_gap_wall ? Vec2f(5 * m_perimeter_width, 0) : Vec2f{0, 0};
    Vec2f offset = Vec2f{0, 0};
    switch (index % 4) {
    case 0:
        res = offset +cleaning_box.ld + pos_offset;
        break;
    case 1:
        res = -offset +cleaning_box.rd + pos_offset + Vec2f(0, y_offset);
        break;
    case 2:
        res = -offset+ cleaning_box.rd + pos_offset;
        break;
    case 3:
        res = offset+cleaning_box.ld + pos_offset + Vec2f(0, y_offset);
        break;
    default: break;
    }
    bool is_contact_pre_extrusion = solid_toolchange && m_enable_tower_interface_features;
    bool is_petg_pre_extrusion   = !is_contact_pre_extrusion && is_petg_filament(m_current_tool) && m_has_filament_switcher;
    if (is_contact_pre_extrusion || is_petg_pre_extrusion) {
        Vec2f        stop_pos                                    = res;
        float        filament_tower_interface_pre_extrusion_dist = is_petg_pre_extrusion
                                                                       ? m_filpar[m_current_tool].filament_petg_pre_extrusion_offset_dist
                                                                       : m_filpar[m_current_tool].filament_tower_interface_pre_extrusion_dist;
        // Orca: unscaled(BoundingBox) here is a template returning BoundingBoxBase<Vec2d>, not BoundingBoxf
        auto         printer_bbx                                 = unscaled(get_extents(m_shared_print_bed));
        printer_bbx.translate((-m_wipe_tower_pos - m_rib_offset).cast<double>()); // first layer never be contact
        if (stop_pos.x() < m_wipe_tower_width / 2.f)
            stop_pos = Vec2f(stop_pos.x() - filament_tower_interface_pre_extrusion_dist, stop_pos.y());
        else
            stop_pos = Vec2f(stop_pos.x() + filament_tower_interface_pre_extrusion_dist, stop_pos.y());
        if (stop_pos.x() < printer_bbx.min[0]) stop_pos.x() = printer_bbx.min[0];
        if (stop_pos.x() > printer_bbx.max[0]) stop_pos.x() = printer_bbx.max[0];
        res = stop_pos;
    }
    return res;
}

WipeTower::ToolChangeResult WipeTower::tool_change(size_t tool, bool extrude_perimeter, bool first_toolchange_to_nonsoluble)
{
    //only for tool = unsigned (-1) ,never get here
    //m_nozzle_change_result.gcode.clear();
    //if (!m_filament_map.empty() && tool < m_filament_map.size() && m_filament_map[m_current_tool] != m_filament_map[tool]) {
    //    m_nozzle_change_result = nozzle_change(m_current_tool, tool);
    //}

    size_t old_tool = m_current_tool;

    float wipe_depth = 0.f;
	float wipe_length = 0.f;
    float purge_volume = 0.f;
    float nozzle_change_depth = 0.f;
	// Finds this toolchange info
	if (tool != (unsigned int)(-1))
	{
		for (const auto &b : m_layer_info->tool_changes)
			if ( b.new_tool == tool ) {
                wipe_length = b.wipe_length;
                wipe_depth = b.required_depth;
                purge_volume = b.purge_volume;
                nozzle_change_depth = b.nozzle_change_depth;
				break;
			}
	}
	else {
		// Otherwise we are going to Unload only. And m_layer_info would be invalid.
	}

    box_coordinates cleaning_box(
		Vec2f(m_perimeter_width, m_perimeter_width),
		m_wipe_tower_width - 2 * m_perimeter_width,
        (tool != (unsigned int)(-1) ? wipe_depth + m_depth_traversed - m_perimeter_width
                                    : m_wipe_tower_depth - m_perimeter_width));

	WipeTowerWriter writer(m_layer_height, m_perimeter_width, m_gcode_flavor, m_filpar, m_enable_arc_fitting, m_travel_speed, m_lag_emit_block_z);
	writer.set_extrusion_flow(m_extrusion_flow)
		.set_z(m_z_pos)
		.set_initial_tool(m_current_tool)
        .set_y_shift(m_y_shift + (tool!=(unsigned int)(-1) && (m_current_shape == SHAPE_REVERSED) ? m_layer_info->depth - m_layer_info->toolchanges_depth(): 0.f))
		.append(";--------------------\n"
				"; CP TOOLCHANGE START\n")
		.comment_with_value(" toolchange #", m_num_tool_changes + 1); // the number is zero-based

    set_for_wipe_tower_writer(writer);

    if (tool != (unsigned)(-1))
        writer.append(std::string("; material : " + (m_current_tool < m_filpar.size() ? m_filpar[m_current_tool].material : "(NONE)") + " -> " + m_filpar[tool].material + "\n").c_str())
              .append(";--------------------\n");

    writer.speed_override_backup();
    writer.speed_override(100);

    float feedrate = is_first_layer() ? std::min(first_layer_speed() * 60.f, 5400.f) : std::min(60.0f * m_filpar[m_current_tool].max_e_speed / m_extrusion_flow, 5400.f);

    // Increase the extruder driver current to allow fast ramming.
    //BBS
	//if (m_set_extruder_trimpot)
	//	writer.set_extruder_trimpot(750);

    // Ram the hot material out of the melt zone, retract the filament into the cooling tubes and let it cool.
    if (tool != (unsigned int)-1){ 			// This is not the last change.
        writer.append_wipe_tower_start();
        toolchange_Unload(writer, cleaning_box, m_filpar[m_current_tool].material,
                          is_base_layer() ? m_filpar[tool].nozzle_temperature_initial_layer : m_filpar[tool].nozzle_temperature);
        toolchange_Change(writer, tool, m_filpar[tool].material); // Change the tool, set a speed override for soluble and flex materials.
        toolchange_Load(writer, cleaning_box);
        // BBS
        //writer.travel(writer.x(), writer.y()-m_perimeter_width); // cooling and loading were done a bit down the road

        if (m_is_multi_extruder && is_tpu_filament(tool)) {
            float dy                  = 2 * m_perimeter_width;
            float nozzle_change_speed = 60.0f * m_filpar[tool].max_e_speed / m_extrusion_flow;
            nozzle_change_speed *= 0.25;

            const float &xl = cleaning_box.ld.x();
            const float &xr = cleaning_box.rd.x();

            Vec2f start_pos = m_nozzle_change_result.start_pos + Vec2f(0, m_perimeter_width);
            bool   left_to_right     = true;
            double tpu_travel_length = 5;
            double e_flow            = extrusion_flow(m_layer_height);
            double length            = tpu_travel_length / e_flow;
            int    tpu_line_count    = length / (m_wipe_tower_width - 2 * m_perimeter_width) + 1;

            writer.travel(start_pos);

            for (int i = 0; true; ++i) {
                if (left_to_right)
                    writer.travel(xr - m_perimeter_width, writer.y(), nozzle_change_speed);
                else
                    writer.travel(xl + m_perimeter_width, writer.y(), nozzle_change_speed);

                if (i == tpu_line_count - 1)
                    break;

                writer.travel(writer.x(), writer.y() + dy);
                left_to_right = !left_to_right;
            }
        }

        Vec2f initial_position = get_next_pos(cleaning_box, wipe_length,false);
        writer.set_initial_position(initial_position, m_wipe_tower_width, m_wipe_tower_depth, m_internal_rotation);

        if (extrude_perimeter) {
            box_coordinates wt_box(Vec2f(0.f, (m_current_shape == SHAPE_REVERSED) ? m_layer_info->toolchanges_depth() - m_layer_info->depth : 0.f), m_wipe_tower_width,
                                   m_layer_info->depth + m_perimeter_width);

            // align the perimeter
            Vec2f pos = initial_position;
            switch (m_cur_layer_id % 4){
            case 0:
                pos = wt_box.ld;
                break;
            case 1:
                pos = wt_box.rd;
                break;
            case 2:
                pos = wt_box.ru;
                break;
            case 3:
                pos = wt_box.lu;
                break;
            default: break;
            }
            writer.set_initial_position(pos, m_wipe_tower_width, m_wipe_tower_depth, m_internal_rotation);

            wt_box = align_perimeter(wt_box);
            writer.rectangle(wt_box, feedrate);
        }

        writer.travel(initial_position);

        toolchange_Wipe(writer, cleaning_box, wipe_length);     // Wipe the newly loaded filament until the end of the assigned wipe area.

        writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_End) + "\n");
        ++ m_num_tool_changes;
    } else
        toolchange_Unload(writer, cleaning_box, m_filpar[m_current_tool].material, m_filpar[m_current_tool].nozzle_temperature);

    m_depth_traversed += (wipe_depth - nozzle_change_depth);

    //BBS
	//if (m_set_extruder_trimpot)
	//	writer.set_extruder_trimpot(550);    // Reset the extruder current to a normal value.
    writer.speed_override_restore();
    writer.feedrate(m_travel_speed * 60.f)
          .flush_planner_queue()
          .reset_extruder()
          .append("; CP TOOLCHANGE END\n"
                  ";------------------\n"
                  "\n\n");

    // Ask our writer about how much material was consumed:
    if (m_current_tool < m_used_filament_length.size())
        m_used_filament_length[m_current_tool] += writer.get_and_reset_used_filament_length();

    return construct_tcr(writer, false, old_tool, false, true, purge_volume,false);
}
#if 0
WipeTower::NozzleChangeResult WipeTower::nozzle_change(int old_filament_id, int new_filament_id)
{
    float wipe_depth               = 0.f;
    float wipe_length              = 0.f;
    float purge_volume             = 0.f;
    int   nozzle_change_line_count = 0;

    // Finds this toolchange info
    if (new_filament_id != (unsigned int) (-1)) {
        for (const auto &b : m_layer_info->tool_changes)
            if (b.new_tool == new_filament_id) {
                wipe_length              = b.wipe_length;
                wipe_depth               = b.required_depth;
                purge_volume             = b.purge_volume;
                if (has_tpu_filament())
                    nozzle_change_line_count = ((b.nozzle_change_depth + WT_EPSILON) / m_nozzle_change_perimeter_width) / 2;
                else
                    nozzle_change_line_count = (b.nozzle_change_depth + WT_EPSILON) / m_nozzle_change_perimeter_width;
                break;
            }
    } else {
        // Otherwise we are going to Unload only. And m_layer_info would be invalid.
    }

    auto format_nozzle_change_line = [](bool start, int old_filament_id, int new_filament_id)->std::string {
        char buff[64];
        std::string tag = start ? GCodeProcessor::reserved_tag(GCodeProcessor::ETags::NozzleChangeStart) : GCodeProcessor::reserved_tag(GCodeProcessor::ETags::NozzleChangeEnd);
        snprintf(buff, sizeof(buff), ";%s OF%d NF%d\n", tag.c_str(), old_filament_id, new_filament_id);
        return std::string(buff);
        };

    float nozzle_change_speed = 60.0f * m_filpar[m_current_tool].max_e_speed / m_extrusion_flow;
    if (is_tpu_filament(m_current_tool)) {
        nozzle_change_speed *= 0.25;
    }

    WipeTowerWriter writer(m_layer_height, m_perimeter_width, m_gcode_flavor, m_filpar, m_enable_arc_fitting);
    writer.set_extrusion_flow(m_extrusion_flow)
        .set_z(m_z_pos)
        .set_initial_tool(m_current_tool)
        .set_extrusion_flow(m_extrusion_flow)
        .set_y_shift(m_y_shift + (new_filament_id != (unsigned int) (-1) && (m_current_shape == SHAPE_REVERSED) ? m_layer_info->depth - m_layer_info->toolchanges_depth() : 0.f))
        .append(format_nozzle_change_line(true,old_filament_id,new_filament_id));

    set_for_wipe_tower_writer(writer);

    box_coordinates cleaning_box(Vec2f(m_perimeter_width, m_perimeter_width), m_wipe_tower_width - 2 * m_perimeter_width,
                                 (new_filament_id != (unsigned int) (-1) ? wipe_depth + m_depth_traversed - m_perimeter_width : m_wipe_tower_depth - m_perimeter_width));

    Vec2f initial_position = cleaning_box.ld + Vec2f(0.f, m_depth_traversed);
    writer.set_initial_position(initial_position, m_wipe_tower_width, m_wipe_tower_depth, m_internal_rotation);

    const float &xl = cleaning_box.ld.x();
    const float &xr = cleaning_box.rd.x();

    float dy = m_layer_info->extra_spacing * m_perimeter_width;
    if (has_tpu_filament())
        dy = 2 * m_perimeter_width;

    float start_y = writer.y();

    m_left_to_right = true;

    bool need_change_flow = false;
    // now the wiping itself:
    for (int i = 0; true; ++i) {
        if (m_left_to_right)
            writer.extrude(xr + wipe_tower_wall_infill_overlap * m_perimeter_width, writer.y(), nozzle_change_speed);
        else
            writer.extrude(xl - wipe_tower_wall_infill_overlap * m_perimeter_width, writer.y(), nozzle_change_speed);

        if (writer.y() - float(EPSILON) > cleaning_box.lu.y())
            break; // in case next line would not fit

        if (i == nozzle_change_line_count - 1)
            break;

        // stepping to the next line:
        writer.extrude(writer.x(), writer.y() + dy);
        m_left_to_right = !m_left_to_right;
    }

    writer.set_extrusion_flow(m_extrusion_flow); // Reset the extrusion flow.

    m_depth_traversed += nozzle_change_line_count * dy;

    NozzleChangeResult result;

    if (is_tpu_filament(m_current_tool))
    {
        bool left_to_right = !m_left_to_right;
        double tpu_travel_length        = 5;
        double e_flow                   = extrusion_flow(m_layer_height);
        double length                   = tpu_travel_length / e_flow;
        int    tpu_line_count = length / (m_wipe_tower_width - 2 * m_perimeter_width) + 1;

        writer.travel(writer.x(), writer.y() - m_perimeter_width);

        for (int i = 0; true; ++i) {
            if (left_to_right)
                writer.travel(xr - m_perimeter_width, writer.y(), nozzle_change_speed);
            else
                writer.travel(xl + m_perimeter_width, writer.y(), nozzle_change_speed);

            if (i == tpu_line_count - 1)
                break;

            writer.travel(writer.x(), writer.y() - dy);
            left_to_right = !left_to_right;
        }
    }
    else {
        result.wipe_path.push_back(writer.pos());
        if (m_left_to_right) {
             result.wipe_path.push_back(Vec2f(0, writer.y()));
        } else {
             result.wipe_path.push_back(Vec2f(m_wipe_tower_width, writer.y()));
        }
    }

    writer.append(format_nozzle_change_line(false, old_filament_id, new_filament_id));

    result.start_pos = writer.start_pos_rotated();
    result.end_pos   = writer.pos();
    result.gcode     = writer.gcode();
    return result;
}
#endif
// Ram the hot material out of the melt zone, retract the filament into the cooling tubes and let it cool.
void WipeTower::toolchange_Unload(
	WipeTowerWriter &writer,
	const box_coordinates 	&cleaning_box,
	const std::string&		 current_material,
	const int 				 new_temperature)
{
    // BBS: toolchange unload is done in change_filament_gcode
#if 0
	float xl = cleaning_box.ld.x() + 1.f * m_perimeter_width;
	float xr = cleaning_box.rd.x() - 1.f * m_perimeter_width;

	const float line_width = m_perimeter_width * m_filpar[m_current_tool].ramming_line_width_multiplicator;       // desired ramming line thickness
	const float y_step = line_width * m_filpar[m_current_tool].ramming_step_multiplicator * m_extra_spacing; // spacing between lines in mm

    writer.append("; CP TOOLCHANGE UNLOAD\n")
        .change_analyzer_line_width(line_width);

	unsigned i = 0;										// iterates through ramming_speed
	m_left_to_right = true;								// current direction of ramming
	float remaining = xr - xl ;							// keeps track of distance to the next turnaround
	float e_done = 0;									// measures E move done from each segment

	writer.travel(xl, cleaning_box.ld.y() + m_depth_traversed + y_step/2.f ); // move to starting position

    // if the ending point of the ram would end up in mid air, align it with the end of the wipe tower:
    if (m_layer_info > m_plan.begin() && m_layer_info < m_plan.end() && (m_layer_info-1!=m_plan.begin() || !m_adhesion )) {

        // this is y of the center of previous sparse infill border
        float sparse_beginning_y = 0.f;
        if (m_current_shape == SHAPE_REVERSED)
            sparse_beginning_y += ((m_layer_info-1)->depth - (m_layer_info-1)->toolchanges_depth())
                                      - ((m_layer_info)->depth-(m_layer_info)->toolchanges_depth()) ;
        else
            sparse_beginning_y += (m_layer_info-1)->toolchanges_depth() + m_perimeter_width;

        float sum_of_depths = 0.f;
        for (const auto& tch : m_layer_info->tool_changes) {  // let's find this toolchange
            if (tch.old_tool == m_current_tool) {
                sum_of_depths += tch.ramming_depth;
                float ramming_end_y = sum_of_depths;
                ramming_end_y -= (y_step/m_extra_spacing-m_perimeter_width) / 2.f;   // center of final ramming line

                if ( (m_current_shape == SHAPE_REVERSED   && ramming_end_y < sparse_beginning_y - 0.5f*m_perimeter_width  ) ||
                     (m_current_shape == SHAPE_NORMAL && ramming_end_y > sparse_beginning_y + 0.5f*m_perimeter_width  )  )
                {
                    writer.extrude(xl + tch.first_wipe_line-1.f*m_perimeter_width,writer.y());
                    remaining -= tch.first_wipe_line-1.f*m_perimeter_width;
                }
                break;
            }
            sum_of_depths += tch.required_depth;
        }
    }

    writer.disable_linear_advance();

    // now the ramming itself:
    while (i < m_filpar[m_current_tool].ramming_speed.size())
    {
        const float x = volume_to_length(m_filpar[m_current_tool].ramming_speed[i] * 0.25f, line_width, m_layer_height);
        const float e = m_filpar[m_current_tool].ramming_speed[i] * 0.25f / filament_area(); // transform volume per sec to E move;
        const float dist = std::min(x - e_done, remaining);		  // distance to travel for either the next 0.25s, or to the next turnaround
        const float actual_time = dist/x * 0.25f;
        writer.ram(writer.x(), writer.x() + (m_left_to_right ? 1.f : -1.f) * dist, 0.f, 0.f, e * (dist / x), dist / (actual_time / 60.f));
        remaining -= dist;

		if (remaining < WT_EPSILON)	{ // we reached a turning point
			writer.travel(writer.x(), writer.y() + y_step, 7200);
			m_left_to_right = !m_left_to_right;
			remaining = xr - xl;
		}
		e_done += dist; // subtract what was actually done
		if (e_done > x - WT_EPSILON) { // current segment finished
			++i;
			e_done = 0;
		}
	}
	Vec2f end_of_ramming(writer.x(),writer.y());
    writer.change_analyzer_line_width(m_perimeter_width);   // so the next lines are not affected by ramming_line_width_multiplier

    // Retraction:
    float old_x = writer.x();
    float turning_point = (!m_left_to_right ? xl : xr );
    if (m_semm && (m_cooling_tube_retraction != 0 || m_cooling_tube_length != 0)) {
        float total_retraction_distance = m_cooling_tube_retraction + m_cooling_tube_length/2.f - 15.f; // the 15mm is reserved for the first part after ramming
        writer.suppress_preview()
              .retract(15.f, m_filpar[m_current_tool].unloading_speed_start * 60.f) // feedrate 5000mm/min = 83mm/s
              .retract(0.70f * total_retraction_distance, 1.0f * m_filpar[m_current_tool].unloading_speed * 60.f)
              .retract(0.20f * total_retraction_distance, 0.5f * m_filpar[m_current_tool].unloading_speed * 60.f)
              .retract(0.10f * total_retraction_distance, 0.3f * m_filpar[m_current_tool].unloading_speed * 60.f)
              .resume_preview();
    }
    // Wipe tower should only change temperature with single extruder MM. Otherwise, all temperatures should
    // be already set and there is no need to change anything. Also, the temperature could be changed
    // for wrong extruder.
    if (m_semm) {
        if (new_temperature != 0 && (new_temperature != m_old_temperature || is_first_layer()) ) { 	// Set the extruder temperature, but don't wait.
            // If the required temperature is the same as last time, don't emit the M104 again (if user adjusted the value, it would be reset)
            // However, always change temperatures on the first layer (this is to avoid issues with priming lines turned off).
            writer.set_extruder_temp(new_temperature, false);
            m_old_temperature = new_temperature;
        }
    }

    // Cooling:
    const int& number_of_moves = m_filpar[m_current_tool].cooling_moves;
    if (number_of_moves > 0) {
        const float& initial_speed = m_filpar[m_current_tool].cooling_initial_speed;
        const float& final_speed   = m_filpar[m_current_tool].cooling_final_speed;

        float speed_inc = (final_speed - initial_speed) / (2.f * number_of_moves - 1.f);

        writer.suppress_preview()
              .travel(writer.x(), writer.y() + y_step);
        old_x = writer.x();
        turning_point = xr-old_x > old_x-xl ? xr : xl;
        for (int i=0; i<number_of_moves; ++i) {
            float speed = initial_speed + speed_inc * 2*i;
            writer.load_move_x_advanced(turning_point, m_cooling_tube_length, speed);
            speed += speed_inc;
            writer.load_move_x_advanced(old_x, -m_cooling_tube_length, speed);
        }
    }

    // let's wait is necessary:
    writer.wait(m_filpar[m_current_tool].delay);
    // we should be at the beginning of the cooling tube again - let's move to parking position:
    writer.retract(-m_cooling_tube_length/2.f+m_parking_pos_retraction-m_cooling_tube_retraction, 2000);

	// this is to align ramming and future wiping extrusions, so the future y-steps can be uniform from the start:
    // the perimeter_width will later be subtracted, it is there to not load while moving over just extruded material
	writer.travel(end_of_ramming.x(), end_of_ramming.y() + (y_step/m_extra_spacing-m_perimeter_width) / 2.f + m_perimeter_width, 2400.f);

	writer.resume_preview()
		  .flush_planner_queue();
#endif
}

// Change the tool, set a speed override for soluble and flex materials.
void WipeTower::toolchange_Change(
	WipeTowerWriter &writer,
    const size_t 	new_tool,
    const std::string&  new_material)
{
    // Ask the writer about how much of the old filament we consumed:
    if (m_current_tool < m_used_filament_length.size())
    	m_used_filament_length[m_current_tool] += writer.get_and_reset_used_filament_length();

    // This is where we want to place the custom gcodes. We will use placeholders for this.
    // These will be substituted by the actual gcodes when the gcode is generated.
    writer.append("[filament_end_gcode]\n");
    writer.append("[change_filament_gcode]\n");

    // BBS: do travel in GCode::append_tcr() for lazy_lift
#if 0
    // Travel to where we assume we are. Custom toolchange or some special T code handling (parking extruder etc)
    // gcode could have left the extruder somewhere, we cannot just start extruding. We should also inform the
    // postprocessor that we absolutely want to have this in the gcode, even if it thought it is the same as before.
    Vec2f current_pos = writer.pos_rotated();
    writer.feedrate(m_travel_speed * 60.f)
          .append(std::string("G1 X") + Slic3r::float_to_string_decimal_point(current_pos.x())
                             +  " Y"  + Slic3r::float_to_string_decimal_point(current_pos.y())
                             + never_skip_tag() + "\n");
#endif

    // The toolchange Tn command will be inserted later, only in case that the user does
    // not provide a custom toolchange gcode.
	writer.set_tool(new_tool); // This outputs nothing, the writer just needs to know the tool has changed.
    writer.append("[filament_start_gcode]\n");

	writer.flush_planner_queue();
	m_current_tool = new_tool;
}

void WipeTower::toolchange_Load(
	WipeTowerWriter &writer,
	const box_coordinates  &cleaning_box)
{
    // BBS: tool load is done in change_filament_gcode
#if 0
    if (m_semm && (m_parking_pos_retraction != 0 || m_extra_loading_move != 0)) {
        float xl = cleaning_box.ld.x() + m_perimeter_width * 0.75f;
        float xr = cleaning_box.rd.x() - m_perimeter_width * 0.75f;
        float oldx = writer.x();	// the nozzle is in place to do the first wiping moves, we will remember the position

        // Load the filament while moving left / right, so the excess material will not create a blob at a single position.
        float turning_point = ( oldx-xl < xr-oldx ? xr : xl );
        float edist = m_parking_pos_retraction+m_extra_loading_move;

        writer.append("; CP TOOLCHANGE LOAD\n")
              .suppress_preview()
              .load(0.2f * edist, 60.f * m_filpar[m_current_tool].loading_speed_start)
              .load_move_x_advanced(turning_point, 0.7f * edist,        m_filpar[m_current_tool].loading_speed)  // Fast phase
              .load_move_x_advanced(oldx,          0.1f * edist, 0.1f * m_filpar[m_current_tool].loading_speed)  // Super slow*/

              .travel(oldx, writer.y()) // in case last move was shortened to limit x feedrate
              .resume_preview();

        // Reset the extruder current to the normal value.
        if (m_set_extruder_trimpot)
            writer.set_extruder_trimpot(550);
    }
#endif
}

// Wipe the newly loaded filament until the end of the assigned wipe area.
void WipeTower::toolchange_Wipe(
	WipeTowerWriter &writer,
	const box_coordinates  &cleaning_box,
	float wipe_length)
{
	// Increase flow on first layer, slow down print.
    // Keep the declared width consistent with the capped flow boost.
    const float wipe_flow_ratio = is_first_layer() ? base_flow_ratio_for_width(int(m_current_tool), m_perimeter_width) : 1.f;
    writer.set_extrusion_flow(m_extrusion_flow * wipe_flow_ratio)
		  .append("; CP TOOLCHANGE WIPE\n");

    // BBS: add the note for gcode-check, when the flow changed, the width should follow the change
    if (is_first_layer()) {
        writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Width) + std::to_string(wipe_flow_ratio * m_perimeter_width) + "\n");
    }

	const float& xl = cleaning_box.ld.x();
	const float& xr = cleaning_box.rd.x();

	// Variables x_to_wipe and traversed_x are here to be able to make sure it always wipes at least
    //   the ordered volume, even if it means violating the box. This can later be removed and simply
    // wipe until the end of the assigned area.

    float x_to_wipe = wipe_length;
    float dy = m_layer_info->extra_spacing * m_perimeter_width;

    const float target_speed = is_first_layer() ? std::min(first_layer_speed() * 60.f, 4800.f) : 4800.f;
    float wipe_speed = 0.33f * target_speed;

    float start_y = writer.y();

#if 0
    // if there is less than 2.5*m_perimeter_width to the edge, advance straightaway (there is likely a blob anyway)
    if ((m_left_to_right ? xr-writer.x() : writer.x()-xl) < 2.5f*m_perimeter_width) {
        writer.travel((m_left_to_right ? xr-m_perimeter_width : xl+m_perimeter_width),writer.y()+dy);
        m_left_to_right = !m_left_to_right;
    }
#endif

    m_left_to_right = ((m_cur_layer_id + 3) % 4 >= 2);
    bool is_from_up = (m_cur_layer_id % 2 == 1);

    // BBS: do not need to move dy
#if 0
    if (m_depth_traversed != 0)
        writer.travel(xl, writer.y() + dy);
#endif

    bool need_change_flow = false;
    // now the wiping itself:
	for (int i = 0; true; ++i)	{
		if (i!=0) {
            if      (wipe_speed < 0.34f * target_speed) wipe_speed = 0.375f * target_speed;
            else if (wipe_speed < 0.377 * target_speed) wipe_speed = 0.458f * target_speed;
            else if (wipe_speed < 0.46f * target_speed) wipe_speed = 0.875f * target_speed;
            else wipe_speed = std::min(target_speed, wipe_speed + 50.f);
		}

        // BBS: check the bridging area and use the bridge flow
        if (need_change_flow || need_thick_bridge_flow(writer.y())) {
            writer.set_extrusion_flow(extrusion_flow(0.2));
            writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + std::to_string(0.2) + "\n");
            need_change_flow = true;
        }

        if (m_left_to_right)
            writer.extrude(xr + wipe_tower_wall_infill_overlap * m_perimeter_width, writer.y(), wipe_speed);
        else
            writer.extrude(xl - wipe_tower_wall_infill_overlap * m_perimeter_width, writer.y(), wipe_speed);

        // BBS: recover the flow in non-bridging area
        if (need_change_flow) {
            writer.set_extrusion_flow(m_extrusion_flow);
            writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + std::to_string(m_layer_height) + "\n");
        }

        if (!is_from_up && (writer.y() - float(EPSILON) > cleaning_box.lu.y()))
            break;		// in case next line would not fit

        if (is_from_up && (writer.y() + float(EPSILON) < cleaning_box.ld.y()))
            break;

        x_to_wipe -= (xr - xl);
		if (x_to_wipe < WT_EPSILON) {
            // BBS: Delete some unnecessary travel
            //writer.travel(m_left_to_right ? xl + 1.5f*m_perimeter_width : xr - 1.5f*m_perimeter_width, writer.y(), 7200);
			break;
		}
		// stepping to the next line:
        if (is_from_up)
            writer.extrude(writer.x(), writer.y() - dy);
        else
            writer.extrude(writer.x(), writer.y() + dy);

		m_left_to_right = !m_left_to_right;
	}

    float end_y = writer.y();

    // We may be going back to the model - wipe the nozzle. If this is followed
    // by finish_layer, this wipe path will be overwritten.
    //writer.add_wipe_point(writer.x(), writer.y())
    //      .add_wipe_point(writer.x(), writer.y() - dy)
    //      .add_wipe_point(! m_left_to_right ? m_wipe_tower_width : 0.f, writer.y() - dy);
    // BBS: modify the wipe_path after toolchange
    writer.add_wipe_point(writer.x(), writer.y())
          .add_wipe_point(! m_left_to_right ? m_wipe_tower_width : 0.f, writer.y());

    if (m_layer_info != m_plan.end() && m_current_tool != m_layer_info->tool_changes.back().new_tool)
        m_left_to_right = !m_left_to_right;

    writer.set_extrusion_flow(m_extrusion_flow); // Reset the extrusion flow.
    // BBS: add the note for gcode-check when the flow changed
    if (is_first_layer()) {
        writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Width) + std::to_string(m_perimeter_width) + "\n");
    }
}



// BBS
WipeTower::box_coordinates WipeTower::align_perimeter(const WipeTower::box_coordinates& perimeter_box)
{
    box_coordinates aligned_box = perimeter_box;

    float spacing = m_extra_spacing * m_perimeter_width;
    float up = perimeter_box.lu(1) - m_perimeter_width - EPSILON;
    up = align_ceil(up, spacing);
    up += m_perimeter_width;
    up = std::min(up, m_wipe_tower_depth);

    float down = perimeter_box.ld(1) - m_perimeter_width + EPSILON;
    down = align_floor(down, spacing);
    down += m_perimeter_width;
    down = std::max(down, -m_y_shift);

    aligned_box.lu(1) = aligned_box.ru(1) = up;
    aligned_box.ld(1) = aligned_box.rd(1) = down;

    return aligned_box;
}

void WipeTower::set_for_wipe_tower_writer(WipeTowerWriter &writer)
{
    writer.set_normal_acceleration(m_normal_accels);
    writer.set_travel_acceleration(m_travel_accels);
    writer.set_first_layer_normal_acceleration(m_first_layer_normal_accels);
    writer.set_first_layer_travel_acceleration(m_first_layer_travel_accels);
    writer.set_max_acceleration(m_max_accels);
    writer.set_multi_nozzle_group_result(m_multi_nozzle_group_result);
    writer.set_accel_to_decel_enable(m_accel_to_decel_enable);
    writer.set_accel_to_decel_factor(m_accel_to_decel_factor);
    writer.set_first_layer(m_cur_layer_id == 0);
    writer.set_layer_id(m_cur_layer_id);
    writer.set_physical_extruder_map(m_physical_extruder_map);
    writer.set_mixed_nozzle_slicing(m_mixed_nozzle_slicing);
}
#if 0
WipeTower::ToolChangeResult WipeTower::finish_layer(bool extrude_perimeter, bool extruder_fill)
{
	assert(! this->layer_finished());
    m_current_layer_finished = true;

    size_t old_tool = m_current_tool;

	WipeTowerWriter writer(m_layer_height, m_perimeter_width, m_gcode_flavor, m_filpar, m_enable_arc_fitting);
	writer.set_extrusion_flow(m_extrusion_flow)
		.set_z(m_z_pos)
		.set_initial_tool(m_current_tool)
        .set_y_shift(m_y_shift - (m_current_shape == SHAPE_REVERSED ? m_layer_info->toolchanges_depth() : 0.f));

    set_for_wipe_tower_writer(writer);

    writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_Start) + "\n");

	// Slow down on the 1st layer.
    bool first_layer = is_first_layer();
    // BBS: speed up perimeter speed to 90mm/s for non-first layer
    float           feedrate   = first_layer ? std::min(m_first_layer_speed * 60.f, 5400.f) : std::min(60.0f * m_filpar[m_current_tool].max_e_speed / m_extrusion_flow, 5400.f);
    float fill_box_y = m_layer_info->toolchanges_depth() + m_perimeter_width;
    box_coordinates fill_box(Vec2f(m_perimeter_width, fill_box_y),
                             m_wipe_tower_width - 2 * m_perimeter_width, m_layer_info->depth - fill_box_y);

    writer.set_initial_position((m_left_to_right ? fill_box.ru : fill_box.lu), // so there is never a diagonal travel
                                 m_wipe_tower_width, m_wipe_tower_depth, m_internal_rotation);

    bool toolchanges_on_layer = m_layer_info->toolchanges_depth() > WT_EPSILON;

    // inner perimeter of the sparse section, if there is space for it:
    if (fill_box.ru.y() - fill_box.rd.y() > m_perimeter_width - WT_EPSILON)
        writer.rectangle_fill_box(this, fill_box.ld, fill_box.rd.x() - fill_box.ld.x(), fill_box.ru.y() - fill_box.rd.y(), feedrate);

    // we are in one of the corners, travel to ld along the perimeter:
    // BBS: Delete some unnecessary travel
    //if (writer.x() > fill_box.ld.x() + EPSILON) writer.travel(fill_box.ld.x(), writer.y());
    //if (writer.y() > fill_box.ld.y() + EPSILON) writer.travel(writer.x(), fill_box.ld.y());

    // Extrude infill to support the material to be printed above.
    const float dy = (fill_box.lu.y() - fill_box.ld.y() - m_perimeter_width);
    float left = fill_box.lu.x() + 2*m_perimeter_width;
    float right = fill_box.ru.x() - 2 * m_perimeter_width;
    std::vector<Vec2f> finish_rect_wipe_path;
    if (extruder_fill && dy > m_perimeter_width)
    {
        writer.travel(fill_box.ld + Vec2f(m_perimeter_width * 2, 0.f))
              .append(";--------------------\n"
                      "; CP EMPTY GRID START\n")
              .comment_with_value(" layer #", m_num_layer_changes + 1);

        // Is there a soluble filament wiped/rammed at the next layer?
        // If so, the infill should not be sparse.
        bool solid_infill = m_layer_info+1 == m_plan.end()
                          ? false
                          : std::any_of((m_layer_info+1)->tool_changes.begin(),
                                        (m_layer_info+1)->tool_changes.end(),
                                   [this](const WipeTowerInfo::ToolChange& tch) {
                                       return m_filpar[tch.new_tool].is_soluble
                                           || m_filpar[tch.old_tool].is_soluble;
                                   });
        solid_infill |= first_layer && m_adhesion;

        if (solid_infill) {
            float sparse_factor = 1.5f; // 1=solid, 2=every other line, etc.
            if (first_layer) { // the infill should touch perimeters
                left  -= m_perimeter_width;
                right += m_perimeter_width;
                sparse_factor = 1.f;
            }
            float y = fill_box.ld.y() + m_perimeter_width;
            int n = dy / (m_perimeter_width * sparse_factor);
            float spacing = (dy-m_perimeter_width)/(n-1);
            int i=0;
            for (i=0; i<n; ++i) {
                writer.extrude(writer.x(), y, feedrate)
                      .extrude(i%2 ? left : right, y);
                y = y + spacing;
            }
            writer.extrude(writer.x(), fill_box.lu.y());
        } else {
            // Extrude an inverse U at the left of the region and the sparse infill.
            writer.extrude(fill_box.lu + Vec2f(m_perimeter_width * 2, 0.f), feedrate);

            const int n = 1+int((right-left)/m_bridging);
            const float dx = (right-left)/n;
            for (int i=1;i<=n;++i) {
                float x=left+dx*i;
                writer.travel(x,writer.y());
                writer.extrude(x,i%2 ? fill_box.rd.y() : fill_box.ru.y());
            }
            // BBS: add wipe_path for this case: only with finish rectangle
            finish_rect_wipe_path.emplace_back(writer.pos());
            finish_rect_wipe_path.emplace_back(Vec2f(left + dx * n, n % 2 ? fill_box.ru.y() : fill_box.rd.y()));
        }

        writer.append("; CP EMPTY GRID END\n"
                      ";------------------\n\n\n\n\n\n\n");
    }

    // outer perimeter (always):
    // BBS
    box_coordinates wt_box(Vec2f(0.f, (m_current_shape == SHAPE_REVERSED ? m_layer_info->toolchanges_depth() : 0.f)),
        m_wipe_tower_width, m_layer_info->depth + m_perimeter_width);
    wt_box = align_perimeter(wt_box);
    if (extrude_perimeter) {
        writer.rectangle(wt_box, feedrate);
    }

    // brim chamfer
    float spacing = m_perimeter_width - m_layer_height * float(1. - M_PI_4);
    // How many perimeters shall the brim have?
    int loops_num = (m_wipe_tower_brim_width + spacing / 2.f) / spacing;
    const float max_chamfer_width = 3.f;
    if (!first_layer) {
        // stop print chamfer if depth changes
        if (m_layer_info->depth != m_plan.front().depth) {
            loops_num = 0;
        }
        else {
            // limit max chamfer width to 3 mm
            int chamfer_loops_num = (int)(max_chamfer_width / spacing);
            int dist_to_1st = m_layer_info - m_plan.begin() - m_first_layer_idx;
            loops_num = std::min(loops_num, chamfer_loops_num) - dist_to_1st;
        }
    }

    if (loops_num > 0) {
        box_coordinates box = wt_box;
        for (size_t i = 0; i < loops_num; ++i) {
            box.expand(spacing);
            writer.rectangle(box, feedrate);
        }

        if (first_layer) {
            // Save actual brim width to be later passed to the Print object, which will use it
            // for skirt calculation and pass it to GLCanvas for precise preview box
            m_wipe_tower_brim_width_real = wt_box.ld.x() - box.ld.x() + spacing / 2.f;
        }
        wt_box = box;
    }

    // Now prepare future wipe. box contains rectangle that was extruded last (ccw).
    Vec2f target = (writer.pos() == wt_box.ld ? wt_box.rd :
                   (writer.pos() == wt_box.rd ? wt_box.ru :
                   (writer.pos() == wt_box.ru ? wt_box.lu :
                    wt_box.ld)));

    // BBS: add wipe_path for this case: only with finish rectangle
    if (finish_rect_wipe_path.size() == 2 && finish_rect_wipe_path[0] == writer.pos())
        target = finish_rect_wipe_path[1];

    writer.add_wipe_point(writer.pos())
          .add_wipe_point(target);

    writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_End) + "\n");

    // Ask our writer about how much material was consumed.
    // Skip this in case the layer is sparse and config option to not print sparse layers is enabled.
    if (! m_no_sparse_layers || toolchanges_on_layer)
        if (m_current_tool < m_used_filament_length.size())
            m_used_filament_length[m_current_tool] += writer.get_and_reset_used_filament_length();

    return construct_tcr(writer, false, old_tool, true, false, 0.f,false);
}
#endif
WipeTower::WipeTowerInfo::ToolChange WipeTower::set_toolchange(int old_tool, int new_tool, float layer_height, float wipe_volume, float purge_volume,int layer_id, float ram_length_override_mm, float wipe_volume_budget)
{
    float depth             = 0.f;
    float width             = m_wipe_tower_width - 2 * m_perimeter_width;
    float                             nozzle_change_width     = m_wipe_tower_width - (m_nozzle_change_perimeter_width + m_perimeter_width);
    // The purge is drawn by the incoming tool at purge_width(new_tool), matching toolchange_wipe_new().
    float length_to_extrude = volume_to_length(wipe_volume, purge_width(new_tool), layer_height);
    // In mixed-nozzle modes never reserve a pitch smaller than the one the emitter uses.
    const float toolchange_gap_width = m_mixed_nozzle_slicing
        ? std::max(get_block_gap_width(new_tool, false), purge_width(new_tool))
        : get_block_gap_width(new_tool, false);
    float                             nozzlechange_gap_width  = get_block_gap_width(old_tool,true);
    const bool is_extruder_change       = !is_same_extruder(old_tool, new_tool, layer_id);
    // A resolved outgoing-tool ram length replaces the vendor extruder-change length only; the
    // _nc keys do not apply to a two-extruder pair.
    float filament_change_length = (ram_length_override_mm >= 0.f && is_extruder_change)
        ? ram_length_override_mm
        : filament_vector_value(is_extruder_change ?
                                                         m_filaments_change_length.first : m_filaments_change_length.second,
                                                     size_t(old_tool));
    depth += std::ceil(length_to_extrude / width) * toolchange_gap_width;
    // depth *= m_extra_spacing;

    float nozzle_change_depth  = 0;
    float nozzle_change_length = 0;
    if (is_need_ramming(old_tool, new_tool, layer_id)) {
        // Sized at the height the outgoing tool actually rams at (ramming_height()).
        double e_flow                   = nozzle_change_extrusion_flow(ramming_height(old_tool, layer_height));
        double length                   = filament_change_length / e_flow;
        int    nozzle_change_line_count = std::ceil(length / nozzle_change_width);
        nozzle_change_depth             = nozzle_change_line_count * nozzlechange_gap_width;
        depth += nozzle_change_depth;
        nozzle_change_length = length;
    }
    WipeTowerInfo::ToolChange tool_change = WipeTowerInfo::ToolChange(old_tool, new_tool, depth, 0.f, 0.f, wipe_volume, length_to_extrude, purge_volume);
    tool_change.nozzle_change_depth       = nozzle_change_depth;
    tool_change.nozzle_change_length      = nozzle_change_length;
    tool_change.nozzle_change_budget      = filament_change_length;
    tool_change.ram_length_override_mm    = ram_length_override_mm;
    tool_change.wipe_volume_budget        = wipe_volume_budget >= 0.f ? wipe_volume_budget : wipe_volume;
    return tool_change;
}

// Appends a toolchange into m_plan and calculates neccessary depth of the corresponding box
void WipeTower::plan_toolchange(float z_par, float layer_height_par, unsigned int old_tool,
                                unsigned int new_tool, float wipe_volume_ec,float wipe_volume_nc,float purge_volume,
                                bool force_emit, float ram_length_override_mm)
{
	assert(m_plan.empty() || m_plan.back().z <= z_par + WT_EPSILON);	// refuses to add a layer below the last one

	if (m_plan.empty() || m_plan.back().z + WT_EPSILON < z_par) // if we moved to a new layer, we'll add it to m_plan first
		m_plan.push_back(WipeTowerInfo(z_par, layer_height_par));

    // OR in: a later call on the same layer must not clear a flag an earlier call set.
    m_plan.back().force_emit = m_plan.back().force_emit || force_emit;

    if (m_first_layer_idx == size_t(-1) && (! m_no_sparse_layers || old_tool != new_tool || force_emit))
        m_first_layer_idx = m_plan.size() - 1;

    if (old_tool == new_tool)	// new layer without toolchanges - we are done
        return;

	// this is an actual toolchange - let's calculate depth to reserve on the wipe tower
    float depth = 0.f;
    float width = m_wipe_tower_width - 2 * m_perimeter_width;

    // BBS: if the wipe tower width is too small, the depth will be infinity
    if (width <= EPSILON)
        return;
    int   layer_id    = static_cast<int>(m_plan.size()) - 1;
    float wipe_volume = is_same_extruder(old_tool, new_tool, layer_id) && !is_same_nozzle(old_tool, new_tool, layer_id) ? wipe_volume_nc : wipe_volume_ec;
    // BBS: remove old filament ramming and first line
#if 0
	float length_to_extrude = volume_to_length(0.25f * std::accumulate(m_filpar[old_tool].ramming_speed.begin(), m_filpar[old_tool].ramming_speed.end(), 0.f),
										m_perimeter_width * m_filpar[old_tool].ramming_line_width_multiplicator,
										layer_height_par);
	depth = (int(length_to_extrude / width) + 1) * (m_perimeter_width * m_filpar[old_tool].ramming_line_width_multiplicator * m_filpar[old_tool].ramming_step_multiplicator);
    float ramming_depth = depth;
    length_to_extrude = width*((length_to_extrude / width)-int(length_to_extrude / width)) - width;
    float first_wipe_line = -length_to_extrude;
    length_to_extrude += volume_to_length(wipe_volume, m_perimeter_width, layer_height_par);
    length_to_extrude = std::max(length_to_extrude,0.f);

    depth += (int(length_to_extrude / width) + 1) * m_perimeter_width;
    depth *= m_extra_spacing;

    m_plan.back().tool_changes.push_back(WipeTowerInfo::ToolChange(old_tool, new_tool, depth, ramming_depth, first_wipe_line, wipe_volume));
#else
    const float purge_row_width = purge_width(int(new_tool));
    float length_to_extrude = volume_to_length(wipe_volume, purge_row_width, layer_height_par);

    depth += std::ceil(length_to_extrude / width) * std::max(m_perimeter_width, purge_row_width);
    //depth *= m_extra_spacing;
    const bool is_extruder_change = !is_same_extruder(old_tool, new_tool, layer_id);
    // Same outgoing-tool override rule as set_toolchange().
    float filament_change_length = (ram_length_override_mm >= 0.f && is_extruder_change)
        ? ram_length_override_mm
        : filament_vector_value(is_extruder_change ?
                                                         m_filaments_change_length.first : m_filaments_change_length.second,
                                                     size_t(old_tool));
    float nozzle_change_depth = 0;
    float nozzle_change_length = 0;
    if (is_need_ramming(old_tool, new_tool, layer_id)) {
        double e_flow                   = nozzle_change_extrusion_flow(layer_height_par);
        double length                   = filament_change_length / e_flow;
        int    nozzle_change_line_count = std::ceil(length / (m_wipe_tower_width - 2*m_nozzle_change_perimeter_width));
        nozzle_change_depth = nozzle_change_line_count * m_nozzle_change_perimeter_width;
        depth += nozzle_change_depth;
        nozzle_change_length = length;
    }
    WipeTowerInfo::ToolChange tool_change = WipeTowerInfo::ToolChange(old_tool, new_tool, depth, 0.f, 0.f, wipe_volume, length_to_extrude, purge_volume);
    tool_change.nozzle_change_depth       = nozzle_change_depth;
    tool_change.nozzle_change_length      = nozzle_change_length;
    tool_change.nozzle_change_budget      = filament_change_length;
    tool_change.ram_length_override_mm    = ram_length_override_mm;
    m_plan.back().tool_changes.push_back(tool_change);
#endif
}
#if 0
void WipeTower::plan_tower()
{
    // BBS
    // calculate extra spacing
    float max_depth = 0.f;
    for (auto& info : m_plan)
        max_depth = std::max(max_depth, info.toolchanges_depth());

    float min_wipe_tower_depth = WipeTower::get_limit_depth_by_height(m_wipe_tower_height);

    {
        if (m_enable_wrapping_detection && max_depth < EPSILON)
            max_depth = wrapping_wipe_tower_depth;

        if (m_enable_timelapse_print && max_depth < EPSILON)
            max_depth = min_wipe_tower_depth;

        if (max_depth + EPSILON < min_wipe_tower_depth && !has_tpu_filament())
            m_extra_spacing = min_wipe_tower_depth / max_depth;
        else
            m_extra_spacing = 1.f;

        for (int idx = 0; idx < m_plan.size(); idx++) {
            auto& info = m_plan[idx];
            if (idx == 0 && m_extra_spacing > 1.f + EPSILON) {
                // apply solid fill for the first layer
                info.extra_spacing = 1.f;
                for (auto& toolchange : info.tool_changes) {
                    float x_to_wipe = volume_to_length(toolchange.wipe_volume, m_perimeter_width, info.height);
                    float line_len = m_wipe_tower_width - 2 * m_perimeter_width;
                    float x_to_wipe_new = x_to_wipe * m_extra_spacing;
                    x_to_wipe_new = std::floor(x_to_wipe_new / line_len) * line_len;
                    x_to_wipe_new = std::max(x_to_wipe_new, x_to_wipe);

                    int line_count = std::ceil((x_to_wipe_new - WT_EPSILON) / line_len);

                    {  // nozzle change length
                        int nozzle_change_line_count = (toolchange.nozzle_change_depth + WT_EPSILON) / m_perimeter_width;
                        line_count += nozzle_change_line_count;
                    }

                    toolchange.required_depth = line_count * m_perimeter_width;
                    toolchange.wipe_volume = x_to_wipe_new / x_to_wipe * toolchange.wipe_volume;
                    toolchange.wipe_length = x_to_wipe_new;
                }
            }
            else {
                info.extra_spacing = m_extra_spacing;
                for (auto& toolchange : info.tool_changes) {
                    toolchange.required_depth *= m_extra_spacing;
                    toolchange.wipe_length = volume_to_length(toolchange.wipe_volume, m_perimeter_width, info.height);
                }
            }
        }
    }

	// Calculate m_wipe_tower_depth (maximum depth for all the layers) and propagate depths downwards
	m_wipe_tower_depth = 0.f;
	for (auto& layer : m_plan)
		layer.depth = 0.f;

    float max_depth_for_all = 0;
    for (int layer_index = int(m_plan.size()) - 1; layer_index >= 0; --layer_index)
	{
        float this_layer_depth = std::max(m_plan[layer_index].depth, m_plan[layer_index].toolchanges_depth());
        if (m_enable_wrapping_detection && (layer_index < m_wrapping_detection_layers) && this_layer_depth < EPSILON)
            this_layer_depth = wrapping_wipe_tower_depth;

        if (m_enable_timelapse_print && this_layer_depth < EPSILON)
            this_layer_depth = min_wipe_tower_depth;

		m_plan[layer_index].depth = this_layer_depth;

		if (this_layer_depth > m_wipe_tower_depth - m_perimeter_width)
			m_wipe_tower_depth = this_layer_depth + m_perimeter_width;

		for (int i = layer_index - 1; i >= 0 ; i--)
		{
			if (m_plan[i].depth - this_layer_depth < 2*m_perimeter_width )
				m_plan[i].depth = this_layer_depth;
		}

        if (m_enable_timelapse_print && layer_index == 0)
            max_depth_for_all = m_plan[0].depth;
    }

    if (m_enable_wrapping_detection) {
        for (int i = m_wrapping_detection_layers - 1; i >= 0; i--) {
            if (m_plan.size() <= m_wrapping_detection_layers && (m_plan[i].depth < wrapping_wipe_tower_depth)) {
                m_plan[i].depth = wrapping_wipe_tower_depth;
            }
        }
    }

    if (m_enable_timelapse_print) {
        for (int i = int(m_plan.size()) - 1; i >= 0; i--) {
            m_plan[i].depth = max_depth_for_all;
        }
    }
}

void WipeTower::save_on_last_wipe()
{
    for (m_layer_info=m_plan.begin();m_layer_info<m_plan.end();++m_layer_info) {
        set_layer(m_layer_info->z, m_layer_info->height, 0, m_layer_info->z == m_plan.front().z, m_layer_info->z == m_plan.back().z);
        if (m_layer_info->tool_changes.size()==0)   // we have no way to save anything on an empty layer
            continue;

        // Which toolchange will finish_layer extrusions be subtracted from?
        // BBS: consider both soluable and support properties
        int idx = first_toolchange_to_nonsoluble_nonsupport(m_layer_info->tool_changes);

        for (int i=0; i<int(m_layer_info->tool_changes.size()); ++i) {
            auto& toolchange = m_layer_info->tool_changes[i];
            tool_change(toolchange.new_tool);

            if (i == idx) {
                float width = m_wipe_tower_width - 3*m_perimeter_width; // width we draw into
                float length_to_save = finish_layer().total_extrusion_length_in_plane();
                float length_to_wipe = volume_to_length(toolchange.wipe_volume,
                                      m_perimeter_width, m_layer_info->height)  - toolchange.first_wipe_line - length_to_save;

                length_to_wipe = std::max(length_to_wipe,0.f);
                float depth_to_wipe = m_perimeter_width * (std::floor(length_to_wipe/width) + ( length_to_wipe > 0.f ? 1.f : 0.f ) ) * m_extra_spacing;

                toolchange.required_depth = toolchange.ramming_depth + depth_to_wipe;
            }
        }
    }
}
#endif
bool WipeTower::is_tpu_filament(int filament_id) const
{
    return m_filpar[filament_id].material == "TPU";
}

bool WipeTower::is_petg_filament(int filament_id) const
{
    return m_filpar[filament_id].material == "PETG";
}

bool WipeTower::is_need_reverse_travel(int filament_id,bool extruder_change) const
{
    if (extruder_change)
        return m_filpar[filament_id].ramming_travel_time.first > EPSILON &&
               filament_vector_value(m_filaments_change_length.first, size_t(filament_id)) > EPSILON;
    return m_filpar[filament_id].ramming_travel_time.second > EPSILON &&
           filament_vector_value(m_filaments_change_length.second, size_t(filament_id)) > EPSILON;
}

// BBS: consider both soluable and support properties
// Return index of first toolchange that switches to non-soluble and non-support extruder
// ot -1 if there is no such toolchange.
int WipeTower::first_toolchange_to_nonsoluble_nonsupport(
        const std::vector<WipeTowerInfo::ToolChange>& tool_changes) const
{
    for (size_t idx=0; idx<tool_changes.size(); ++idx)
        if (! m_filpar[tool_changes[idx].new_tool].is_soluble && ! m_filpar[tool_changes[idx].new_tool].is_support)
            return idx;
    return -1;
}

WipeTower::ToolChangeResult WipeTower::merge_tcr(ToolChangeResult &first, ToolChangeResult &second)
{
    // An empty result has no real position, so merging it would add a bogus travel and time.
    if (!is_valid_gcode(first.gcode))
        return second;
    if (!is_valid_gcode(second.gcode))
        return first;
    assert(first.new_tool == second.initial_tool);
    WipeTower::ToolChangeResult out = first;
    out.departure_retract_extra = second.is_tool_change ? second.departure_retract_extra :
        std::max(first.departure_retract_extra, second.departure_retract_extra);
    if (second.is_tool_change || out.departure_wipe_path.empty())
        out.departure_wipe_path = second.departure_wipe_path;
    out.elapsed_time += second.elapsed_time;
    // The merged result retains at most one nozzle-change snippet, just as below.
    if (!first.nozzle_change_result.gcode.empty() && !second.nozzle_change_result.gcode.empty())
        out.elapsed_time -= second.nozzle_change_result.elapsed_time;
    if ((first.end_pos - second.start_pos).norm() > (float)EPSILON) {
        std::string travel_gcode = "G1 X" + Slic3r::float_to_string_decimal_point(second.start_pos.x(), 3) + " Y" +
                                   Slic3r::float_to_string_decimal_point(second.start_pos.y(), 3) + " F" + std::to_string(m_max_speed) + "\n";
        bool need_insert_travel = true;
        if (second.is_tool_change
            && is_approx(second.start_pos.x(), second.tool_change_start_pos.x())
            && is_approx(second.start_pos.y(), second.tool_change_start_pos.y())) {
            // will insert travel in gcode.cpp
            need_insert_travel = false;
        }

        if (need_insert_travel) {
            out.gcode += travel_gcode;
            out.elapsed_time += m_max_speed > 0.f ?
                (first.end_pos - second.start_pos).norm() / m_max_speed * 60.f :
                std::numeric_limits<float>::quiet_NaN();
        }
    }
    out.gcode += second.gcode;
    out.extrusions.insert(out.extrusions.end(), second.extrusions.begin(), second.extrusions.end());
    out.structural_emissions.insert(out.structural_emissions.end(),
                                    second.structural_emissions.begin(),
                                    second.structural_emissions.end());
    out.end_pos = second.end_pos;
    out.wipe_path = second.wipe_path;
    out.initial_tool = first.initial_tool;
    out.new_tool = second.new_tool;
    out.is_contact   = first.is_contact || second.is_contact;
    if (!first.nozzle_change_result.gcode.empty())
        out.nozzle_change_result = first.nozzle_change_result;
    else if (!second.nozzle_change_result.gcode.empty())
        out.nozzle_change_result = second.nozzle_change_result;

    if (first.is_tool_change) {
        out.is_tool_change = true;
        out.tool_change_start_pos = first.tool_change_start_pos;
    }
    else if (second.is_tool_change) {
        out.is_tool_change = true;
        out.tool_change_start_pos = second.tool_change_start_pos;
    }
    else {
        out.is_tool_change = false;
    }

    // BBS
    out.purge_volume += second.purge_volume;
    return out;
}

void WipeTower::get_all_wall_skip_points() {
    m_wall_skip_points.clear();
    m_wall_skip_points.resize(m_plan.size());
    for (int i = 0; i < m_plan.size(); i++) {
        const WipeTowerInfo &layer = m_plan[i];
        get_wall_skip_points(m_plan[i],i);
    }
}


void WipeTower::get_wall_skip_points(const WipeTowerInfo &layer, int layer_id)
{
    const int                      pre_access_layer = 4;
    std::unordered_map<int, float> cur_block_depth;
    for (int i = 0; i < int(layer.tool_changes.size()); ++i) {
        const WipeTowerInfo::ToolChange &tool_change         = layer.tool_changes[i];
        size_t                           old_filament        = tool_change.old_tool;
        size_t                           new_filament        = tool_change.new_tool;
        float                            nozzle_change_depth = tool_change.nozzle_change_depth;
        float                            wipe_depth          = tool_change.required_depth - nozzle_change_depth;
        if (!is_valid_last_layer(old_filament, layer_id, m_plan[layer_id].z)) nozzle_change_depth = 0.f;
        auto *block = get_block_by_category(m_filpar[new_filament].category, false);
        if (!block) continue;
        float process_depth = 0.f;
        if (!cur_block_depth.count(m_filpar[new_filament].category)) cur_block_depth[m_filpar[new_filament].category] = block->start_depth;
        process_depth = cur_block_depth[m_filpar[new_filament].category];
        if (is_need_ramming(new_filament, old_filament, layer_id)) {
            // Bounds-checked: the category table can be shorter than the filament count.
            if (get_filament_category(int(new_filament)) == get_filament_category(int(old_filament)))
                process_depth += nozzle_change_depth;
            else {
                if (!cur_block_depth.count(m_filpar[old_filament].category)) {
                    auto *old_block = get_block_by_category(m_filpar[old_filament].category, false);
                    if (!old_block) continue;
                    cur_block_depth[m_filpar[old_filament].category] = old_block->start_depth;
                }
                cur_block_depth[m_filpar[old_filament].category] += nozzle_change_depth;
            }
        }

        float infill_gap_width = get_block_gap_width(new_filament, false);
        Vec2f res;
        int   index = layer_id % 4;
        switch (index % 4) {
        case 0: res = Vec2f(0, process_depth); break;
        case 1: res = Vec2f(m_wipe_tower_width, process_depth + wipe_depth - m_plan[layer_id].extra_spacing * infill_gap_width); break;
        case 2: res = Vec2f(m_wipe_tower_width, process_depth); break;
        case 3: res = Vec2f(0, process_depth + wipe_depth - m_plan[layer_id].extra_spacing * infill_gap_width); break;
        default: break;
        }

        m_wall_skip_points[layer_id].emplace_back(res);

        cur_block_depth[m_filpar[new_filament].category] = process_depth + wipe_depth;

        bool solid_toolchange = block->layers_type[layer_id] == WipeTowerLayerType::Contact;
        if (solid_toolchange && m_enable_tower_interface_features) {
            for (int j = 0; j < pre_access_layer; j++) {
                int pre_layer_id = layer_id - j;
                if (pre_layer_id < 0) break;
                m_wall_skip_points[pre_layer_id].push_back(res);
            }
        }
    }
    if (m_enable_tower_interface_features) {
        for (auto &block : m_wipe_tower_blocks) {
            float block_depth = cur_block_depth.count(block.filament_adhesiveness_category) ? cur_block_depth[block.filament_adhesiveness_category] : block.start_depth;
            if (block_depth + EPSILON >= block.start_depth + block.layer_depths[layer_id] - m_perimeter_width) { continue; }
            bool block_solid = block.layers_type[layer_id] == WipeTowerLayerType::Contact;
            bool add_skip_point = block_solid && std::abs(block_depth - block.start_depth) < EPSILON;
            if (add_skip_point) {
                Vec2f res;
                int   index = layer_id % 4;

                float dy_skip = block.layer_depths[layer_id] - m_perimeter_width;
                int   n_skip  = (int) ((dy_skip + 0.25f * m_perimeter_width) / m_perimeter_width + 1);
                float gird_depth = m_perimeter_width * (n_skip - 1); // in sync with finish_block_solid
                switch (index % 4) {
                case 0: res = Vec2f(0, block_depth); break;
                case 1: res = Vec2f(m_wipe_tower_width, block_depth + gird_depth); break;
                case 2: res = Vec2f(m_wipe_tower_width, block_depth); break;
                case 3: res = Vec2f(0, block_depth + gird_depth); break;
                default: break;
                }
                m_wall_skip_points[layer_id].emplace_back(res);
                for (int j = 0; j < pre_access_layer; j++) {
                    int pre_layer_id = layer_id - j;
                    if (pre_layer_id < 0) break;
                    m_wall_skip_points[pre_layer_id].push_back(res);
                }
            }
        }
    }
}

float WipeTower::purge_rows_depth(int tool, float wipe_length, float wipe_volume_budget, bool solid) const
{
    // As toolchange_wipe_new() lays them: rows across the box at the tool's purge width and pitch, until the prime
    // volume or the wipe length is used up, whichever comes first; one row more for good measure.
    if (solid || tool < 0 || size_t(tool) >= m_filpar.size())
        return std::numeric_limits<float>::max();
    const float width = purge_width(tool);
    const float row   = m_wipe_tower_width - 2 * m_perimeter_width;
    const float flow  = purge_extrusion_flow(tool, m_layer_height);
    if (row <= WT_EPSILON || flow <= 0.f)
        return std::numeric_limits<float>::max();
    float rows = std::ceil(std::max(wipe_length, 0.f) / row);
    if (wipe_volume_budget > 0.f)
        rows = std::min(rows, std::ceil(wipe_volume_budget / filament_area() / (row * flow)));
    const float pitch = is_base_layer() ? width : m_layer_info->extra_spacing * width;
    return (std::max(rows, 1.f) + 1.f) * pitch;
}

float WipeTower::beside_held_rows(const WipeTowerBlock *block, int tool, float z, float height, float box_depth,
                                  float needed, float moved_depth) const
{
    const float held = held_road_z(block);
    if (held <= z + WT_EPSILON || tool < 0 || size_t(tool) >= m_filpar.size())
        return -1.f;
    const auto rows = m_held_rows_end.find(block->block_id);
    if (rows == m_held_rows_end.end() || m_cur_layer_id < 0 || size_t(m_cur_layer_id) >= block->layer_depths.size())
        return -1.f;
    // Rows that start after the held ones (as after this level's own ramming) are beside them already.
    if (block->cur_depth + WT_EPSILON >= rows->second)
        return block->cur_depth;
    const float highest = m_filpar[size_t(tool)].max_layer_height;
    if (!std::isfinite(highest) || highest <= 0.f || height + held - z <= highest + WT_EPSILON)
        return -1.f;
    // The part of the box after the held rows holds them, or the block has room for them moved there.
    if (needed <= block->cur_depth + box_depth - rows->second + WT_EPSILON ||
        rows->second + moved_depth <= block->start_depth + block->layer_depths[m_cur_layer_id] - m_perimeter_width + WT_EPSILON)
        return rows->second;
    return -1.f;
}

WipeTower::ToolChangeResult WipeTower::tool_change_new(size_t new_tool, bool solid_toolchange,bool solid_nozzlechange)
{
    m_nozzle_change_result.gcode.clear();
    m_nozzle_change_result.structural_emissions.clear();
    m_purge_rows_top_y = -1.f;
    bool hotend_change = false;
    const bool is_nozzle_change = is_need_ramming(m_current_tool, new_tool, m_cur_layer_id);
    if (is_nozzle_change) {
        hotend_change = is_same_extruder(m_current_tool, new_tool, m_cur_layer_id);
        //If it is the last layer and exceeds the printable height, cancel ramming
        if (is_valid_last_layer(m_current_tool, m_cur_layer_id, m_z_pos)) m_nozzle_change_result = ramming(m_current_tool, new_tool, solid_nozzlechange, !hotend_change);
    }

    size_t old_tool = m_current_tool;
    float wipe_depth          = 0.f;
    float wipe_length         = 0.f;
    // Requested prime volume (mm^3), kept apart from the quantised wipe length; purge_volume is
    // only reported bookkeeping.
    float wipe_volume_budget  = 0.f;
    float purge_volume        = 0.f;
    float nozzle_change_depth = 0.f;
    int   nozzle_change_line_count = 0;
    // -1 when the hand-off kept a vendor volume; departure relief applies only to resolver-sized ones.
    float ram_length_override = -1.f;
    std::vector<Vec2f> departure_row;

    if (new_tool != (unsigned int) (-1)) {
        for (const auto &b : m_layer_info->tool_changes)
            if (b.new_tool == new_tool) {
                wipe_length         = b.wipe_length;
                wipe_volume_budget  = b.wipe_volume_budget;
                wipe_depth          = b.required_depth;
                purge_volume        = b.purge_volume;
                nozzle_change_depth = b.nozzle_change_depth;
                ram_length_override = b.ram_length_override_mm;
                break;
            }
    }
    m_current_tool        = new_tool;
    WipeTowerBlock* block = get_block_by_category(m_filpar[new_tool].category, false);
    if (!block) {
        assert(block != nullptr);
        return WipeTower::ToolChangeResult();
    }
    m_cur_block = block;
    // Under a held road in this block the purge rows go beside the held rows where they can (beside_held_rows());
    // otherwise they are lifted (see below).
    float box_bottom = block->cur_depth;
    float box_top    = block->cur_depth + wipe_depth - nozzle_change_depth;
    bool  purge_lifted = false;
    if (m_mixed_nozzle_slicing && held_road_z(block) > m_z_pos + WT_EPSILON) {
        // Only the part of the box after the held rows is used: the box is not moved, as the rest of the level's
        // rows follow it.
        const float beside = beside_held_rows(block, int(new_tool), m_z_pos, m_layer_height, box_top - box_bottom,
            purge_rows_depth(int(new_tool), wipe_length, wipe_volume_budget, solid_toolchange),
            std::numeric_limits<float>::max());
        if (beside >= 0.f)
            box_bottom = beside;
        purge_lifted = beside < 0.f;
    }
    // Purge rows lifted over a held road stand higher than the next level's wall, so they keep half a row clear of
    // the wall line instead of running up to it.
    const float lifted_inset = purge_lifted ? 0.5f * purge_width(int(new_tool)) : 0.f;
    box_coordinates cleaning_box(Vec2f(m_perimeter_width + lifted_inset, box_bottom),
                                 m_wipe_tower_width - 2 * (m_perimeter_width + lifted_inset), box_top - box_bottom);
    // The same holds across the depth. Where the box starts the block, the wall's near side (the first block) or
    // the block before, laid later at the level's height, is beside the first row; where it ends, the block fill
    // laid after it is. There the rows keep half a row clear too.
    box_coordinates rows_box = cleaning_box;
    if (purge_lifted) {
        const float rows_bottom = std::max(box_bottom, block->start_depth + lifted_inset);
        const float rows_top    = box_top - lifted_inset;
        if (rows_top > rows_bottom + WT_EPSILON)
            rows_box = box_coordinates(Vec2f(cleaning_box.ld.x(), rows_bottom), cleaning_box.rd.x() - cleaning_box.ld.x(),
                                       rows_top - rows_bottom);
    }
    // Rows after a ramming held above this level in the block (the arriving tool's purge) start clear of it, in the
    // box's spare depth.
    else if (const auto clear = m_level_clear_from.find(block->block_id); m_mixed_nozzle_slicing && clear != m_level_clear_from.end()) {
        const float rows_bottom = clear->second + 0.5f * purge_width(int(new_tool));
        if (rows_bottom > box_bottom + WT_EPSILON && rows_bottom < box_top - m_perimeter_width - WT_EPSILON)
            rows_box = box_coordinates(Vec2f(cleaning_box.ld.x(), rows_bottom), cleaning_box.rd.x() - cleaning_box.ld.x(),
                                       box_top - rows_bottom);
    }

    WipeTowerWriter writer(m_layer_height, m_perimeter_width, m_gcode_flavor, m_filpar, m_enable_arc_fitting, m_travel_speed, m_lag_emit_block_z);
    writer.set_extrusion_flow(m_extrusion_flow)
        .set_z(m_z_pos)
        .set_initial_tool(m_current_tool)
        .set_y_shift(m_y_shift + (new_tool != (unsigned int) (-1) && (m_current_shape == SHAPE_REVERSED) ? m_layer_info->depth - m_layer_info->toolchanges_depth() : 0.f))
        .append(";--------------------\n"
                "; CP TOOLCHANGE START\n")
        .comment_with_value(" toolchange #", m_num_tool_changes + 1); // the number is zero-based

    set_for_wipe_tower_writer(writer);

    std::vector<StructuralEmission> purge_records;
    bool purge_held = false;
    if (new_tool != (unsigned) (-1))
        writer.append( std::string("; material : " + (m_current_tool < m_filpar.size() ? m_filpar[m_current_tool].material : "(NONE)") + " -> " + m_filpar[new_tool].material + "\n").c_str())
            .append(";--------------------\n");

    writer.speed_override_backup();
    writer.speed_override(100);

    // Ram the hot material out of the melt zone, retract the filament into the cooling tubes and let it cool.
    if (new_tool != (unsigned int) -1) { // This is not the last change.
        Vec2f initial_position = get_next_pos(rows_box, wipe_length, solid_toolchange);
        // Rows laid from the top down (odd layers) start where they would in the whole box, within the rows' box:
        // only the rows next to the wall move.
        if (m_cur_layer_id % 2 == 1 && (rows_box.ld.y() != cleaning_box.ld.y() || rows_box.lu.y() != cleaning_box.lu.y()))
            initial_position.y() = std::max(rows_box.ld.y() + m_depth_traversed,
                                            std::min(get_next_pos(cleaning_box, wipe_length, solid_toolchange).y(),
                                                     rows_box.lu.y() + m_depth_traversed - m_perimeter_width));
        writer.set_initial_position(initial_position, m_wipe_tower_width, m_wipe_tower_depth, m_internal_rotation);

        writer.append_wipe_tower_start();
        // Hold the initial-layer temperature for the whole tower base, emitted only when the target
        // changes. M104 does not wait: the purge comes before the joint that needs the heat.
        if (m_mixed_nozzle_slicing && m_tower_base_top_z > 0.f &&
            size_t(new_tool) < m_tower_temperature_held.size()) {
            const int target = is_tower_base_layer() ? m_filpar[new_tool].nozzle_temperature_initial_layer
                                                     : m_filpar[new_tool].nozzle_temperature;
            if (target != 0 && target != m_tower_temperature_held[new_tool]) {
                writer.format_line_M104(target, get_extruder_id(int(new_tool), m_cur_layer_id), false);
                m_tower_temperature_held[new_tool] = target;
            }
        }
        toolchange_Unload(writer, cleaning_box, m_filpar[m_current_tool].material,
                          is_base_layer() ? m_filpar[new_tool].nozzle_temperature_initial_layer : m_filpar[new_tool].nozzle_temperature);
        toolchange_Change(writer, new_tool, m_filpar[new_tool].material); // Change the tool, set a speed override for soluble and flex materials.
        toolchange_Load(writer, cleaning_box);
# if 0
        if (m_is_multi_extruder && is_need_reverse_travel(new_tool)) {
            float dy = m_layer_info->extra_spacing * m_nozzle_change_perimeter_width;
            if (m_layer_info->extra_spacing < m_tpu_fixed_spacing) {
                dy = m_tpu_fixed_spacing * m_nozzle_change_perimeter_width;
            }

            float nozzle_change_speed = 60.0f * m_filpar[new_tool].max_e_speed / m_extrusion_flow;
            nozzle_change_speed *= 0.25;

            const float &xl = cleaning_box.ld.x();
            const float &xr = cleaning_box.rd.x();

            Vec2f  start_pos         = m_nozzle_change_result.origin_start_pos + Vec2f(0, m_nozzle_change_perimeter_width);
            bool   left_to_right     = true;
            int tpu_line_count = (nozzle_change_line_count + 2 - 1) / 2; // nozzle_change_line_count / 2 round up

            writer.travel(start_pos);

            for (int i = 0; true; ++i) {
                if (left_to_right)
                    writer.travel(xr - m_perimeter_width, writer.y(), nozzle_change_speed);
                else
                    writer.travel(xl + m_perimeter_width, writer.y(), nozzle_change_speed);

                if (i == tpu_line_count - 1) break;

                writer.travel(writer.x(), writer.y() + dy);
                left_to_right = !left_to_right;
            }
            writer.travel(initial_position);
        }
#endif
        const size_t purge_begin = writer.extrusions().size();
        // Over a held road in this block (a held coarse prime road or ramming) the purge rows are lifted to its top
        // and laid that much taller, so they never run into it; the head comes back down over the wall line, clear
        // of the rows.
        const float level_z      = m_z_pos;
        const float level_height = m_layer_height;
        const float held         = held_road_z(block);
        if (purge_lifted) {
            m_layer_height += held - m_z_pos;
            m_z_pos         = held;
            writer.move_z(held);
        }
        toolchange_wipe_new(writer, rows_box, wipe_length, solid_toolchange, wipe_volume_budget, is_nozzle_change);
        departure_row = middle_purge_row(writer.extrusions(), purge_begin);
        // Wipe rows belong to the block's internal support domain, separate from the outer wall.
        purge_records = make_structural_emissions(writer.extrusions(), m_z_pos, m_layer_height,
            StructuralRole::InteriorDeposit, static_cast<unsigned int>(2 + block->block_id), purge_begin);
        if (purge_lifted) {
            purge_held = true;
            for (StructuralEmission &record : purge_records) {
                record.held_above_level = true;
                record.held_level       = true;
            }
            m_z_pos         = level_z;
            m_layer_height  = level_height;
            writer.travel(0.f, writer.y());
            writer.move_z(level_z);
        }

        writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_End) + "\n");
        ++m_num_tool_changes;
    } else
        toolchange_Unload(writer, cleaning_box, m_filpar[m_current_tool].material, m_filpar[m_current_tool].nozzle_temperature);

    float box_depth = wipe_depth - nozzle_change_depth;
    // On the tower's bed level the purge rows are dense and stop at the prime volume, well short of
    // the box planned for them. The block fill that follows starts where the rows ended, so the first
    // layer has no bare strip for the next level's rows and ramming to land on.
    if (m_mixed_nozzle_slicing && is_first_layer() && !solid_toolchange && new_tool != (unsigned int) -1 &&
        m_purge_rows_top_y >= block->cur_depth) {
        const float used = m_purge_rows_top_y - block->cur_depth + 0.5f * (purge_width(int(new_tool)) + m_perimeter_width);
        box_depth = std::min(box_depth, used);
    }
    block->cur_depth += box_depth;
    block->last_filament_change_id = new_tool;
    // Lifted rows are held rows too: a level below the held top starts its block fill after them.
    if (purge_held) {
        float &end = m_held_rows_end[block->block_id];
        end = std::max(end, block->cur_depth);
        m_held_spans[block->block_id].push_back({box_bottom, block->cur_depth, held_road_z(block)});
    }

    // BBS
    writer.speed_override_restore();
    writer.feedrate(m_travel_speed * 60.f)
        .flush_planner_queue()
        .reset_extruder()
        .append("; CP TOOLCHANGE END\n"
                ";------------------\n"
                "\n\n");

    // Ask our writer about how much material was consumed:
    if (m_current_tool < m_used_filament_length.size())
        m_used_filament_length[m_current_tool] += writer.get_and_reset_used_filament_length();

    auto result = construct_tcr(writer, false, old_tool, false, true, purge_volume, solid_toolchange);
    if (m_mixed_nozzle_slicing && ram_length_override >= 0.f && is_fine_return(old_tool, new_tool))
    {
        result.departure_retract_extra = mixed_nozzle_fine_return_extra_retract;
        result.departure_wipe_path     = departure_row;
    }
    result.structural_emissions.insert(result.structural_emissions.end(), purge_records.begin(), purge_records.end());
    return result;
}

//for extruder change and nozzle change
WipeTower::NozzleChangeResult WipeTower::ramming(int old_filament_id, int new_filament_id, bool solid_infill, bool extruder_change)
{
    auto format_line_M106 = []() { return std::string{"M106 S255\n"};};
    auto format_line_M633 = []() { return std::string{"M633\n"};};
    auto format_line_M632 = [](int filament_id, int nozzle_id) {
        std::string buffer = "M632 S" + std::to_string(filament_id);
        if (nozzle_id >= 0)
            buffer += " H" + std::to_string(nozzle_id);
        buffer += " M N\n";
        return buffer;
    };

    int   nozzle_change_line_count = 0;
    float x_offset                 = m_perimeter_width + (m_nozzle_change_perimeter_width - m_perimeter_width) / 2;
    float nozzle_change_box_width  = m_wipe_tower_width - 2 * x_offset;
    float nozzle_change_depth      = 0.f;
    // Filament length (mm) this ramming block may deposit. The planned line count stays an upper
    // bound on depth; the budget keeps the volume right when the bridge flow is used.
    float nozzle_change_budget     = 0.f;
    if (new_filament_id != (unsigned int) (-1)) {
        for (const auto &b : m_layer_info->tool_changes)
            if (b.new_tool == new_filament_id) {
                nozzle_change_line_count = std::ceil(b.nozzle_change_length  / nozzle_change_box_width);
                nozzle_change_depth      = b.nozzle_change_depth;
                // Zero means no budget: Off mode lays the planned line count as upstream does.
                nozzle_change_budget     = m_mixed_nozzle_slicing ? b.nozzle_change_budget : 0.f;
                break;
            }
    }
    auto format_nozzle_change_line = [this](bool start, int old_filament_id, int new_filament_id) -> std::string {
        char        buff[64];
        // Orca: the nozzle-change markers are standalone tag constants, not ETags entries
        // (the Reserved_Tags parallel arrays have a non-BBL variant that must stay aligned).
        std::string tag = start ? GCodeProcessor::Nozzle_Change_Start_Tag : GCodeProcessor::Nozzle_Change_End_Tag;
        int         old_nozzle_id = get_nozzle_id(old_filament_id, m_cur_layer_id);
        int         new_nozzle_id = get_nozzle_id(new_filament_id, m_cur_layer_id);
        snprintf(buff, sizeof(buff), ";%s OF%d NF%d ON%d NN%d\n", tag.c_str(), old_filament_id, new_filament_id,old_nozzle_id,new_nozzle_id);
        return std::string(buff);
    };

    // The outgoing tool rams at the nearest height within its own layer-height limits. On a
    // lagging tower the level height belongs to the arriving tool and may be outside them.
    float ram_height = ramming_height(old_filament_id, m_layer_height);
    float ram_z      = m_z_pos - m_layer_height + ram_height;
    // Over a held road (a held coarse prime road or an earlier held ramming) the ramming rows are lifted to its top,
    // so they never run into it, or laid beside the held rows when the outgoing tool cannot lay them that tall.
    bool ram_lifted = false;
    WipeTowerBlock *ram_block = get_block_by_category(m_filpar[old_filament_id].category, false);
    // Moved after the held rows, the ramming takes the rest of this level's rows in the block along with it.
    float rows_after = 0.f;
    if (ram_block != nullptr) {
        bool from_here = false;
        for (const auto &change : m_layer_info->tool_changes) {
            from_here = from_here || (new_filament_id >= 0 && change.new_tool == size_t(new_filament_id));
            if (!from_here)
                continue;
            if (m_filpar[change.old_tool].category == ram_block->filament_adhesiveness_category)
                rows_after += change.nozzle_change_depth;
            if (m_filpar[change.new_tool].category == ram_block->filament_adhesiveness_category)
                rows_after += change.required_depth - change.nozzle_change_depth;
        }
    }
    const float ram_beside = beside_held_rows(ram_block, old_filament_id, ram_z, ram_height, nozzle_change_depth,
                                              std::numeric_limits<float>::max(), std::max(rows_after, nozzle_change_depth));
    // A ramming below its level that cannot go beside the held rows nor be lifted to them (too tall for the tool)
    // stands on them, when its box lies on held rows: at the outgoing tool's own height on their top, within the level.
    const auto stand_on_held = [&]() {
        const float held = held_road_z(ram_block);
        const auto  spans = ram_block != nullptr ? m_held_spans.find(ram_block->block_id) : m_held_spans.end();
        if (spans == m_held_spans.end() || ram_z >= m_z_pos - WT_EPSILON || !m_mixed_nozzle_slicing)
            return false;
        const float highest = m_filpar[old_filament_id].max_layer_height;
        if (!std::isfinite(highest) || highest <= 0.f || ram_height + held - ram_z <= highest + WT_EPSILON)
            return false;
        const float from = ram_block->cur_depth, to = ram_block->cur_depth + nozzle_change_depth;
        for (const HeldSpan &span : spans->second) {
            if (span.from > from + WT_EPSILON || span.to < to - WT_EPSILON)
                continue;
            const float height = ramming_height(old_filament_id, m_z_pos - span.z);
            if (span.z + height > m_z_pos + WT_EPSILON || span.z + height < held - WT_EPSILON)
                continue;
            ram_height = height;
            ram_z      = span.z + height;
            return true;
        }
        return false;
    };
    if (ram_beside >= 0.f)
        ram_block->cur_depth = ram_beside;
    else if (stand_on_held())
        ram_lifted = true;
    else if (const float held = held_road_z(ram_block); held > ram_z + WT_EPSILON) {
        ram_height += held - ram_z;
        ram_z       = held;
        ram_lifted  = true;
    }
    const bool  ram_moves  = std::abs(ram_z - m_z_pos) > WT_EPSILON;
    float nz_extrusion_flow = nozzle_change_extrusion_flow(ram_height);
    WipeTowerWriter writer(ram_height, m_nozzle_change_perimeter_width, m_gcode_flavor, m_filpar, m_enable_arc_fitting, m_travel_speed, m_lag_emit_block_z);
    writer.set_extrusion_flow(nz_extrusion_flow)
        .set_z(ram_z)
        .set_initial_tool(m_current_tool)
        .set_y_shift(m_y_shift + (new_filament_id != (unsigned int) (-1) && (m_current_shape == SHAPE_REVERSED) ? m_layer_info->depth - m_layer_info->toolchanges_depth() : 0.f));
    set_for_wipe_tower_writer(writer);

    WipeTowerBlock* block = get_block_by_category(m_filpar[old_filament_id].category, false);
    if (!block) {
        assert(false);
        return WipeTower::NozzleChangeResult();
    }
    m_cur_block = block;

    float dy = is_base_layer() ? m_nozzle_change_perimeter_width : m_layer_info->extra_spacing * get_block_gap_width(m_current_tool, true);
    // A ramming that stands above its level's top (the outgoing tool at its own minimum, or lifted to held rows)
    // has roads of the level laid after it and lower: the wall, the block before it, the arriving tool's purge
    // rows and the block fill. Like lifted purge rows, it keeps half a wall road clear of them: in from the wall
    // at its ends, and off the block's start. The roads after it in the block keep clear of it
    // (m_level_clear_from).
    const bool  ram_held  = m_mixed_nozzle_slicing && ram_z > m_z_pos + WT_EPSILON;
    const float ram_clear = ram_held ? 0.5f * m_perimeter_width : 0.f;
    box_coordinates cleaning_box(Vec2f(x_offset + ram_clear, block->cur_depth + (m_nozzle_change_perimeter_width - m_perimeter_width) / 2),
                                 nozzle_change_box_width - 2 * ram_clear,
                                 nozzle_change_depth);
    if (ram_held && nozzle_change_line_count > 0) {
        // Off the block's start the rows close up where they have room, so the last one stays where it was.
        const float shift = block->start_depth + ram_clear - block->cur_depth;
        if (shift > WT_EPSILON) {
            const float closed_up = nozzle_change_line_count > 1 ? dy - shift / float(nozzle_change_line_count - 1) : 0.f;
            if (!solid_infill && closed_up >= m_nozzle_change_perimeter_width - WT_EPSILON)
                dy = closed_up;
            cleaning_box = box_coordinates(cleaning_box.ld + Vec2f(0.f, shift), cleaning_box.rd.x() - cleaning_box.ld.x(),
                                           cleaning_box.lu.y() - cleaning_box.ld.y() - shift);
        }
    }
    Vec2f initial_position = cleaning_box.ld;
    writer.set_initial_position(initial_position, m_wipe_tower_width, m_wipe_tower_depth, m_internal_rotation);

    // --- Nozzle change preamble: notify firmware + precool ---
    writer.append(format_nozzle_change_line(true, old_filament_id, new_filament_id));
    if (!extruder_change) {
        int new_nozzle_id = m_multi_nozzle_group_result->is_support_dynamic_nozzle_map()
                                ? get_nozzle_id(new_filament_id, m_cur_layer_id) : -1;
        writer.append(format_line_M632(new_filament_id, new_nozzle_id));
        if (m_filpar[m_current_tool].precool_target_temp.second != 0) {
            writer.format_line_M104(m_filpar[m_current_tool].precool_target_temp.second, get_extruder_id(m_current_tool, m_cur_layer_id))
                .append(format_line_M106());
        }
        writer.append(format_line_M633());
    }

    NozzleChangeResult result;

    if (nozzle_change_line_count > 0) {
        // The exporter brought the head to the level's own Z; a ramming held to other limits
        // says its own height. set_z() already wrote it for a catch-up writer.
        if (ram_moves && !m_lag_emit_block_z)
            writer.move_z(ram_z);
        if (ram_moves)
            writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + std::to_string(ram_height) + "\n");
        float max_e_ramming_speed = extruder_change ? m_filpar[m_current_tool].max_e_ramming_speed.first : m_filpar[m_current_tool].max_e_ramming_speed.second;
        float nozzle_change_speed = 60.0f * max_e_ramming_speed / nz_extrusion_flow;
        if (solid_infill)
            nozzle_change_speed = std::min(40.f * 60.f, nozzle_change_speed);
        float bridge_speed = std::min(60.0f * max_e_ramming_speed / nozzle_change_extrusion_flow(0.2), nozzle_change_speed);

        const float &xl = cleaning_box.ld.x();
        const float &xr = cleaning_box.rd.x();
        dy = solid_infill ? m_nozzle_change_perimeter_width : dy;
        if (solid_infill)
            nozzle_change_line_count = std::floor(EPSILON + (cleaning_box.ru[1] - cleaning_box.rd[1] + (m_nozzle_change_perimeter_width - m_perimeter_width) / 2.f) /
                                                                m_nozzle_change_perimeter_width);
        m_left_to_right = true;
        bool need_change_flow   = false;
        float ramming_length    = nozzle_change_line_count * (xr - xl);
        int   extruder_id      = get_extruder_id(m_current_tool, m_cur_layer_id);
        float precool_t         = extruder_change ? m_filpar[m_current_tool].precool_t.first[extruder_id] : m_filpar[m_current_tool].precool_t.second[extruder_id];
        float precool_t_first_layer = extruder_change ? m_filpar[m_current_tool].precool_t_first_layer.first[extruder_id] :
                                                        m_filpar[m_current_tool].precool_t_first_layer.second[extruder_id];
        float per_cooling_max_speed = nozzle_change_speed;
        if (extruder_change) {
            if (is_first_layer() && precool_t_first_layer > EPSILON)
                per_cooling_max_speed = ramming_length / precool_t_first_layer * 60.f;
            else if (precool_t > EPSILON)
                per_cooling_max_speed = ramming_length / precool_t * 60.f;
        }//BBS:nozzle change does not require forcing a cooldown to a specific temperature.
        if (nozzle_change_speed > per_cooling_max_speed) nozzle_change_speed = per_cooling_max_speed;
        if (bridge_speed > per_cooling_max_speed) bridge_speed = per_cooling_max_speed;
        // Mixed-nozzle: a ramming laid on the bed is bed contact like the rest of the tower's first
        // layer, so it goes no faster than the tower's first-layer speed.
        if (m_mixed_nozzle_slicing && ram_z - ram_height <= WT_EPSILON) {
            nozzle_change_speed = std::min(nozzle_change_speed, first_layer_speed() * 60.f);
            bridge_speed        = std::min(bridge_speed, first_layer_speed() * 60.f);
        }
        LimitFlow LimitRamming = extruder_change ? LimitFlow::LimitRammingFlow : LimitFlow::LimitRammingFlowNC;
        // Deposited filament length, charged at each segment's actual flow, checked against the budget.
        float nozzle_change_emitted = 0.f;
        for (int i = 0; true; ++i) {
            if (need_thick_bridge_flow(writer.pos().y())) {
                writer.set_extrusion_flow(nozzle_change_extrusion_flow(0.2));
                writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + std::to_string(0.2) + "\n");
                need_change_flow = true;
            }
            {
                const size_t road_begin = writer.extrusions().size();
                const Vec2f before = writer.pos();
                if (m_left_to_right)
                    writer.extrude(xr + wipe_tower_wall_infill_overlap * m_perimeter_width, writer.y(), need_change_flow ? bridge_speed : nozzle_change_speed, LimitRamming);
                else
                    writer.extrude(xl - wipe_tower_wall_infill_overlap * m_perimeter_width, writer.y(), need_change_flow ? bridge_speed : nozzle_change_speed, LimitRamming);
                const float line_flow = need_change_flow ? nozzle_change_extrusion_flow(0.2) : nz_extrusion_flow;
                nozzle_change_emitted += (writer.pos() - before).norm() * line_flow;
                auto records = make_structural_emissions(writer.extrusions(), ram_z,
                    need_change_flow ? 0.2f : ram_height, StructuralRole::InteriorDeposit,
                    static_cast<unsigned int>(2 + block->block_id), road_begin);
                for (StructuralEmission &record : records) {
                    record.held_above_level = ram_z > m_z_pos + WT_EPSILON;
                    record.held_level       = ram_lifted || record.held_above_level;
                }
                result.structural_emissions.insert(result.structural_emissions.end(), records.begin(), records.end());
            }
            if (i == nozzle_change_line_count - 1)
                break;
            if (nozzle_change_budget > 0.f && nozzle_change_emitted + WT_EPSILON >= nozzle_change_budget)
                break;
            if ((writer.y() + dy - cleaning_box.ru.y()+(m_nozzle_change_perimeter_width+m_perimeter_width)/2) > (float)EPSILON) break;
            if (need_change_flow) {
                writer.set_extrusion_flow(nozzle_change_extrusion_flow(ram_height));
                writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + std::to_string(ram_height) + "\n");
                need_change_flow = false;
            }
            {
                const size_t road_begin = writer.extrusions().size();
                const Vec2f before = writer.pos();
                writer.extrude(writer.x(), writer.y() + dy, nozzle_change_speed, LimitRamming);
                nozzle_change_emitted += (writer.pos() - before).norm() * nz_extrusion_flow;
                auto records = make_structural_emissions(writer.extrusions(), ram_z,
                    ram_height, StructuralRole::InteriorDeposit,
                    static_cast<unsigned int>(2 + block->block_id), road_begin);
                for (StructuralEmission &record : records) {
                    record.held_above_level = ram_z > m_z_pos + WT_EPSILON;
                    record.held_level       = ram_lifted || record.held_above_level;
                }
                result.structural_emissions.insert(result.structural_emissions.end(), records.begin(), records.end());
            }
            if (nozzle_change_budget > 0.f && nozzle_change_emitted + WT_EPSILON >= nozzle_change_budget)
                break;
            m_left_to_right = !m_left_to_right;
        }
        if (need_change_flow) {
            writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + std::to_string(ram_height) + "\n");
        }
        // The rows step up the block, so the last one is the top; later roads of the level start clear of it.
        if (ram_held) {
            float &clear_from = m_level_clear_from[block->block_id];
            clear_from = std::max(clear_from, writer.y() + 0.5f * m_nozzle_change_perimeter_width + ram_clear);
        }
        // A ramming held below the level climbs back to the level's Z. One held above stays: the
        // tool switch lifts the head and the arriving tool's block brings it down.
        if (ram_moves && ram_z < m_z_pos)
            writer.move_z(m_z_pos);

        writer.set_extrusion_flow(nz_extrusion_flow);
        block->cur_depth += nozzle_change_depth;
        block->last_nozzle_change_id = old_filament_id;
        // Rows held above the level stand over the tower top until a full level reaches them. Later levels
        // in this block lift their purge and ramming rows to them and start their block fill after them.
        if (ram_z > m_z_pos + WT_EPSILON)
            m_held_rammings.push_back({block->block_id, ram_z, block->cur_depth - nozzle_change_depth, block->cur_depth});
        // --- Post-ramming: re-arm nozzle change for travel phase ---
        if (!extruder_change) {
            int new_nozzle_id = m_multi_nozzle_group_result->is_support_dynamic_nozzle_map()
                                    ? get_nozzle_id(new_filament_id, m_cur_layer_id) : -1;
            writer.append(format_line_M632(new_filament_id, new_nozzle_id));
        }

        if (is_need_reverse_travel(m_current_tool, extruder_change)) {
            bool   left_to_right     = !m_left_to_right;
            int  tpu_line_count = nozzle_change_line_count;
            nozzle_change_speed *= 2; // due to nozzle change 2 perimeter
            float ramming_travel_time     = extruder_change ? m_filpar[m_current_tool].ramming_travel_time.first : m_filpar[m_current_tool].ramming_travel_time.second;
            float need_reverse_travel_dis = ramming_travel_time * nozzle_change_speed / 60.f;
            float real_travel_dis         = tpu_line_count * (xr - xl - 2 * m_perimeter_width);
            if (real_travel_dis < need_reverse_travel_dis)
                nozzle_change_speed *= real_travel_dis / need_reverse_travel_dis;
            writer.travel(writer.x(), writer.y() + dy/2);

            for (int i = 0; true; ++i) {
                need_reverse_travel_dis -= (xr - xl - 2 * m_perimeter_width);
                float offset_dis = 0.f;
                if (need_reverse_travel_dis < 0) {
                    offset_dis              = -need_reverse_travel_dis;
                }
                if (left_to_right)
                    writer.travel(xr - m_perimeter_width - offset_dis, writer.y(), nozzle_change_speed);
                else
                    writer.travel(xl + m_perimeter_width + offset_dis , writer.y(), nozzle_change_speed);
                if (need_reverse_travel_dis < EPSILON) break;
                if (i == tpu_line_count - 1)
                    break;

                writer.travel(writer.x(), writer.y() - dy);
                left_to_right = !left_to_right;
            }
        } else {
            result.wipe_path.push_back(writer.pos_rotated());
            if (m_left_to_right) {
                result.wipe_path.push_back(Vec2f(0, writer.pos_rotated().y()));
            } else {
                result.wipe_path.push_back(Vec2f(m_wipe_tower_width, writer.pos_rotated().y()));
            }
        }
        if (!extruder_change) writer.append(format_line_M633());
    }

    writer.append(format_nozzle_change_line(false, old_filament_id, new_filament_id));

    result.start_pos = writer.start_pos_rotated();
    result.origin_start_pos = initial_position;
    result.end_pos   = writer.pos_rotated();
    result.gcode     = writer.gcode();
    result.elapsed_time = writer.elapsed_time();
    result.is_extruder_change = extruder_change;
    // Structural records for the ramming roads were added in the loop above; the enclosing
    // tool-change result merges them once.
    return result;
}

WipeTower::ToolChangeResult WipeTower::finish_layer_new(bool extrude_perimeter, bool extrude_fill, bool extrude_fill_wall)
{
    assert(!this->layer_finished());
    m_current_layer_finished = true;

    WipeTowerWriter writer(m_layer_height, m_perimeter_width, m_gcode_flavor, m_filpar, m_enable_arc_fitting, m_travel_speed, m_lag_emit_block_z);
    writer.set_extrusion_flow(m_extrusion_flow)
        .set_z(m_z_pos)
        .set_initial_tool(m_current_tool)
        .set_y_shift(m_y_shift - (m_current_shape == SHAPE_REVERSED ? m_layer_info->toolchanges_depth() : 0.f));

    set_for_wipe_tower_writer(writer);

    writer.append_wipe_tower_start();

    // First-layer base: in mixed-nozzle modes draw at the capped flow boost. The width tag is
    // declared inside the bracket either way and reset before it closes.
    const float base_flow_ratio = (is_first_layer() && m_mixed_nozzle_slicing) ?
        base_flow_ratio_for_width(int(m_current_tool), m_perimeter_width) : 1.f;
    if (is_first_layer())
        writer.set_extrusion_flow(m_extrusion_flow * base_flow_ratio)
              .change_analyzer_line_width(base_flow_ratio * m_perimeter_width);

    // Slow down on the 1st layer.
    bool first_layer = is_first_layer();
    // BBS: speed up perimeter speed to 90mm/s for non-first layer
    float feedrate = is_first_layer() ? std::min(first_layer_speed() * 60.f, m_max_speed) : std::min(60.0f * m_filpar[m_current_tool].max_e_speed / m_extrusion_flow, m_max_speed);

    bool toolchanges_on_layer = m_layer_info->toolchanges_depth() > WT_EPSILON;

    std::vector<Vec2f> finish_rect_wipe_path;
    const bool         multi_block_fill = (m_wipe_tower_blocks.size() > 1) && (extrude_fill_wall || extrude_fill);

    // Build list of fill boxes: one per block when multi_block_fill, else one for whole tower.
    std::vector<box_coordinates> fill_boxes;
    std::vector<unsigned int> fill_domains;
    std::vector<StructuralEmission> structural_emissions;
    if (multi_block_fill) {
        for (const WipeTowerBlock &block : m_wipe_tower_blocks) {
            float block_fill_height = block.depth - 2 * m_perimeter_width;
            if (m_cur_layer_id >= 0 && size_t(m_cur_layer_id) < block.layer_depths.size())
                block_fill_height = block.layer_depths[m_cur_layer_id] - 2 * m_perimeter_width;
            if (block_fill_height <= WT_EPSILON)
                continue;
            fill_boxes.emplace_back(
                Vec2f(m_perimeter_width, block.start_depth + m_perimeter_width),
                m_wipe_tower_width - 2 * m_perimeter_width,
                block_fill_height);
            fill_domains.push_back(static_cast<unsigned int>(2 + block.block_id));
        }
    }
    if (fill_boxes.empty()) {
        float fill_box_depth = m_wipe_tower_depth - 2 * m_perimeter_width;
        if (m_wipe_tower_blocks.size() == 1)
            fill_box_depth = m_layer_info->depth - 2 * m_perimeter_width;
        fill_boxes.emplace_back(Vec2f(m_perimeter_width, m_perimeter_width), m_wipe_tower_width - 2 * m_perimeter_width, fill_box_depth);
        fill_domains.push_back(m_wipe_tower_blocks.size() == 1
            ? static_cast<unsigned int>(2 + m_wipe_tower_blocks.front().block_id) : 0u);
    }

    writer.set_initial_position((m_left_to_right ? fill_boxes.front().ru : fill_boxes.front().lu), m_wipe_tower_width, m_wipe_tower_depth, m_internal_rotation);

    bool solid_infill = (m_layer_info + 1 == m_plan.end()) ? false :
                        std::any_of((m_layer_info + 1)->tool_changes.begin(), (m_layer_info + 1)->tool_changes.end(),
                                    [this](const WipeTowerInfo::ToolChange &tch) { return m_filpar[tch.new_tool].is_soluble || m_filpar[tch.old_tool].is_soluble; });
    solid_infill |= first_layer && m_adhesion;

    for (size_t i = 0; i < fill_boxes.size(); ++i) {
        const box_coordinates &fill_box = fill_boxes[i];
        const size_t fill_begin = writer.extrusions().size();
        if (i > 0)
            writer.travel(m_left_to_right ? fill_box.ru : fill_box.lu);

        if (extrude_fill_wall && (fill_box.ru.y() - fill_box.rd.y() > WT_EPSILON))
            writer.rectangle_fill_box(this, fill_box, finish_rect_wipe_path, feedrate);
        // Extrude infill to support the material to be printed above.
        const float         dy    = (fill_box.lu.y() - fill_box.ld.y() - m_perimeter_width);
        float               left  = fill_box.lu.x() + 2 * m_perimeter_width;
        float               right = fill_box.ru.x() - 2 * m_perimeter_width;

        if (extrude_fill && dy > m_perimeter_width) {
            writer.travel(fill_box.ld + Vec2f(m_perimeter_width * 2, 0.f))
                .append(";--------------------\n"
                        "; CP EMPTY GRID START\n")
                .comment_with_value(" layer #", m_num_layer_changes + 1);

            if (solid_infill) {
                float sparse_factor = 1.5f; // 1=solid, 2=every other line, etc.
                if (first_layer) {          // the infill should touch perimeters
                    left -= m_perimeter_width;
                    right += m_perimeter_width;
                    sparse_factor = 1.f;
                }
                float y       = fill_box.ld.y() + m_perimeter_width;
                int   n       = dy / (m_perimeter_width * sparse_factor);
                float spacing = (dy - m_perimeter_width) / (n - 1);
                int   i       = 0;
                for (i = 0; i < n; ++i) {
                    writer.extrude(writer.x(), y, feedrate).extrude(i % 2 ? left : right, y);
                    y = y + spacing;
                }
                writer.extrude(writer.x(), fill_box.lu.y());
            } else {
                // The grid spans the open tower, so it is laid at the tool's own solid width.
                const float grid_flow    = structural_extrusion_flow(int(m_current_tool), m_extrusion_flow);
                const bool  grid_widened = grid_flow != m_extrusion_flow;
                if (grid_widened)
                    writer.set_extrusion_flow(grid_flow).change_analyzer_line_width(structural_road_width(int(m_current_tool)));
                // Extrude an inverse U at the left of the region and the sparse infill.
                writer.extrude(fill_box.lu + Vec2f(m_perimeter_width * 2, 0.f), feedrate);

                const int   n  = 1 + int((right - left) / m_bridging);
                const float dx = (right - left) / n;
                for (int i = 1; i <= n; ++i) {
                    float x = left + dx * i;
                    writer.travel(x, writer.y());
                    // As in finish_block: hold the grid road to the tower's own speed.
                    writer.extrude(x, i % 2 ? fill_box.rd.y() : fill_box.ru.y(), m_mixed_nozzle_slicing ? feedrate : 0.f);
                }
                if (grid_widened)
                    writer.set_extrusion_flow(m_extrusion_flow * base_flow_ratio).change_analyzer_line_width(base_flow_ratio * m_perimeter_width);

                finish_rect_wipe_path.clear();
                // BBS: add wipe_path for this case: only with finish rectangle
                finish_rect_wipe_path.emplace_back(writer.pos());
                finish_rect_wipe_path.emplace_back(Vec2f(left + dx * n, n % 2 ? fill_box.ru.y() : fill_box.rd.y()));
            }

            writer.append("; CP EMPTY GRID END\n"
                          ";------------------\n\n\n\n\n\n\n");
        }
        if (fill_domains[i] != 0) {
            auto records = make_structural_emissions(writer.extrusions(), m_z_pos, m_layer_height,
                StructuralRole::TowerBlock, fill_domains[i], fill_begin);
            structural_emissions.insert(structural_emissions.end(), records.begin(), records.end());
        }
    }

    const size_t perimeter_begin = writer.extrusions().size();
    // outer perimeter (always):
    // BBS
    float wipe_tower_depth = m_wipe_tower_depth;
    if (m_wipe_tower_blocks.size() == 1) {
        wipe_tower_depth = m_layer_info->depth + m_perimeter_width;
    }
    box_coordinates wt_box(Vec2f(0.f, 0.f), m_wipe_tower_width, wipe_tower_depth);
    wt_box = align_perimeter(wt_box);

    //if (extrude_perimeter && !m_use_rib_wall) {
    //    if (!m_use_gap_wall)
    //        writer.rectangle(wt_box, feedrate);
    //    else
    //        generate_support_wall(writer, wt_box, feedrate, first_layer);
    //}
    Polygon outer_wall;
    // The base wall is one continuous loop (no gap wall), so it never crosses between segments.
    // The wall uses the tool's own solid width, and so do the brim loops below.
    const float wall_flow = structural_extrusion_flow(int(m_current_tool), m_extrusion_flow);
    const bool  wall_widened = wall_flow != m_extrusion_flow;
    if (wall_widened)
        writer.set_extrusion_flow(wall_flow).change_analyzer_line_width(structural_road_width(int(m_current_tool)));
    outer_wall = generate_support_wall_new(writer, wt_box, feedrate, first_layer, m_use_rib_wall, extrude_perimeter,
                                           m_use_gap_wall && !is_tower_base_layer());
    // Back to the layer's flow, with the first-layer boost the brim loops below are laid at.
    if (wall_widened)
        writer.set_extrusion_flow(m_extrusion_flow * base_flow_ratio).change_analyzer_line_width(base_flow_ratio * m_perimeter_width);
    if (extrude_perimeter) {
        Polyline shift_polyline = to_polyline(outer_wall);
        shift_polyline.translate(0, scaled(m_y_shift));
        m_outer_wall[m_z_pos].push_back(shift_polyline);
    }
    // brim chamfer
    float spacing = m_perimeter_width - m_layer_height * float(1. - M_PI_4);
    // How many perimeters shall the brim have?
    int loops_num = (m_wipe_tower_brim_width + spacing / 2.f) / spacing;
    const float max_chamfer_width = 3.f;
    if (!first_layer) {
        // stop print chamfer if depth changes
        if (m_layer_info->depth != m_plan.front().depth) {
            loops_num = 0;
        } else {
            // limit max chamfer width to 3 mm
            int chamfer_loops_num = (int) (max_chamfer_width / spacing);
            int dist_to_1st       = m_layer_info - m_plan.begin() - m_first_layer_idx;
            loops_num             = std::min(loops_num, chamfer_loops_num) - dist_to_1st;
        }
    }
    // A tool wider than the shared width lays its brim and chamfer loops at its own width and
    // spacing, as it would on its own. Its bed-level brim is as wide as the configured brim, and a
    // chamfer loop never reaches past the shared chamfer, so the level below always carries it.
    const float brim_road_width = structural_road_width(int(m_current_tool));
    const bool  brim_widened    = brim_road_width > m_perimeter_width + 1e-4f;
    if (brim_widened) {
        const float wide_spacing = brim_road_width - m_layer_height * float(1. - M_PI_4);
        loops_num = first_layer ? int((m_wipe_tower_brim_width + wide_spacing / 2.f) / wide_spacing) :
                                  int(float(std::max(loops_num, 0)) * spacing / wide_spacing + 1e-3f);
        spacing   = wide_spacing;
        // The first-layer boost is capped for the road it widens, this one, not the shared width.
        const float brim_flow_ratio = (is_first_layer() && m_mixed_nozzle_slicing) ?
            base_flow_ratio_for_width(int(m_current_tool), brim_road_width) : 1.f;
        writer.set_extrusion_flow(structural_extrusion_flow(int(m_current_tool), m_extrusion_flow) * brim_flow_ratio)
              .change_analyzer_line_width(brim_flow_ratio * brim_road_width);
    }

    if (loops_num > 0) {
        //box_coordinates box = wt_box;
        for (size_t i = 0; i < loops_num; ++i) {
            outer_wall = offset(outer_wall, scaled(spacing)).front();
            writer.polygon(outer_wall, feedrate);
            m_outer_wall[m_z_pos].push_back(to_polyline(outer_wall));
        }

            /*for (size_t i = 0; i < loops_num; ++i) {
                box.expand(spacing);
                writer.rectangle(box, feedrate);
            }*/

        if (first_layer) {
            // Save actual brim width to be later passed to the Print object, which will use it
            // for skirt calculation and pass it to GLCanvas for precise preview box
            m_wipe_tower_brim_width_real = loops_num * spacing + spacing / 2.f;
            //m_wipe_tower_brim_width_real = wt_box.ld.x() - box.ld.x() + spacing / 2.f;
        }
        //wt_box = box;
    }

    if (brim_widened)
        writer.set_extrusion_flow(m_extrusion_flow * base_flow_ratio).change_analyzer_line_width(base_flow_ratio * m_perimeter_width);

    if (extrude_perimeter || loops_num > 0) {
        writer.add_wipe_path(outer_wall, m_filpar[m_current_tool].wipe_dist);
    }
    else {
        // Now prepare future wipe. box contains rectangle that was extruded last (ccw).
        Vec2f target = (writer.pos() == wt_box.ld ? wt_box.rd : (writer.pos() == wt_box.rd ? wt_box.ru : (writer.pos() == wt_box.ru ? wt_box.lu : wt_box.ld)));

        // BBS: add wipe_path for this case: only with finish rectangle
        if (finish_rect_wipe_path.size() == 2 && finish_rect_wipe_path[0] == writer.pos()) target = finish_rect_wipe_path[1];

        writer.add_wipe_point(writer.pos()).add_wipe_point(target);
    }
    // Restore flow and declared width before the bracket closes.
    if (is_first_layer())
        writer.set_extrusion_flow(m_extrusion_flow)
              .change_analyzer_line_width(m_perimeter_width);
    writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_End) + "\n");

    // Ask our writer about how much material was consumed.
    // Skip this in case the layer is sparse and config option to not print sparse layers is enabled.
    if (!m_no_sparse_layers || toolchanges_on_layer || m_layer_info->force_emit)
        if (m_current_tool < m_used_filament_length.size())
            m_used_filament_length[m_current_tool] += writer.get_and_reset_used_filament_length();

    m_nozzle_change_result.gcode.clear();
    m_nozzle_change_result.structural_emissions.clear();
    auto perimeter_records = make_structural_emissions(writer.extrusions(), m_z_pos, m_layer_height,
        StructuralRole::TowerPerimeter, 1u, perimeter_begin);
    structural_emissions.insert(structural_emissions.end(), perimeter_records.begin(), perimeter_records.end());
    auto result = construct_tcr(writer, false, m_current_tool, true, false, 0.f, false);
    result.structural_emissions = std::move(structural_emissions);
    return result;
}

WipeTower::ToolChangeResult WipeTower::finish_block(const WipeTowerBlock &block, int filament_id, bool extrude_fill)
{
    WipeTowerWriter writer(m_layer_height, m_perimeter_width, m_gcode_flavor, m_filpar, m_enable_arc_fitting, m_travel_speed, m_lag_emit_block_z);
    writer.set_extrusion_flow(m_extrusion_flow)
        .set_z(m_z_pos)
        .set_initial_tool(filament_id)
        .set_y_shift(m_y_shift - (m_current_shape == SHAPE_REVERSED ? m_layer_info->toolchanges_depth() : 0.f));

    set_for_wipe_tower_writer(writer);

    writer.append_wipe_tower_start();

    // Slow down on the 1st layer.
    bool first_layer = is_first_layer();
    // BBS: speed up perimeter speed to 90mm/s for non-first layer
    float feedrate = is_first_layer() ? std::min(first_layer_speed() * 60.f, m_max_speed) : std::min(60.0f * m_filpar[filament_id].max_e_speed / m_extrusion_flow, m_max_speed);

    box_coordinates fill_box(Vec2f(0, 0), 0, 0);
    fill_box = box_coordinates(Vec2f(m_perimeter_width, block.cur_depth), m_wipe_tower_width - 2 * m_perimeter_width, block.start_depth + block.layer_depths[m_cur_layer_id] - block.cur_depth - m_perimeter_width);

    writer.set_initial_position((m_left_to_right ? fill_box.ru : fill_box.lu), m_wipe_tower_width, m_wipe_tower_depth, m_internal_rotation);

    bool toolchanges_on_layer = m_layer_info->toolchanges_depth() > WT_EPSILON;

    std::vector<Vec2f> finish_rect_wipe_path;
    // inner perimeter of the sparse section, if there is space for it:
    if (fill_box.ru.y() - fill_box.rd.y() > WT_EPSILON) {
        writer.rectangle_fill_box(this, fill_box, finish_rect_wipe_path, feedrate);
    }

    // Extrude infill to support the material to be printed above.
    const float        dy    = (fill_box.lu.y() - fill_box.ld.y() - m_perimeter_width);
    float              left  = fill_box.lu.x() + 2 * m_perimeter_width;
    float              right = fill_box.ru.x() - 2 * m_perimeter_width;
    if (extrude_fill && dy > m_perimeter_width) {
        writer.travel(fill_box.ld + Vec2f(m_perimeter_width * 2, 0.f))
            .append(";--------------------\n"
                    "; CP EMPTY GRID START\n")
            .comment_with_value(" layer #", m_num_layer_changes + 1);

        // Is there a soluble filament wiped/rammed at the next layer?
        // If so, the infill should not be sparse.
        bool solid_infill = m_layer_info + 1 == m_plan.end() ?
                                false :
                                std::any_of((m_layer_info + 1)->tool_changes.begin(), (m_layer_info + 1)->tool_changes.end(),
                                            [this](const WipeTowerInfo::ToolChange &tch) { return m_filpar[tch.new_tool].is_soluble || m_filpar[tch.old_tool].is_soluble; });
        solid_infill |= first_layer && m_adhesion;

        if (solid_infill) {
            float sparse_factor = 1.5f; // 1=solid, 2=every other line, etc.
            if (first_layer) {          // the infill should touch perimeters
                left -= m_perimeter_width;
                right += m_perimeter_width;
                sparse_factor = 1.f;
            }
            float y       = fill_box.ld.y() + m_perimeter_width;
            int   n       = dy / (m_perimeter_width * sparse_factor);
            float spacing = (dy - m_perimeter_width) / (n - 1);
            int   i       = 0;
            for (i = 0; i < n; ++i) {
                writer.extrude(writer.x(), y, feedrate).extrude(i % 2 ? left : right, y);
                y = y + spacing;
            }
            writer.extrude(writer.x(), fill_box.lu.y());
        } else {
            // The grid spans the open tower, so it is laid at the tool's own solid width.
            const float grid_flow    = structural_extrusion_flow(filament_id, m_extrusion_flow);
            const bool  grid_widened = grid_flow != m_extrusion_flow;
            if (grid_widened)
                writer.set_extrusion_flow(grid_flow).change_analyzer_line_width(structural_road_width(filament_id));
            // Extrude an inverse U at the left of the region and the sparse infill.
            writer.extrude(fill_box.lu + Vec2f(m_perimeter_width * 2, 0.f), feedrate);

            const int   n  = 1 + int((right - left) / m_bridging);
            const float dx = (right - left) / n;
            for (int i = 1; i <= n; ++i) {
                float x = left + dx * i;
                writer.travel(x, writer.y());
                // travel() leaves the tower travel speed in force, so in mixed-nozzle modes name the
                // print feedrate explicitly rather than running the grid road at travel speed.
                writer.extrude(x, i % 2 ? fill_box.rd.y() : fill_box.ru.y(), m_mixed_nozzle_slicing ? feedrate : 0.f);
            }
            if (grid_widened)
                writer.set_extrusion_flow(m_extrusion_flow).change_analyzer_line_width(m_perimeter_width);
            finish_rect_wipe_path.clear();
            // BBS: add wipe_path for this case: only with finish rectangle
            finish_rect_wipe_path.emplace_back(writer.pos());
            finish_rect_wipe_path.emplace_back(Vec2f(left + dx * n, n % 2 ? fill_box.ru.y() : fill_box.rd.y()));
        }

        writer.append("; CP EMPTY GRID END\n"
                      ";------------------\n\n\n\n\n\n\n");
    }

    // outer perimeter (always):
    // BBS
    box_coordinates wt_box(Vec2f(0.f, 0.f), m_wipe_tower_width, m_layer_info->depth + m_perimeter_width);
    wt_box = align_perimeter(wt_box);

    // Now prepare future wipe. box contains rectangle that was extruded last (ccw).
    Vec2f target = (writer.pos() == wt_box.ld ? wt_box.rd : (writer.pos() == wt_box.rd ? wt_box.ru : (writer.pos() == wt_box.ru ? wt_box.lu : wt_box.ld)));

    // BBS: add wipe_path for this case: only with finish rectangle
    if (finish_rect_wipe_path.size() == 2 && finish_rect_wipe_path[0] == writer.pos()) target = finish_rect_wipe_path[1];

    writer.add_wipe_point(writer.pos()).add_wipe_point(target);

    writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_End) + "\n");

    // Ask our writer about how much material was consumed.
    // Skip this in case the layer is sparse and config option to not print sparse layers is enabled.
    if (!m_no_sparse_layers || toolchanges_on_layer || m_layer_info->force_emit)
        if (filament_id < m_used_filament_length.size())
            m_used_filament_length[filament_id] += writer.get_and_reset_used_filament_length();

    return construct_block_tcr(writer, false, filament_id, true, 0.f,
                               StructuralRole::TowerBlock, -1.f,
                               static_cast<unsigned int>(2 + block.block_id));
}

WipeTower::ToolChangeResult WipeTower::finish_block_solid(const WipeTowerBlock &block, int filament_id, bool extrude_fill, WipeTowerLayerType layer_type, int wall_tool)
{
    float layer_height = m_layer_height;
    float e_flow = m_extrusion_flow;
    if (m_cur_layer_id > 1 && block.layers_type[m_cur_layer_id - 1]==WipeTowerLayerType::Normal && m_extrusion_flow < extrusion_flow(0.2)
        && tool_can_thicken(filament_id)) {
        layer_height = 0.2;
        e_flow = extrusion_flow(0.2);
    }

    // A tool wider than the shared width lays the solid fill at its own width and spacing, so it is
    // as full as on a single nozzle. The rows move in from the wall and from the rows before them
    // by the extra width. Not on a contact level that starts the block, where the wall's skip
    // points (get_wall_skip_points) assume the shared rows.
    const bool is_full_block   = std::abs(block.cur_depth - block.start_depth) < EPSILON;
    const bool skip_point_fill = is_full_block && layer_type == WipeTowerLayerType::Contact && m_enable_tower_interface_features;
    const float shared_fill_depth = block.start_depth + block.layer_depths[m_cur_layer_id] - block.cur_depth - m_perimeter_width;
    float road_width  = m_perimeter_width;
    float wall_inset  = 0.f;
    float start_inset = 0.f;
    if (! skip_point_fill && structural_road_width(filament_id) > m_perimeter_width + 1e-4f) {
        const float width = structural_road_width(filament_id);
        // The rows sit as far from the wall's centreline as half of both widths.
        const float to_wall = 0.5f * (structural_road_width(wall_tool >= 0 ? wall_tool : filament_id) + width) - m_perimeter_width;
        // After this level's purge rows the block's depth assumed a shared-width first row.
        const float to_start = is_full_block ? to_wall : 0.5f * (width - m_perimeter_width);
        // A strip too shallow for the wider rows keeps the shared ones.
        if (shared_fill_depth - to_wall - to_start > 0.f) {
            road_width  = width;
            wall_inset  = to_wall;
            start_inset = to_start;
            e_flow      = layer_height * (width - layer_height * float(1. - M_PI_4)) / filament_area();
        }
    }
    const bool fill_widened = road_width > m_perimeter_width + 1e-4f;

    WipeTowerWriter writer(layer_height, road_width, m_gcode_flavor, m_filpar, m_enable_arc_fitting, m_travel_speed, m_lag_emit_block_z);
    writer.set_extrusion_flow(e_flow)
        .set_z(m_z_pos)
        .set_initial_tool(filament_id)
        .set_y_shift(m_y_shift - (m_current_shape == SHAPE_REVERSED ? m_layer_info->toolchanges_depth() : 0.f));

    set_for_wipe_tower_writer(writer);

    writer.append_wipe_tower_start();

    // Slow down on the 1st layer.
    // BBS: speed up perimeter speed to 90mm/s for non-first layer
    // A wider road keeps under the filament's volumetric limit too.
    const float speed_flow = fill_widened ? std::max(m_extrusion_flow, e_flow) : m_extrusion_flow;
    float feedrate = is_first_layer() ? std::min(first_layer_speed() * 60.f, m_max_speed) : std::min(60.0f * m_filpar[filament_id].max_e_speed / speed_flow, m_max_speed);
    feedrate       = (layer_type == WipeTowerLayerType::Contact || layer_type == WipeTowerLayerType::Contact_UP) ? 20.f * 60.f : feedrate;
    box_coordinates fill_box(Vec2f(0, 0), 0, 0);
    fill_box = box_coordinates(Vec2f(m_perimeter_width + wall_inset, block.cur_depth + start_inset), m_wipe_tower_width - 2 * (m_perimeter_width + wall_inset),
                               shared_fill_depth - start_inset - wall_inset);
    bool toolchanges_on_layer = m_layer_info->toolchanges_depth() > WT_EPSILON;
    const float dy                   = (fill_box.lu.y() - fill_box.ld.y());
    int   n                    = (dy + 0.25 * road_width) / road_width+1;
    float spacing              = road_width;
    Vec2f initial_pos(0, 0);
    bool        up_to_down = false;
    //set initial pos 
    {
        int   index      = m_cur_layer_id % 4;
        float gird_depth = spacing * (n-1);
        switch (index % 4) {
        case 0:
            initial_pos = fill_box.ld;
            m_left_to_right = true;
            up_to_down      = false;
            break;
        case 1:
            initial_pos     = Vec2f(fill_box.rd.x(), fill_box.rd.y() + gird_depth);
            m_left_to_right = false;
            up_to_down      = true;
            break;
        case 2:
            initial_pos     = fill_box.rd;
            m_left_to_right = false;
            up_to_down      = false;
            break;
        case 3:
            initial_pos     = Vec2f(fill_box.ld.x(), gird_depth + fill_box.ld.y());
            m_left_to_right = true;
            up_to_down      = true;
            break;
        default: break;
        }
    }
    // Extrude infill to support the material to be printed above.
    float              left  = fill_box.lu.x();
    float              right = fill_box.ru.x();
    std::vector<Vec2f> finish_rect_wipe_path;
    {
        writer
            .append(";--------------------\n"
                    "; CP EMPTY GRID START\n")
            .comment_with_value(" layer #", m_num_layer_changes + 1);
        if (is_full_block && layer_type == WipeTowerLayerType::Contact && m_enable_tower_interface_features ) {
            Vec2f        stop_pos                                    = initial_pos;
            float        filament_tower_interface_pre_extrusion_dist = m_filpar[m_current_tool].filament_tower_interface_pre_extrusion_dist;
            // Orca: unscaled(BoundingBox) here is a template returning BoundingBoxBase<Vec2d>, not BoundingBoxf
            auto         printer_bbx                                 = unscaled(get_extents(m_shared_print_bed));
            printer_bbx.translate((- m_wipe_tower_pos - m_rib_offset).cast<double>()); // first layer never be contact
            if (stop_pos.x() < m_wipe_tower_width/2.f)
                stop_pos = Vec2f(stop_pos.x() - filament_tower_interface_pre_extrusion_dist, stop_pos.y());
            else
                stop_pos = Vec2f(stop_pos.x() + filament_tower_interface_pre_extrusion_dist, stop_pos.y());
            if (stop_pos.x() < printer_bbx.min[0]) stop_pos.x() = printer_bbx.min[0];
            if (stop_pos.x() > printer_bbx.max[0]) stop_pos.x() = printer_bbx.max[0];
            initial_pos = stop_pos;
            if (m_filpar[m_current_tool].filament_tower_interface_print_temp != m_filpar[m_current_tool].nozzle_temperature)
                writer.format_line_M109(m_filpar[m_current_tool].filament_tower_interface_print_temp, get_extruder_id(m_current_tool, m_cur_layer_id));
                writer.retract(-m_filpar[m_current_tool].filament_tower_interface_pre_extrusion_length - 2.f, 100.f);
        }
        writer.set_initial_position(initial_pos, m_wipe_tower_width, m_wipe_tower_depth, m_internal_rotation);

        int   i             = 0;
        for (i = 0; i < n; ++i) {
            writer.extrude(m_left_to_right ? right : left, writer.y(), feedrate);
            if (i == n - 1) {
                writer.add_wipe_point(writer.pos()).add_wipe_point(Vec2f(m_left_to_right ? left : right, writer.y()));
                break;
            }
            m_left_to_right = !m_left_to_right;
            writer.extrude(writer.x(), writer.y()+spacing*(up_to_down?-1:1), feedrate);
        }
        if (layer_type == WipeTowerLayerType::Contact && m_enable_tower_interface_features && m_filpar[m_current_tool].filament_tower_interface_print_temp != m_filpar[m_current_tool].nozzle_temperature)
            writer.format_line_M104(m_filpar[m_current_tool].nozzle_temperature, get_extruder_id(m_current_tool, m_cur_layer_id));
        writer.append("; CP EMPTY GRID END\n"
                      ";------------------\n\n\n\n\n\n\n");
    }

    writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_End) + "\n");

    // Ask our writer about how much material was consumed.
    // Skip this in case the layer is sparse and config option to not print sparse layers is enabled.
    if (!m_no_sparse_layers || toolchanges_on_layer || m_layer_info->force_emit)
        if (filament_id < m_used_filament_length.size())
            m_used_filament_length[filament_id] += writer.get_and_reset_used_filament_length();

    return construct_block_tcr(writer, false, filament_id, true, 0.f,
                               StructuralRole::TowerBlock, layer_height,
                               static_cast<unsigned int>(2 + block.block_id));
}

void WipeTower::toolchange_wipe_new(WipeTowerWriter &writer, const box_coordinates &cleaning_box, float wipe_length,bool solid_tool_toolchange, float wipe_volume_budget, bool is_nozzle_change)
{
    // Purge rows are laid only by the incoming tool, so they use its own purge_width() rather
    // than the shared minimum-nozzle width.
    const float purge_flow  = purge_extrusion_flow(int(m_current_tool), m_layer_height);
    const float purge_width_here = purge_width(int(m_current_tool));
    // The base flow boost is capped by the road's width headroom; a purge row is already at its
    // tool's design width, so in mixed-nozzle modes it takes no boost.
    const float purge_flow_ratio = is_first_layer() ? base_flow_ratio_for_width(int(m_current_tool), purge_width_here) : 1.f;
    const float purge_flow_for_layer = purge_flow * purge_flow_ratio;
    writer.set_extrusion_flow(purge_flow_for_layer)
          // Orca: CP_TOOLCHANGE_WIPE is a standalone tag constant, not an ETags entry
          .append(";" + GCodeProcessor::Toolchange_Wipe_Tag + " CT" + std::to_string(solid_tool_toolchange) + " FL" + std::to_string(is_first_layer()) + "\n");
    if (!m_nozzle_change_result.gcode.empty())
        writer.change_analyzer_line_width(purge_width_here);
    // Restate the row height: a spliced nozzle change carries its own tags, and on a compact tower
    // the block height differs from the part's layer height.
    writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) +
                  std::to_string(m_layer_height) + "\n");

    // BBS: add the note for gcode-check, when the flow changed, the width should follow the change
    if (is_first_layer()) {
        writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Width) + std::to_string(purge_flow_ratio * purge_width_here) + "\n");
    }

    //if (solid_tool_toolchange && m_filpar[m_current_tool].filament_tower_interface_print_temp != m_filpar[m_current_tool].nozzle_temperature)
    //    writer.append(format_line_M109(m_filpar[m_current_tool].filament_tower_interface_print_temp, m_filament_map[this->m_current_tool] - 1));
    //if (solid_tool_toolchange && m_filpar[m_current_tool].filament_tower_interface_pre_extrusion_length != 0)
    //    writer.retract(-m_filpar[m_current_tool].filament_tower_interface_pre_extrusion_length, 100.f);

    float        retract_length = m_filpar[m_current_tool].retract_length;
    float        retract_speed  = m_filpar[m_current_tool].retract_speed * 60;
    const float &xl = cleaning_box.ld.x();
    const float &xr = cleaning_box.rd.x();
    bool         should_flat_ironging = m_flat_ironing;
    bool         should_line_ironing  = true;
    if (!m_contact_ironing && solid_tool_toolchange) {
        should_flat_ironging     = false;
        should_line_ironing = false;
    }
    // Captured before the nozzle-change override below: the short lead-in extrude is the row's
    // first move and must stay, or the full-width row would carry all its E as the first move.
    // Only the scrub after it is skipped on a nozzle change.
    bool         do_ironing_lead_in = should_line_ironing;
    // The colour-flush ironing scrub has nothing to flush on a nozzle change (the nozzles never
    // share filament); keep it for a same-nozzle colour change.
    if (is_nozzle_change) {
        should_flat_ironging = false;
        should_line_ironing  = false;
    }
    bool  should_cooling_before_tower = !solid_tool_toolchange;
    bool  should_cooling_before_object = false;//interface layer heating print tower, then cooling print object
    int cooling_begin_line = 2;
    float x_to_wipe = wipe_length;
    // In mixed-nozzle modes the row pitch follows the rows' own purge width. Off mode keeps
    // upstream's pitch (purge_width_here is the shared perimeter width there).
    float dy                 = m_mixed_nozzle_slicing
        ? (is_base_layer() ? purge_width_here : m_layer_info->extra_spacing * purge_width_here)
        : (is_first_layer() ? purge_width_here
                            : m_layer_info->extra_spacing * get_block_gap_width(int(m_current_tool), false));
    if (solid_tool_toolchange)
        dy = purge_width_here;
    x_to_wipe                = solid_tool_toolchange ? std::numeric_limits<float>::max(): x_to_wipe;
    // Mixed-nozzle only: E budget for the zigzag lines and stepovers, charged at each segment's
    // live flow, so the deposited volume matches the requested prime whichever flow is used.
    const bool  wipe_budget_active = m_mixed_nozzle_slicing && !solid_tool_toolchange && wipe_volume_budget > 0.f;
    const float wipe_budget_e      = wipe_budget_active ? wipe_volume_budget / filament_area() : 0.f;
    float       wipe_emitted_e     = 0.f;
    float target_speed       = is_first_layer() ? std::min(first_layer_speed() * 60.f, m_max_speed) : m_max_speed;
    target_speed             = solid_tool_toolchange ? m_contact_speed : target_speed;
    const std::vector<float> WipeSpeedMap{0.33f * target_speed, 0.375f * target_speed, 0.458f * target_speed, 0.875f * target_speed,
                                                       std::min(target_speed, 0.875f * target_speed + 50.f)};
    float                    wipe_speed = WipeSpeedMap[0];

    m_left_to_right = ((m_cur_layer_id + 3) % 4 >= 2);

    bool is_from_up = (m_cur_layer_id % 2 == 1);

    auto estimate_time_kernel = [&WipeSpeedMap,&xr,&xl](int n) {
        float time = std::numeric_limits<float>::max();
        float one_line_len = xr - xl;
        if (n <= 1)
            time = one_line_len / WipeSpeedMap[0];
        else if (n <= 2)
            time = one_line_len / WipeSpeedMap[0] + one_line_len / WipeSpeedMap[1];
        else if (n <= 3)
            time = one_line_len / WipeSpeedMap[0] + one_line_len / WipeSpeedMap[1] + one_line_len / WipeSpeedMap[2];
        else if (n <= 4)
            time = one_line_len / WipeSpeedMap[0] + one_line_len / WipeSpeedMap[1] + one_line_len / WipeSpeedMap[2] + one_line_len / WipeSpeedMap[3];
        else {
            time = one_line_len / WipeSpeedMap[0] + one_line_len / WipeSpeedMap[1] + one_line_len / WipeSpeedMap[2] + one_line_len / WipeSpeedMap[3];
            time += (n - 4) * one_line_len / WipeSpeedMap[4];
        }
        return time * 60.f;
    };
    auto estimate_wipe_time = [&estimate_time_kernel, & cleaning_box, &target_speed, &x_to_wipe, &xr, &xl, &dy, &WipeSpeedMap, &solid_tool_toolchange](int begin_line) -> float {
        int                      n            = std::ceil(x_to_wipe / (xr - xl));
        if (solid_tool_toolchange) n = (cleaning_box.lu[1] - cleaning_box.ld[1]) / dy;
        float total_time   = estimate_time_kernel(n);
        float beg_time   = n <= 0 ? 0 : estimate_time_kernel(begin_line);
        return total_time-beg_time;
    };

    bool should_heating = m_filpar[m_current_tool].filament_cooling_before_tower > EPSILON && !solid_tool_toolchange && !is_base_layer();
    auto add_M104_by_requirement = [&writer, &should_heating, this]() {
        if (m_filpar[m_current_tool].filament_cooling_before_tower < EPSILON) return;
        if (!should_heating) return;
        float target_temp = is_base_layer() ? m_filpar[m_current_tool].nozzle_temperature_initial_layer : m_filpar[m_current_tool].nozzle_temperature;
        writer.format_line_M104(target_temp, get_extruder_id(m_current_tool, m_cur_layer_id));
    };
    float speed_factor = 1.f;
    if (should_heating)
    {
        //No additional heating time is required.
        //float estimate_time = estimate_wipe_time(0);
        //int   extruder_id   = m_filament_map[m_current_tool] - 1;
        //float heat_time     = m_filpar[m_current_tool].filament_cooling_before_tower / m_hotend_heating_rate[extruder_id];
        //heat_time /= 2.f;
        //speed_factor = estimate_time / (heat_time+estimate_time);
        //wipe_speed *= speed_factor;
    }
    if (should_cooling_before_object) {
        int n = (cleaning_box.lu[1] - cleaning_box.ld[1]) / dy;
        int extruder_id = get_extruder_id(m_current_tool, m_cur_layer_id);
        float cooling_time     = (m_filpar[m_current_tool].filament_tower_interface_print_temp - m_filpar[m_current_tool].nozzle_temperature) / m_hotend_cooling_rate[extruder_id];
        if (n < 2) {
            float estimate_time = estimate_wipe_time(0);
            speed_factor        = estimate_time > cooling_time? 1: estimate_time / cooling_time;
            cooling_begin_line = 0;
        } else {
            float estimate_time = estimate_wipe_time(2);
            speed_factor        = estimate_time > cooling_time ? 1 : estimate_time / cooling_time;
            cooling_begin_line = 2; //TODO: No slowdown for the first two lines.
        }
        wipe_speed *= speed_factor;
    }


    m_purge_rows_top_y = writer.y();
    for (int i = 0; true; ++i) {
        if (i < WipeSpeedMap.size()) wipe_speed = WipeSpeedMap[i] * speed_factor;
        m_purge_rows_top_y = std::max(m_purge_rows_top_y, writer.y());

        bool need_change_flow = need_thick_bridge_flow(writer.y());
        // BBS: check the bridging area and use the bridge flow
        // Bridging flow also at this tool's purge width.
        if (need_change_flow) {
            writer.set_extrusion_flow(purge_extrusion_flow(int(m_current_tool), 0.2));
            writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + std::to_string(0.2) + "\n");
        }
        float flat_iron_area = m_filpar[m_current_tool].flat_iron_area;
        float ironing_length = 3.;

        if (should_cooling_before_object && i == cooling_begin_line) {
            writer.format_line_M104(m_filpar[m_current_tool].nozzle_temperature, get_extruder_id(m_current_tool, m_cur_layer_id));
        }

        // Net displacement over the line approximates xr-xl; ironing returns near its start.
        const Vec2f main_line_before = writer.pos();
        if (i == 0 && m_use_gap_wall) { // BBS: add ironing after extruding start
            if (m_left_to_right) {
                if (do_ironing_lead_in) {
                    float dx = xr + wipe_tower_wall_infill_overlap * m_perimeter_width - writer.pos().x();
                    if (abs(dx) < ironing_length) ironing_length = abs(dx);
                    writer.extrude(writer.x() + ironing_length, writer.y(), wipe_speed);
                    if (should_line_ironing) {
                        writer.retract(retract_length, retract_speed);
                        writer.travel(writer.x() - 1.5 * ironing_length, writer.y(), 600.);
                        if (should_flat_ironging) {
                            writer.travel(writer.x() + 0.5f * ironing_length, writer.y(), 240.);
                            Vec2f pos{writer.x() + 1.f * ironing_length, writer.y()};
                            writer.spiral_flat_ironing(writer.pos(), flat_iron_area, m_perimeter_width, flat_iron_speed);
                            writer.travel(pos, wipe_speed);
                        } else
                            writer.travel(writer.x() + 1.5 * ironing_length, writer.y(), 240.);
                        writer.retract(-retract_length, retract_speed);
                    }
                }
                add_M104_by_requirement();
                writer.extrude(xr + wipe_tower_wall_infill_overlap * m_perimeter_width, writer.y(), wipe_speed);
            } else {
                if (do_ironing_lead_in) {
                    float dx = xl - wipe_tower_wall_infill_overlap * m_perimeter_width - writer.pos().x();
                    if (abs(dx) < ironing_length) ironing_length = abs(dx);
                    writer.extrude(writer.x() - ironing_length, writer.y(), wipe_speed);
                    if (should_line_ironing) {
                        writer.retract(retract_length, retract_speed);
                        writer.travel(writer.x() + 1.5 * ironing_length, writer.y(), 600.);
                        if (should_flat_ironging) {
                            writer.travel(writer.x() - 0.5f * ironing_length, writer.y(), 240.);
                            Vec2f pos{writer.x() - 1.0f * ironing_length, writer.y()};
                            writer.spiral_flat_ironing(writer.pos(), flat_iron_area, m_perimeter_width, flat_iron_speed);
                            writer.travel(pos, wipe_speed);
                        } else
                            writer.travel(writer.x() - 1.5 * ironing_length, writer.y(), 240.);
                        writer.retract(-retract_length, retract_speed);
                    }
                }
                add_M104_by_requirement();
                writer.extrude(xl - wipe_tower_wall_infill_overlap * m_perimeter_width, writer.y(), wipe_speed);
            }
        } else {
            if (i == 0) add_M104_by_requirement();
            if (m_left_to_right)
                writer.extrude(xr + wipe_tower_wall_infill_overlap * m_perimeter_width, writer.y(), wipe_speed);
            else
                writer.extrude(xl - wipe_tower_wall_infill_overlap * m_perimeter_width, writer.y(), wipe_speed);
        }

        if (wipe_budget_active) {
            const float line_flow = need_change_flow ? purge_extrusion_flow(int(m_current_tool), 0.2f) : purge_flow_for_layer;
            wipe_emitted_e += (writer.pos() - main_line_before).norm() * line_flow;
        }

        // BBS: recover the flow in non-bridging area
        // Recover to this purge's own flow, not the shared one.
        if (need_change_flow) {
            writer.set_extrusion_flow(purge_flow_for_layer);
            writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + std::to_string(m_layer_height) + "\n");
        }

        if (!is_from_up && (writer.y() + dy - float(EPSILON) >cleaning_box.lu.y() - m_perimeter_width))
            break; // in case next line would not fit

        if (is_from_up && (writer.y() - dy+ float(EPSILON))<cleaning_box.ld.y()) // Because the top of the clean box cannot have wiring, but the bottom can have wiring.
            break;

        // Stop once the requested prime volume has been deposited.
        if (wipe_budget_active && wipe_emitted_e + WT_EPSILON >= wipe_budget_e)
            break;

        x_to_wipe -= (xr - xl);
        if (x_to_wipe < WT_EPSILON) {
            // BBS: Delete some unnecessary travel
            // writer.travel(m_left_to_right ? xl + 1.5f*m_perimeter_width : xr - 1.5f*m_perimeter_width, writer.y(), 7200);
            break;
        }
        // stepping to the next line:
        {
            const Vec2f before = writer.pos();
            if (is_from_up)
                writer.extrude(writer.x(), writer.y() - dy);
            else
                writer.extrude(writer.x(), writer.y() + dy);
            if (wipe_budget_active)
                wipe_emitted_e += (writer.pos() - before).norm() * purge_flow_for_layer;
        }
        if (wipe_budget_active && wipe_emitted_e + WT_EPSILON >= wipe_budget_e)
            break;

        m_left_to_right = !m_left_to_right;
    }

    writer.add_wipe_point(writer.x(), writer.y()).add_wipe_point(!m_left_to_right ? m_wipe_tower_width : 0.f, writer.y());

    if (m_layer_info != m_plan.end() && m_current_tool != m_layer_info->tool_changes.back().new_tool) m_left_to_right = !m_left_to_right;

    writer.set_extrusion_flow(m_extrusion_flow); // Reset the extrusion flow.
    // BBS: add the note for gcode-check when the flow changed
    if (is_first_layer()) { writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Width) + std::to_string(m_perimeter_width) + "\n"); }
}

WipeTower::WipeTowerBlock * WipeTower::get_block_by_category(int filament_adhesiveness_category, bool create)
{
    auto iter = std::find_if(m_wipe_tower_blocks.begin(), m_wipe_tower_blocks.end(), [&filament_adhesiveness_category](const WipeTower::WipeTowerBlock &item) {
        return item.filament_adhesiveness_category == filament_adhesiveness_category;
    });

    if (iter != m_wipe_tower_blocks.end()) {
        return &(*iter);
    }

    if (create) {
        WipeTower::WipeTowerBlock new_block;
        new_block.block_id = m_wipe_tower_blocks.size();
        new_block.filament_adhesiveness_category = filament_adhesiveness_category;
        m_wipe_tower_blocks.emplace_back(new_block);
        return &m_wipe_tower_blocks.back();
    }

    return nullptr;
}

void WipeTower::add_depth_to_block(int filament_id, int filament_adhesiveness_category, float depth, bool is_nozzle_change)
{
    std::vector<WipeTower::BlockDepthInfo> &layer_depth = m_all_layers_depth[m_cur_layer_id];
    auto iter = std::find_if(layer_depth.begin(), layer_depth.end(), [&filament_adhesiveness_category](const WipeTower::BlockDepthInfo &item) {
        return item.category == filament_adhesiveness_category;
    });

    if (iter != layer_depth.end()) {
        iter->depth += depth;
        if (is_nozzle_change)
            iter->nozzle_change_depth += depth;
    }
    else {
        WipeTower::BlockDepthInfo new_block;
        new_block.category = filament_adhesiveness_category;
        new_block.depth = depth;
        if (is_nozzle_change)
            new_block.nozzle_change_depth += depth;
        layer_depth.emplace_back(std::move(new_block));
    }
}

int WipeTower::get_filament_category(int filament_id)
{
    if (filament_id >= m_filament_categories.size())
        return 0;
    return m_filament_categories[filament_id];
}

void WipeTower::reset_block_status()
{
    for (auto &block : m_wipe_tower_blocks) {
        block.cur_depth = block.start_depth;
        block.last_filament_change_id = -1;
        block.last_nozzle_change_id   = -1;
    }
}
void WipeTower::set_nozzle_last_layer_id()
{
    for (int idx = 0; idx < m_plan.size(); idx++) {
        auto &info = m_plan[idx];
        for(int i =0 ; i<info.tool_changes.size();i++) {
            int old_tool = info.tool_changes[i].old_tool;
            int new_tool = info.tool_changes[i].new_tool;
            if (old_tool >= 0) m_last_layer_id[get_extruder_id(old_tool, idx)] = idx;
            m_last_layer_id[get_extruder_id(new_tool, idx)] = idx;
        }
    }
}

void WipeTower::set_first_layer_flow_ratio(const float flow_ratio)
{
    m_first_layer_flow_ratio = flow_ratio;
}

// Orca: default/initial-layer/travel acceleration are object-scope options here (PrintConfig
// members read directly in the BBS ctor), so Print resolves the columns and pushes them in.
void WipeTower::set_accelerations(const std::vector<double> &normal, const std::vector<double> &first_layer_normal,
                                  const std::vector<double> &travel, const std::vector<double> &first_layer_travel)
{
    auto to_accels = [](const std::vector<double> &values, std::vector<unsigned int> &accels) {
        accels.clear();
        for (double value : values)
            accels.emplace_back((unsigned int) floor(value + 0.5));
    };
    to_accels(normal, m_normal_accels);
    to_accels(first_layer_normal, m_first_layer_normal_accels);
    to_accels(travel, m_travel_accels);
    to_accels(first_layer_travel, m_first_layer_travel_accels);
}

void WipeTower::update_all_layer_depth(float wipe_tower_depth)
{
    m_wipe_tower_depth = 0.f;
    float start_offset = m_perimeter_width;
    float start_depth = start_offset;
    for (auto& block : m_wipe_tower_blocks) {
        block.depth *= m_extra_spacing;
        block.start_depth = start_depth;
        start_depth += block.depth;
        m_wipe_tower_depth += block.depth;

        for (auto& layer_depth : block.layer_depths) {
            layer_depth *= m_extra_spacing;
        }

        for (WipeTowerInfo& plan_info : m_plan) {
            plan_info.depth *= m_extra_spacing;
        }
    }
    if (m_wipe_tower_depth > 0)
        m_wipe_tower_depth += start_offset;

    if (m_enable_wrapping_detection || m_enable_timelapse_print) {
        if (is_approx(m_wipe_tower_depth, 0.f))
            m_wipe_tower_depth = wipe_tower_depth;
        for (WipeTowerInfo &plan_info : m_plan) {
            plan_info.depth = m_wipe_tower_depth;
        }
    }
}

void WipeTower::generate_wipe_tower_blocks(bool add_solid_flag)
{
    // 1. generate all layer depth
    m_all_layers_depth.clear();
    m_all_layers_depth.resize(m_plan.size());
    m_cur_layer_id = 0;
    for (auto& info : m_plan) {
        for (const WipeTowerInfo::ToolChange &tool_change : info.tool_changes) {
            if (!is_need_ramming(tool_change.old_tool, tool_change.new_tool, m_cur_layer_id)) {
                int filament_adhesiveness_category = get_filament_category(tool_change.new_tool);
                add_depth_to_block(tool_change.new_tool, filament_adhesiveness_category, tool_change.required_depth);
            }
            else {
                int old_filament_category = get_filament_category(tool_change.old_tool);
                add_depth_to_block(tool_change.old_tool, old_filament_category, tool_change.nozzle_change_depth, true);
                int new_filament_category = get_filament_category(tool_change.new_tool);
                add_depth_to_block(tool_change.new_tool, new_filament_category, tool_change.required_depth - tool_change.nozzle_change_depth);

            }
        }
        ++m_cur_layer_id;
    }

    // 2. generate all layer depth
    std::vector<std::unordered_map<int, float>> all_layer_category_to_depth(m_plan.size());
    for (size_t layer_id = 0; layer_id < m_all_layers_depth.size(); ++layer_id) {
        const auto& layer_blocks = m_all_layers_depth[layer_id];
        std::unordered_map<int, float> &category_to_depth = all_layer_category_to_depth[layer_id];
        for (auto block : layer_blocks) {
            category_to_depth[block.category] = block.depth;
        }
    }

    // 3. generate wipe tower block
    m_wipe_tower_blocks.clear();
    for (int layer_id = 0; layer_id < all_layer_category_to_depth.size(); ++layer_id) {
        const auto &layer_category_depths = all_layer_category_to_depth[layer_id];
        for (auto iter = layer_category_depths.begin(); iter != layer_category_depths.end(); ++iter) {
            auto* block = get_block_by_category(iter->first, true);
            if (block->layer_depths.empty()) {
                block->layer_depths.resize(all_layer_category_to_depth.size(), 0);
                block->finish_depth.resize(all_layer_category_to_depth.size(), 0);
                block->layers_type.resize(all_layer_category_to_depth.size(), WipeTowerLayerType::Normal);
            }
            block->depth = std::max(block->depth, iter->second);
            block->layer_depths[layer_id] = iter->second;
        }
    }
    // 4. get real depth for every layer
    for (int layer_id = m_plan.size() - 1; layer_id >= 0; --layer_id) {
        m_plan[layer_id].depth = 0;
        for (auto& block : m_wipe_tower_blocks) {
            if (layer_id < m_plan.size() - 1)
                block.layer_depths[layer_id] = std::max(block.layer_depths[layer_id], block.layer_depths[layer_id + 1]);
            m_plan[layer_id].depth += block.layer_depths[layer_id];
        }
    }

    if (m_tower_framework) {
        for (int layer_id = 1; layer_id < m_plan.size(); ++layer_id) {
            m_plan[layer_id].depth = 0;
            for (auto &block : m_wipe_tower_blocks) {
                block.layer_depths[layer_id] = block.layer_depths[0];
                m_plan[layer_id].depth += block.layer_depths[layer_id];
            }
        }
    }

        // add solid infill flag
    if (add_solid_flag) {
        int solid_infill_layer_low = 4;
        std::vector<std::unordered_set<int>> layers_used_tools;

        int first_tool = -1;
        for (const auto &layer : m_plan) {
            if (!layer.tool_changes.empty()) {
                first_tool = layer.tool_changes.front().old_tool;
                break;
            }
        }
        for (auto &info : m_plan) {
            std::unordered_set<int> used_tools;
            if (info.tool_changes.empty()) {
                used_tools.insert(get_filament_category(first_tool));
            } else {
                for (const WipeTowerInfo::ToolChange &tool_change : info.tool_changes) {
                    used_tools.insert(get_filament_category(tool_change.old_tool));
                    used_tools.insert(get_filament_category(tool_change.new_tool));
                }
                first_tool = info.tool_changes.back().new_tool;
            }
            layers_used_tools.push_back(used_tools);
        }

        for (WipeTowerBlock &block : m_wipe_tower_blocks) {
            for (int layer_id = 0; layer_id < all_layer_category_to_depth.size(); ++layer_id) {
                std::unordered_map<int, float> &category_to_depth = all_layer_category_to_depth[layer_id];
                if (category_to_depth[block.filament_adhesiveness_category] < block.layer_depths[layer_id] - m_perimeter_width) {
                    bool cur_has_block_category = layers_used_tools[layer_id].count(block.filament_adhesiveness_category);
                    int  layer_count            = solid_infill_layer_low;
                    while (layer_count > 0) {
                        if (layer_id + layer_count < all_layer_category_to_depth.size()) {
                            std::unordered_map<int, float> &up_layer_depth = all_layer_category_to_depth[layer_id + layer_count];
                            {
                                bool up_has_block_category = layers_used_tools[layer_id + layer_count].count(block.filament_adhesiveness_category);
                                if (cur_has_block_category != up_has_block_category) {
                                    block.layers_type[layer_id] = WipeTowerLayerType::Solid;
                                    break;
                                }
                            }
                        }
                        --layer_count;
                    }
                }
                if (layer_id > 0) {
                    bool cur_has_block_category = layers_used_tools[layer_id].count(block.filament_adhesiveness_category);
                    bool pre_has_block_category = layers_used_tools[layer_id - 1].count(block.filament_adhesiveness_category);
                    if (cur_has_block_category != pre_has_block_category) { block.layers_type[layer_id] = WipeTowerLayerType::Contact; }
                    if (block.layers_type[layer_id - 1] == WipeTowerLayerType::Contact && block.layers_type[layer_id] != WipeTowerLayerType::Contact) {
                        block.layers_type[layer_id] = WipeTowerLayerType::Contact_UP;
                    }
                }
                // The tower base is solid all the way through.
                if (m_tower_base_top_z > 0.f && layer_id < int(m_plan.size()) &&
                    m_plan[layer_id].z <= m_tower_base_top_z + float(EPSILON))
                    block.layers_type[layer_id] = WipeTowerLayerType::Solid;
            }
        }
    }

}
void WipeTower::calc_block_infill_gap()
{
    //1.calc block infill gap width
    struct BlockInfo
    {
        bool has_ramming = false;
        bool has_reverse_travel = false;
        float depth              = 0.f;
    };
    std::unordered_map<int, BlockInfo> block_info;
    std::unordered_map<int, BlockInfo> high_block_info;
    for (int i= (int)m_plan.size()-1;i>=0;i--)
    {
        for (auto &toolchange : m_plan[i].tool_changes) {
            int new_tool =toolchange.new_tool;
            int old_tool =toolchange.old_tool;
            if (is_need_ramming(old_tool,new_tool, i)) {
                bool extruder_change = !is_same_extruder(new_tool, old_tool, i);
                block_info[m_filpar[old_tool].category].has_ramming=true;
                if (is_need_reverse_travel(old_tool, extruder_change)) block_info[m_filpar[old_tool].category].has_reverse_travel = true;
                block_info[m_filpar[old_tool].category].depth += toolchange.nozzle_change_depth;
            }
            if (!block_info.count(m_filpar[new_tool].category)) block_info.insert({m_filpar[new_tool].category,BlockInfo{}});
            block_info[m_filpar[new_tool].category].depth += toolchange.required_depth - toolchange.nozzle_change_depth;
        }
        for (auto &block : block_info) {
            if (high_block_info.count(block.first) && high_block_info[block.first].depth > block.second.depth)
                block.second.depth = high_block_info[block.first].depth;
        }
        high_block_info = block_info;

        for (auto &block : block_info) { block.second.depth = 0.f;}
        if (i == 0) block_info = high_block_info;
    }
    float max_depth = std::accumulate(block_info.begin(), block_info.end(), 0.f, [](float value, const std::pair<int,BlockInfo> &block) { return value + block.second.depth; });
    float height_to_depth = get_limit_depth_by_height(m_wipe_tower_height);
    float height_to_spacing = max_depth > height_to_depth ? 1.f : height_to_depth / max_depth;

    float spacing_ratio = m_extra_spacing - 1.f;
    float extra_width = spacing_ratio * m_perimeter_width;
    float line_gap_tol  = 2.f * m_nozzle_change_perimeter_width; //If the block's line_gap is greater than it, the block should be aligned.
    for (auto &info : block_info) {
        //case1: no ramming, it can always align
        if (!info.second.has_ramming) {
            m_block_infill_gap_width[info.first].first = m_block_infill_gap_width[info.first].second = extra_width + m_perimeter_width;
        }
        // case2: has ramming, but no reverse travel
        //
        else if (!info.second.has_reverse_travel) {
            float line_gap                              = m_nozzle_change_perimeter_width + extra_width;
            if (!m_use_rib_wall) line_gap *= height_to_spacing;
            if (line_gap < line_gap_tol) {
                m_block_infill_gap_width[info.first].first  = m_perimeter_width + extra_width;
                m_block_infill_gap_width[info.first].second = m_nozzle_change_perimeter_width + extra_width;
            } else {
                m_block_infill_gap_width[info.first].first = m_block_infill_gap_width[info.first].second = m_nozzle_change_perimeter_width + extra_width;
            }
        }
        // case 3: has ramming and reverse travel
        else {
            float extra_tpu_fix_spacing = m_tpu_fixed_spacing - 1.f;
            float line_gap = m_nozzle_change_perimeter_width + std::max(extra_tpu_fix_spacing * m_perimeter_width, extra_width);
            if (!m_use_rib_wall) line_gap = height_to_spacing * line_gap;
            if (line_gap < line_gap_tol) {
                m_block_infill_gap_width[info.first].first  = m_perimeter_width + extra_width;
                m_block_infill_gap_width[info.first].second = m_nozzle_change_perimeter_width + std::max(extra_tpu_fix_spacing * m_perimeter_width, extra_width);
            } else {
                m_block_infill_gap_width[info.first].first = m_block_infill_gap_width[info.first].second = m_nozzle_change_perimeter_width +
                                                                                                           std::max(extra_tpu_fix_spacing * m_perimeter_width, extra_width);
            }
        }
    }

    //2. recalculate toolchange depth
     for (int idx = 0; idx < m_plan.size(); idx++) {
        for (auto &toolchange : m_plan[idx].tool_changes) {
            toolchange = set_toolchange(toolchange.old_tool, toolchange.new_tool, m_plan[idx].height, toolchange.wipe_volume, toolchange.purge_volume,idx, toolchange.ram_length_override_mm, toolchange.wipe_volume_budget);
        }
     }
     m_extra_spacing = 1.f;
}

void WipeTower::plan_tower_new()
{
    // As in Print::wipe_tower_data: mixed-nozzle towers take the auto brim from height tiers.
    if (m_wipe_tower_brim_width < 0)
        m_wipe_tower_brim_width = m_mixed_nozzle_slicing
            ? mixed_nozzle_tower_brim_width(m_wipe_tower_height)
            : get_auto_brim_by_height(m_wipe_tower_height);
    // The base must be known before blocks are laid out, since its layers are solid.
    update_tower_base_extent();
    m_tower_temperature_held.assign(m_filpar.size(), 0);
    calc_block_infill_gap();
    // Minimum footprint for a tall mixed-nozzle tower, carried by the rectangle rather than the
    // rib diagonals. tower_footprint_floor() already accounts for the rib's reach.
    const float footprint_floor = tower_footprint_floor();
    const float footprint_floor_width = footprint_floor > 0.f ? align_ceil(footprint_floor, m_perimeter_width) : 0.f;
    if (footprint_floor_width > m_wipe_tower_width)
        m_wipe_tower_width = footprint_floor_width;
    if (m_use_rib_wall) {
        // recalculate wipe_tower_with and layer's depth
        generate_wipe_tower_blocks(false);
        float max_depth    = std::accumulate(m_wipe_tower_blocks.begin(), m_wipe_tower_blocks.end(), 0.f, [](float a, const auto &t) { return a + t.depth; }) + m_perimeter_width;
        float square_width = align_ceil(std::sqrt(max_depth * m_wipe_tower_width * m_extra_spacing), m_perimeter_width);
        if (!m_preserve_planned_width)
            m_wipe_tower_width = std::max(square_width, footprint_floor_width);
        for (int idx = 0; idx < m_plan.size(); idx++) {
            for (auto &toolchange : m_plan[idx].tool_changes) {
                toolchange = set_toolchange(toolchange.old_tool, toolchange.new_tool, m_plan[idx].height, toolchange.wipe_volume, toolchange.purge_volume,idx, toolchange.ram_length_override_mm, toolchange.wipe_volume_budget);
            }
        }
    }

    generate_wipe_tower_blocks(true);

    float max_depth = 0.f;
    for (const auto &block : m_wipe_tower_blocks) {
        max_depth += block.depth;
    }
    //std::cout << " after square " << m_wipe_tower_width << "  depth  " << max_depth << std::endl;

    float min_wipe_tower_depth = get_limit_depth_by_height(m_wipe_tower_height);

    // only for get m_extra_spacing
    {
        if (m_enable_wrapping_detection && max_depth < EPSILON) {
            max_depth = wrapping_wipe_tower_depth;
            if (m_use_rib_wall) {
                m_wipe_tower_width = max_depth;
            }
        }

        if (m_enable_timelapse_print && max_depth < EPSILON) {
            max_depth = min_wipe_tower_depth;
            if (m_use_rib_wall) { m_wipe_tower_width = max_depth; }
        }

        // Same floor on the depth axis.
        const float depth_floor = std::max(min_wipe_tower_depth, footprint_floor);
        if (max_depth > EPSILON && max_depth + EPSILON < depth_floor) {
            // The rib still absorbs the stock rule.
            if (m_use_rib_wall)
                m_rib_length = std::max(m_rib_length, min_wipe_tower_depth * (float) std::sqrt(2));
            // The mixed-nozzle floor is already net of the rib, so the block itself grows to it.
            if (!m_use_rib_wall || footprint_floor > 0.f)
                m_extra_spacing = std::max(depth_floor / max_depth, m_extra_spacing);
        }

        for (int idx = 0; idx < m_plan.size(); idx++) {
            auto &info = m_plan[idx];
            if (idx == 0 /*&& m_extra_spacing > 1.f + EPSILON*/) {
                // apply solid fill for the first layer
                info.extra_spacing = 1.f;
                for (auto &toolchange : info.tool_changes) {
                    //float x_to_wipe     = volume_to_length(toolchange.wipe_volume, m_perimeter_width, info.height);
                    float line_len      = m_wipe_tower_width - 2 * m_perimeter_width;
                    float wipe_depth = (toolchange.required_depth - toolchange.nozzle_change_depth) * m_extra_spacing;
                    // Purge rows are emitted at the incoming tool's width; count them in that pitch.
                    const float purge_row_width = purge_width(toolchange.new_tool);
                    float wipe_line_count = wipe_depth / purge_row_width;
                    float nozzle_change_depth = toolchange.nozzle_change_depth * m_extra_spacing;

                    int nozzle_change_line_count = (toolchange.nozzle_change_depth * m_extra_spacing + WT_EPSILON) / m_nozzle_change_perimeter_width;

                    toolchange.required_depth = wipe_depth + nozzle_change_depth;
                    toolchange.wipe_length = wipe_line_count * line_len;
                    toolchange.wipe_volume          = length_to_volume(toolchange.wipe_length, purge_row_width, info.height);
                    toolchange.nozzle_change_length = nozzle_change_line_count * (m_wipe_tower_width - (m_nozzle_change_perimeter_width + m_perimeter_width));
                    toolchange.nozzle_change_depth  = nozzle_change_depth;
                }
            } else {
                info.extra_spacing = m_extra_spacing;
                for (auto &toolchange : info.tool_changes) {
                    toolchange.required_depth *= m_extra_spacing;
                    toolchange.nozzle_change_depth *= m_extra_spacing;
                    // Pair the planned length with toolchange_wipe_new()'s incoming-tool purge
                    // flow and width so its row count and starting position share one geometry.
                    toolchange.wipe_length = volume_to_length(toolchange.wipe_volume,
                                                              purge_width(toolchange.new_tool), info.height);
                }
            }
        }
    }

    update_all_layer_depth(max_depth);
    set_nozzle_last_layer_id();
    if(m_use_gap_wall) get_all_wall_skip_points();
    float diagonal = sqrt(m_wipe_tower_depth * m_wipe_tower_depth + m_wipe_tower_width * m_wipe_tower_width);
    m_rib_length    = std::max({m_rib_length, diagonal});
    m_rib_length += m_extra_rib_length;
    m_rib_length = std::max(diagonal, m_rib_length);
    m_rib_width  = std::min(m_rib_width, std::min(m_wipe_tower_depth, m_wipe_tower_width) / 2.f); // Ensure that the rib wall of the wipetower are attached to the infill.

}

int WipeTower::get_wall_filament_for_all_layer()
{
    std::map<int, int> category_counts;
    std::map<int, int> filament_counts;
    int current_tool = m_current_tool;
    for (const auto &layer : m_plan) {
        if (layer.tool_changes.empty()){
            filament_counts[current_tool]++;
            category_counts[get_filament_category(current_tool)]++;
            continue;
        }
        std::unordered_set<int> used_tools;
        std::unordered_set<int> used_category;
        for (size_t i = 0; i < layer.tool_changes.size(); ++i) {
            if (i == 0) {
                filament_counts[layer.tool_changes[i].old_tool]++;
                category_counts[get_filament_category(layer.tool_changes[i].old_tool)]++;
                used_tools.insert(layer.tool_changes[i].old_tool);
                used_category.insert(get_filament_category(layer.tool_changes[i].old_tool));
            }
            if (!used_category.count(get_filament_category(layer.tool_changes[i].new_tool)))
                category_counts[get_filament_category(layer.tool_changes[i].new_tool)]++;
            if (!used_tools.count(layer.tool_changes[i].new_tool))
                filament_counts[layer.tool_changes[i].new_tool]++;
            used_tools.insert(layer.tool_changes[i].new_tool);
            used_category.insert(get_filament_category(layer.tool_changes[i].new_tool));
        }
        current_tool = layer.tool_changes.empty()?current_tool:layer.tool_changes.back().new_tool;
    }

    // std::vector<std::pair<int, int>> category_counts_vec;
    int selected_category = -1;
    int selected_count    = 0;

    for (auto iter = category_counts.begin(); iter != category_counts.end(); ++iter) {
        if (iter->second > selected_count) {
            selected_category = iter->first;
            selected_count    = iter->second;
        }
    }

    // std::sort(category_counts_vec.begin(), category_counts_vec.end(), [](const std::pair<int, int> &left, const std::pair<int, int>& right) {
    //     return left.second > right.second;
    // });

    int filament_id    = -1;
    int filament_count = 0;
    for (auto iter = filament_counts.begin(); iter != filament_counts.end(); ++iter) {
        // Bounds-checked read of the category table.
        if (get_filament_category(iter->first) == selected_category && iter->second > filament_count) {
            filament_id    = iter->first;
            filament_count = iter->second;
        }
    }
    return filament_id;
}

void WipeTower::set_lagging(float lag_max, int coarse_tool, std::vector<int> filament_extruders,
                            int coarse_extruder, float coarse_layer_height, float fine_layer_height)
{
    m_lag_max                = std::max(0.f, lag_max);
    m_lag_coarse_tool        = coarse_tool;
    m_lag_filament_extruders = std::move(filament_extruders);
    m_lag_coarse_extruder    = coarse_extruder;
    m_lag_coarse_height      = coarse_layer_height > 0.f ? coarse_layer_height : 0.f;
    m_lag_fine_height        = fine_layer_height > 0.f ? fine_layer_height : 0.f;
}

void WipeTower::plan_lagging_tower()
{
    // Rewrite the plan so the coarse tool builds the tower and the fine tool only primes on it,
    // and the tower is visited only at filament switches. Every level keeps its place, since the
    // exporter takes one generated level per scheduled event; only the deposit height (or whether
    // anything is deposited) changes.
    if (!lagging() || m_plan.empty())
        return;
    const float coarse_h = m_lag_coarse_height;
    const float fine_h   = m_lag_fine_height;
    const float base_top = m_tower_base_top_z;

    std::vector<WipeTowerInfo> lagged;
    lagged.reserve(m_plan.size() + 8);
    // The top of the last full tower layer. A prime road never moves it.
    float  tower_top       = 0.f;
    // The highest prime road laid on the base so far; a later base level must not sit under it.
    float  prime_top       = 0.f;
    size_t first_layer_idx = size_t(-1);
    unsigned int active_tool = m_current_tool;
    for (const WipeTowerInfo &source : m_plan)
        if (!source.tool_changes.empty()) {
            active_tool = source.tool_changes.front().old_tool;
            break;
        }
    // The coarse tool must never be left a step below its own minimum layer height. For each
    // level, the part Z of the next coarse arrival; inside that window switchless levels stay off
    // the tower and a fine switch lays only its prime road.
    const float coarse_min = (m_lag_coarse_tool >= 0 && size_t(m_lag_coarse_tool) < m_filpar.size()) ?
        m_filpar[size_t(m_lag_coarse_tool)].min_layer_height : 0.f;
    std::vector<float> next_coarse_arrival(m_plan.size(), std::numeric_limits<float>::max());
    {
        float next = std::numeric_limits<float>::max();
        for (size_t idx = m_plan.size(); idx-- > 0; ) {
            next_coarse_arrival[idx] = next;
            const WipeTowerInfo &level = m_plan[idx];
            // Any arrival on the coarse nozzle hands the tower to the coarse tool.
            if (std::any_of(level.tool_changes.begin(), level.tool_changes.end(),
                            [this](const WipeTowerInfo::ToolChange &tc) { return on_coarse_nozzle(int(tc.new_tool)); }))
                next = level.z;
        }
    }
    const auto too_close_under_coarse_arrival = [&](size_t idx, float part_z) {
        return coarse_min > 0.f && next_coarse_arrival[idx] < std::numeric_limits<float>::max() &&
               next_coarse_arrival[idx] - part_z < coarse_min - WT_EPSILON;
    };
    const auto idle_level = [&](const WipeTowerInfo &source, float part_z) {
        WipeTowerInfo level = source;
        level.part_z     = part_z;
        level.z          = tower_top;
        level.height     = fine_h;
        level.idle       = true;
        level.force_emit = false;
        return level;
    };
    // The height of a coarse arrival's first road on the tower top: up to the part, at most one coarse
    // layer, and above the bed never under the coarse minimum. Less than the minimum under the part,
    // the arrival lays only its prime road, one minimum step up (see the coarse run below).
    const auto coarse_step = [&](float top, float part_z) {
        const float room   = part_z - top;
        const float lowest = top > WT_EPSILON ? std::min(coarse_min, coarse_h) : 0.f;
        if (room > WT_EPSILON)
            return std::max(std::min(coarse_h, room), lowest);
        return lowest > 0.f ? lowest : coarse_h;
    };
    // The Z of the lowest road a visit above the base lays when the tower top is at top, by the
    // rules below: a coarse run's first step, a fine run's prime road, a maintenance step, and the
    // outgoing tool's ramming, which keeps to its own nozzle's heights (ramming_height()).
    const auto next_visit_first_road = [&](const WipeTowerInfo &next, float top, unsigned int active) {
        if (next.tool_changes.empty()) {
            const float height = on_coarse_nozzle(int(active)) ? coarse_h : fine_h;
            return top + std::min(height, std::max(next.z - top, 0.f));
        }
        const WipeTowerInfo::ToolChange &first = next.tool_changes.front();
        float step = fine_h;
        if (on_coarse_nozzle(int(first.new_tool)))
            step = coarse_step(top, next.z);
        return top + std::min(step, ramming_height(int(first.old_tool), step));
    };

    // Set while a base level is held back under a coarse arrival; that arrival then lays the whole step.
    bool base_step_held = false;
    for (size_t source_idx = 0; source_idx < m_plan.size(); ++source_idx) {
        const WipeTowerInfo &source = m_plan[source_idx];
        const float part_z = source.z;
        if (!source.tool_changes.empty())
            active_tool = source.tool_changes.back().new_tool;
        if (base_top > 0.f && part_z <= base_top + WT_EPSILON) {
            // Inside the base too, a held-back level lays nothing and a fine switch under a coarse
            // arrival lays only its prime road. A bed level without a switch is never held back.
            const bool on_bed = tower_top <= WT_EPSILON;
            if (!on_bed && source.tool_changes.empty() && !source.force_emit && too_close_under_coarse_arrival(source_idx, part_z)) {
                lagged.push_back(idle_level(source, part_z));
                base_step_held = true;
                continue;
            }
            // A raised coarse step can leave the tower above the part. A level whose tool cannot lay
            // the step left there lays nothing, or only its prime road.
            const bool coarse_arrives = std::any_of(source.tool_changes.begin(), source.tool_changes.end(),
                [this](const WipeTowerInfo::ToolChange &tc) { return on_coarse_nozzle(int(tc.new_tool)); });
            const bool under_minimum = !on_bed && !coarse_arrives && active_tool < m_filpar.size() &&
                (part_z - tower_top < m_filpar[active_tool].min_layer_height - WT_EPSILON || part_z < prime_top - WT_EPSILON);
            if (under_minimum && source.tool_changes.empty()) {
                lagged.push_back(idle_level(source, part_z));
                continue;
            }
            const bool held = too_close_under_coarse_arrival(source_idx, part_z);
            if (!source.tool_changes.empty() && !on_coarse_nozzle(int(source.tool_changes.back().new_tool)) &&
                (under_minimum || held)) {
                WipeTowerInfo level = source;
                level.z          = tower_top + fine_h;
                level.height     = fine_h;
                level.part_z     = part_z;
                level.wall_owner = int(source.tool_changes.back().new_tool);
                level.prime_only = true;
                // On the bed this is the tower's first layer, laid whole (brim, wall and fill) at the
                // first-layer height. Its tool changes stay prime-only, so the tower depth is unchanged,
                // but the tower top rises to it: otherwise the next level treats the bed as empty and
                // lays a fine prime at the same Z over it and a coarse step from the bed into it.
                const bool bed_level = on_bed && first_layer_idx == size_t(-1);
                if (bed_level) {
                    level.z        = part_z;
                    level.height   = part_z;
                    level.bed_fill = true;
                }
                if (first_layer_idx == size_t(-1))
                    first_layer_idx = lagged.size();
                prime_top = std::max(prime_top, level.z);
                if (bed_level)
                    tower_top = level.z;
                lagged.push_back(std::move(level));
                base_step_held = base_step_held || held;
                continue;
            }
            // The solid base keeps pace with the part, one level per part layer.
            WipeTowerInfo level = source;
            level.part_z = part_z;
            level.z      = part_z;
            level.height = std::max(part_z - tower_top, WT_EPSILON);
            // Fine arrivals after the last coarse one, as when a support interface filament is put
            // last so it never lays the tower's bottom (ToolOrdering). They prime one fine layer on
            // the new top, in the same visit.
            size_t coarse_end = source.tool_changes.size();
            while (coarse_end > 0 && ! on_coarse_nozzle(int(source.tool_changes[coarse_end - 1].new_tool)))
                -- coarse_end;
            // Right above the bed level a coarse arrival can come closer than its own minimum. It lays
            // a full minimum step, a little ahead of the part, and owns the level.
            if (!on_bed && coarse_end > 0 && level.height < coarse_min - WT_EPSILON) {
                level.z        = tower_top + coarse_min;
                level.height   = coarse_min;
                base_step_held = true;
            }
            std::vector<WipeTowerInfo::ToolChange> fine_after;
            // On the tower's bed level, and after a held-back window, they never lay the level.
            if ((base_step_held || tower_top <= WT_EPSILON) && coarse_end > 0 && coarse_end < source.tool_changes.size()) {
                fine_after.assign(source.tool_changes.begin() + long(coarse_end), source.tool_changes.end());
                level.tool_changes.assign(source.tool_changes.begin(), source.tool_changes.begin() + long(coarse_end));
            }
            // After a held-back window the step is too tall for the fine tool, so the first coarse
            // arrival of the run owns the whole level, wall included. So it does when the fine tool
            // that starts the visit cannot lay the step for another reason, as after fine-only levels
            // (fine skins on a coarse body's bottom) followed by layers without a tower visit.
            const unsigned int visiting_tool = source.tool_changes.empty() ? active_tool : source.tool_changes.front().old_tool;
            const bool too_tall_for_visitor = coarse_end > 0 && visiting_tool < m_filpar.size() &&
                !on_coarse_nozzle(int(visiting_tool)) && level.height > m_filpar[visiting_tool].max_layer_height + WT_EPSILON;
            if ((base_step_held || too_tall_for_visitor) && coarse_end > 0) {
                size_t run_begin = coarse_end - 1;
                while (run_begin > 0 && on_coarse_nozzle(int(source.tool_changes[run_begin - 1].new_tool)))
                    -- run_begin;
                level.wall_owner = int(source.tool_changes[run_begin].new_tool);
                // Fine arrivals ahead of that run cannot lay a step taller than the fine tool's maximum,
                // and the outgoing fine tool would ram under their purge. They only prime, one fine layer
                // on the held top, and the coarse run joins the same visit.
                const size_t ramming_tool = source.tool_changes[run_begin].old_tool;
                const bool fine_ahead = run_begin > 0 && ramming_tool < m_filpar.size() &&
                    level.height > m_filpar[ramming_tool].max_layer_height + WT_EPSILON &&
                    std::none_of(source.tool_changes.begin(), source.tool_changes.begin() + long(run_begin),
                                 [this](const WipeTowerInfo::ToolChange &tc) { return on_coarse_nozzle(int(tc.new_tool)); });
                if (fine_ahead) {
                    WipeTowerInfo prime = source;
                    prime.tool_changes.assign(source.tool_changes.begin(), source.tool_changes.begin() + long(run_begin));
                    prime.z          = tower_top + fine_h;
                    prime.height     = fine_h;
                    prime.part_z     = part_z;
                    prime.wall_owner = int(source.tool_changes[run_begin - 1].new_tool);
                    prime.prime_only = true;
                    if (first_layer_idx == size_t(-1))
                        first_layer_idx = lagged.size();
                    prime_top = std::max(prime_top, prime.z);
                    lagged.push_back(std::move(prime));
                    level.tool_changes.assign(source.tool_changes.begin() + long(run_begin), source.tool_changes.begin() + long(coarse_end));
                    level.joins_visit = true;
                }
            }
            base_step_held = false;
            if (first_layer_idx == size_t(-1))
                first_layer_idx = lagged.size();
            tower_top = level.z;
            lagged.push_back(std::move(level));
            if (! fine_after.empty()) {
                WipeTowerInfo prime = source;
                prime.tool_changes = std::move(fine_after);
                prime.z            = tower_top + fine_h;
                prime.height       = fine_h;
                prime.part_z       = part_z;
                prime.wall_owner   = int(prime.tool_changes.back().new_tool);
                prime.prime_only   = true;
                prime.joins_visit  = true;
                prime_top = std::max(prime_top, prime.z);
                lagged.push_back(std::move(prime));
            }
            continue;
        }
        if (source.tool_changes.empty()) {
            if (source.force_emit) {
                // ToolOrdering reserves maintenance visits before an irregular gap: build real
                // support with the active nozzle, at that nozzle's height. Additional levels stay
                // in the same exporter visit.
                const bool  coarse_visit = on_coarse_nozzle(int(active_tool));
                const float height       = coarse_visit ? coarse_h : fine_h;
                // The coarse tool never lays a maintenance step under its own minimum.
                const float lowest       = coarse_visit ? coarse_min : 0.f;
                bool first = true;
                do {
                    const float step = std::min(height, part_z - tower_top);
                    if (step <= WT_EPSILON || step < lowest - WT_EPSILON)
                        break;
                    WipeTowerInfo level = source;
                    level.z = tower_top + step;
                    level.height = step;
                    level.part_z = part_z;
                    level.wall_owner = int(active_tool);
                    level.force_emit = true;
                    level.catch_up = !first;
                    tower_top = level.z;
                    lagged.push_back(std::move(level));
                    first = false;
                } while (part_z - tower_top >= height - WT_EPSILON);
                // Every scheduled level keeps its place in the plan, deposit or not.
                if (first)
                    lagged.push_back(idle_level(source, part_z));
                continue;
            }
            // No switch, no visit. The level stays in the plan and in the generated stream so the
            // exporter's own count still lines up, and deposits nothing.
            WipeTowerInfo level = source;
            level.part_z     = part_z;
            // Raft and support levels without a tool change land here too.
            level.z          = tower_top;
            level.height     = fine_h;
            level.idle       = true;
            level.force_emit = false;
            lagged.push_back(std::move(level));
            continue;
        }
        if (first_layer_idx == size_t(-1))
            first_layer_idx = lagged.size();
        // Split the level's changes into runs of consecutive arrivals on one nozzle (not one
        // filament id: a nozzle may carry several filaments). A coarse run builds at the coarse
        // height, owned by its first arrival; a fine run only primes at the fine height, owned by
        // its last arrival. Runs after the first join the same exporter visit (joins_visit).
        size_t run_begin = 0;
        while (run_begin < source.tool_changes.size()) {
            const bool coarse_run = on_coarse_nozzle(int(source.tool_changes[run_begin].new_tool));
            size_t run_end = run_begin + 1;
            while (run_end < source.tool_changes.size() &&
                   on_coarse_nozzle(int(source.tool_changes[run_end].new_tool)) == coarse_run)
                ++ run_end;
            const bool whole_level = run_begin == 0 && run_end == source.tool_changes.size();
            WipeTowerInfo run = source;
            if (!whole_level) {
                run.tool_changes.assign(source.tool_changes.begin() + long(run_begin),
                                        source.tool_changes.begin() + long(run_end));
                run.joins_visit = run_begin > 0;
            }
            const unsigned int arriving = coarse_run ? source.tool_changes[run_begin].new_tool
                                                     : source.tool_changes[run_end - 1].new_tool;
            run_begin = run_end;
            if (coarse_run) {
                // Full coarse layers until the tower top is within one coarse layer of the part,
                // at least one. The tool change belongs to the first; the rest are catch-up layers
                // in the same visit (normally only just above the base).
                float top   = tower_top;
                // Closer than its minimum under the part, the coarse tool would lay a step ahead of
                // it, and the next nozzle change would ram higher still. It lays only its prime road
                // there, as a fine return does, and the next coarse layer covers it.
                const float lowest = std::min(coarse_min, coarse_h);
                if (top > WT_EPSILON && lowest > 0.f && part_z - top < lowest - WT_EPSILON) {
                    WipeTowerInfo level = run;
                    level.z          = top + lowest;
                    level.height     = lowest;
                    level.part_z     = part_z;
                    level.wall_owner = int(arriving);
                    level.prime_only = true;
                    level.held_above = true;
                    lagged.push_back(std::move(level));
                    continue;
                }
                bool  first = true;
                while (true) {
                    const float step = coarse_step(top, part_z);
                    WipeTowerInfo level = first ? run : WipeTowerInfo(top + step, step);
                    level.z          = top + step;
                    level.height     = step;
                    level.part_z     = part_z;
                    level.wall_owner = int(arriving);
                    if (!first) {
                        level.tool_changes.clear();
                        level.extruder_fill = source.extruder_fill;
                        level.catch_up      = true;
                        level.force_emit    = true;
                    }
                    top = level.z;
                    lagged.push_back(std::move(level));
                    first = false;
                    if (part_z - top < coarse_h - WT_EPSILON)
                        break;
                }
                tower_top = top;
            } else {
                // A fine return lays only its prime road, one fine layer on the tower top, without
                // moving the top; the next coarse layer covers it.
                WipeTowerInfo level = run;
                level.z          = tower_top + fine_h;
                level.height     = fine_h;
                level.part_z     = part_z;
                level.wall_owner = int(arriving);
                level.prime_only = true;
                lagged.push_back(std::move(level));
                // Before the first coarse arrival nothing bounds the lag, so the fine tool brings
                // the tower up itself whenever it falls more than m_lag_max behind. At the end of
                // a level it also builds ahead for the next visit, whose lowest road has to be
                // within m_lag_max of its own part layer: after layers without a visit, or where
                // the fine tool rams at its own maximum under a coarse step, that road sits lower
                // than this visit would leave it. Never above this part layer.
                size_t next_visit = m_plan.size();
                if (run_begin >= source.tool_changes.size())
                    for (next_visit = source_idx + 1; next_visit < m_plan.size(); ++ next_visit)
                        if (!m_plan[next_visit].tool_changes.empty() || m_plan[next_visit].force_emit)
                            break;
                const auto next_visit_too_low = [&](float top) {
                    return next_visit < m_plan.size() && top + fine_h <= part_z + WT_EPSILON &&
                           m_plan[next_visit].z - next_visit_first_road(m_plan[next_visit], top, active_tool) >
                               m_lag_max + WT_EPSILON;
                };
                float top = tower_top;
                while (m_lag_max > 0.f && (part_z - top > m_lag_max + WT_EPSILON || next_visit_too_low(top))) {
                    WipeTowerInfo fill(top + fine_h, fine_h);
                    fill.part_z        = part_z;
                    fill.wall_owner    = int(arriving);
                    fill.extruder_fill = source.extruder_fill;
                    fill.catch_up      = true;
                    fill.force_emit    = true;
                    top = fill.z;
                    lagged.push_back(std::move(fill));
                }
                tower_top = top;
            }
        }
    }
    m_plan            = std::move(lagged);
    m_first_layer_idx = first_layer_idx == size_t(-1) ? size_t(0) : first_layer_idx;
    // Re-derive purge and ramming geometry for the heights the tower actually deposits at.
    for (size_t idx = 0; idx < m_plan.size(); ++idx)
        for (auto &toolchange : m_plan[idx].tool_changes)
            toolchange = set_toolchange(toolchange.old_tool, toolchange.new_tool, m_plan[idx].height,
                                        toolchange.wipe_volume, toolchange.purge_volume, int(idx),
                                        toolchange.ram_length_override_mm, toolchange.wipe_volume_budget);
}

// Diagnostic dump of the plan: one line per level and one per tool change, with the fields road
// lengths and feedrates are derived from.
std::string WipeTower::format_plan() const
{
    std::ostringstream out;
    out.precision(17); // ios_base member, same as the row digest in Print.cpp
    for (size_t level = 0; level < m_plan.size(); ++ level) {
        const WipeTowerInfo &info = m_plan[level];
        out << "level " << level
            << " z=" << double(info.z)
            << " part_z=" << double(info.part_z)
            << " height=" << double(info.height)
            << " depth=" << double(info.depth)
            << " extra_spacing=" << double(info.extra_spacing)
            << " force_emit=" << (info.force_emit ? 1 : 0)
            << " idle=" << (info.idle ? 1 : 0)
            << " catch_up=" << (info.catch_up ? 1 : 0)
            << " prime_only=" << (info.prime_only ? 1 : 0)
            << " wall_owner=" << info.wall_owner << "\n";
        for (size_t at = 0; at < info.tool_changes.size(); ++ at) {
            const WipeTowerInfo::ToolChange &change = info.tool_changes[at];
            out << "level " << level << " change " << at
                << " leaves=" << change.old_tool
                << " enters=" << change.new_tool
                << " required_depth=" << double(change.required_depth)
                << " wipe_volume=" << double(change.wipe_volume)
                << " purge_volume=" << double(change.purge_volume)
                << " nozzle_change_depth=" << double(change.nozzle_change_depth) << "\n";
        }
    }
    return out.str();
}

std::string WipeTower::plan_digest() const
{
    if (m_plan_as_received.empty())
        return std::string();
    return "plan as received\n" + m_plan_as_received + "plan as generated\n" + format_plan();
}

void WipeTower::plan_levels()
{
    if (m_plan.empty())
        return;
    // Taken before the plan is rewritten, so plan_digest() can tell input from generation differences.
    m_plan_as_received = format_plan();
    //m_extra_spacing = 1.f;
    m_wipe_tower_height = m_plan.back().z;//real wipe_tower_height
    // The base must be known before the lagging rewrite (it keeps pace; the rest lags).
    // plan_tower_new() recomputes it on the rewritten plan.
    update_tower_base_extent();
    plan_lagging_tower();
    m_wipe_tower_height = m_plan.back().z;
    plan_tower_new();
}

void WipeTower::generate_new(std::vector<std::vector<WipeTower::ToolChangeResult>> &result)
{
    if (m_plan.empty())
        return;
    plan_levels();
    m_layer_info = m_plan.begin();

    for (const auto &layer : m_plan) {
        if (!layer.tool_changes.empty()) {
            m_current_tool = layer.tool_changes.front().old_tool;
            break;
        }
    }

    m_held_road_z.clear();
    m_held_rows_end.clear();
    m_held_spans.clear();
    m_held_rammings.clear();
    m_level_clear_from.clear();
    for (auto &used : m_used_filament_length) // reset used filament stats
        used = 0.f;

    int wall_filament = get_wall_filament_for_all_layer();

    std::vector<WipeTower::ToolChangeResult> layer_result;
    int index = 0;
    for (auto layer : m_plan) {
        reset_block_status();
        m_cur_layer_id = index++;
        set_layer(layer.z, layer.height, 0, false, layer.z == m_plan.back().z);
        // Index the plan directly: a lagging plan can have two levels at the same Z, which
        // set_layer()'s walk by Z cannot tell apart.
        m_layer_info       = m_plan.begin() + (long) m_cur_layer_id;
        m_part_z_pos       = layer.part_z > 0.f ? layer.part_z : layer.z;
        // Only a catch-up layer sits above the Z the exporter moved to, so only it emits its own Z.
        m_lag_emit_block_z = lagging() && layer.catch_up;
        if (layer.idle) {
            // Not a tower visit: report the part's Z and deposit nothing, keeping the exporter's
            // one-level-per-event count.
            ToolChangeResult idle = ToolChangeResult();
            idle.print_z       = m_part_z_pos;
            idle.tower_z_start = m_z_pos;
            idle.layer_height  = layer.height;
            idle.initial_tool  = int(m_current_tool);
            idle.new_tool      = int(m_current_tool);
            layer_result.emplace_back(std::move(idle));
            result.emplace_back(std::move(layer_result));
            continue;
        }
        if (m_layer_info->depth < m_perimeter_width) continue;
        if (m_wipe_tower_blocks.size() == 1) {
            if (m_layer_info->depth < m_wipe_tower_depth - m_perimeter_width) {
                // align y shift to perimeter width
                float dy  = m_extra_spacing * m_perimeter_width;
                m_y_shift = (m_wipe_tower_depth - m_layer_info->depth) / 2.f;
                m_y_shift = align_round(m_y_shift, dy);
            }
        }

        //get_wall_skip_points(layer);

        ToolChangeResult finish_layer_tcr;
        ToolChangeResult timelapse_wall;

        auto get_wall_filament_for_this_layer = [this, &layer, &wall_filament]() -> int {
            if (layer.tool_changes.size() == 0)
                return -1;

            int candidate_id = -1;
            for (size_t idx = 0; idx < layer.tool_changes.size(); ++idx) {
                if (idx == 0) {
                    if (layer.tool_changes[idx].old_tool == wall_filament && is_valid_last_layer(layer.tool_changes[idx].old_tool, this->m_cur_layer_id, layer.z))
                        return wall_filament;
                    else if (m_filpar[layer.tool_changes[idx].old_tool].category == m_filpar[wall_filament].category &&
                             is_valid_last_layer(layer.tool_changes[idx].old_tool, this->m_cur_layer_id, layer.z)) {
                        candidate_id = layer.tool_changes[idx].old_tool;
                    }
                }
                if (layer.tool_changes[idx].new_tool == wall_filament) {
                    return wall_filament;
                }

                if ((candidate_id == -1) && (m_filpar[layer.tool_changes[idx].new_tool].category == m_filpar[wall_filament].category))
                    candidate_id = layer.tool_changes[idx].new_tool;
            }
            return candidate_id == -1 ? layer.tool_changes[0].new_tool : candidate_id;
        };
        int wall_idx = get_wall_filament_for_this_layer();
        // On a lagging level the arriving tool draws everything, so per-tool height limits hold.
        if (layer.wall_owner >= 0)
            wall_idx = layer.wall_owner;
        // In a lagging tower's base, which keeps pace with the part, a coarse tool can hand over to a
        // fine one on a level one fine layer tall (coarse support in a fine body). The coarse tool
        // cannot lay that step, so the last fine arrival of the same material kind lays the wall and
        // any block the coarse tool would have finished. Levels the coarse tool can lay, and materials
        // the tower keeps apart, are left as they are.
        const auto fine_stand_in = [&](int tool) {
            if (!lagging() || layer.z - layer.height <= WT_EPSILON || tool < 0 || size_t(tool) >= m_filpar.size() ||
                !on_coarse_nozzle(tool) || layer.height >= m_filpar[size_t(tool)].min_layer_height - WT_EPSILON)
                return -1;
            for (auto change = layer.tool_changes.rbegin(); change != layer.tool_changes.rend(); ++ change)
                if (!on_coarse_nozzle(int(change->new_tool)) &&
                    m_filpar[change->new_tool].category == m_filpar[size_t(tool)].category)
                    return int(change->new_tool);
            return -1;
        };
        if (const int stand_in = fine_stand_in(wall_idx); stand_in >= 0)
            wall_idx = stand_in;
        // A fine return lays its prime road and no block fill.
        const bool prime_only = layer.prime_only;

        // Prints with the timelapse/wrapping wall never get a lagging tower (see
        // mixed_nozzle_tower_lagging), so the two need no reconciling here.
        bool only_generate_wall = m_enable_timelapse_print || (m_enable_wrapping_detection && m_slice_used_filaments <= 1);
        // this layer has no tool_change
        if (wall_idx == -1) {
            bool need_insert_solid_infill = false;
            for (const WipeTowerBlock &block : m_wipe_tower_blocks) {
                if (block.layers_type[m_cur_layer_id] != WipeTowerLayerType::Normal) {
                    need_insert_solid_infill = true;
                    break;
                }
            }

            if (need_insert_solid_infill) {
                wall_idx = m_current_tool;
            } else {
                if (only_generate_wall) {
                    timelapse_wall = only_generate_out_wall(true);
                }
                finish_layer_tcr = finish_layer_new(only_generate_wall ? false : true, layer.extruder_fill);
                std::for_each(m_wipe_tower_blocks.begin(), m_wipe_tower_blocks.end(), [this](WipeTowerBlock &block) {
                    block.finish_depth[this->m_cur_layer_id] = block.start_depth;
                });
            }
        }

        // generate tool change
        bool insert_wall = false;
        int  insert_finish_layer_idx = -1;
        if (wall_idx != -1 && only_generate_wall) {
            timelapse_wall = only_generate_out_wall(true);
        }
        for (int i = 0; i < int(layer.tool_changes.size()); ++i) {
            ToolChangeResult wall_gcode;
            // A held coarse prime level lays no wall: a wall that high would be run into by the next level's.
            if (i == 0 && (layer.tool_changes[i].old_tool == wall_idx) && !layer.held_above) {
                finish_layer_tcr = finish_layer_new(only_generate_wall ? false : true, false, false);
            }
            bool        solid_nozzlechange = false, solid_toolchange = false;
            const auto * block = get_block_by_category(m_filpar[layer.tool_changes[i].new_tool].category, false);
            if (block) solid_toolchange = block->layers_type[m_cur_layer_id] == WipeTowerLayerType::Contact;

            const auto * block2 = get_block_by_category(m_filpar[layer.tool_changes[i].old_tool].category, false);
            if(block2) solid_nozzlechange = block2->layers_type[m_cur_layer_id] == WipeTowerLayerType::Contact;
            layer_result.emplace_back(tool_change_new(layer.tool_changes[i].new_tool, solid_toolchange,solid_nozzlechange));

            if (i == 0 && (layer.tool_changes[i].old_tool == wall_idx)) {

            }
            else if (layer.tool_changes[i].new_tool == wall_idx && !layer.held_above) {
                finish_layer_tcr = finish_layer_new(only_generate_wall ? false : true, false, false);
                insert_finish_layer_idx = i;
            }
        }

        // insert finish block (a bed-level prime-only level still fills the whole first layer)
        if (wall_idx != -1 && (!prime_only || layer.bed_fill)) {
            if (layer.tool_changes.empty()) {
                finish_layer_tcr = finish_layer_new(only_generate_wall ? false : true, false, false);
            }

            for (WipeTowerBlock& block : m_wipe_tower_blocks) {
                block.finish_depth[m_cur_layer_id] = block.start_depth + block.depth;
                if (held_road_z(&block) > m_z_pos + WT_EPSILON)
                    if (const auto rows = m_held_rows_end.find(block.block_id); rows != m_held_rows_end.end())
                        block.cur_depth = std::max(block.cur_depth, rows->second);
                // Its first road keeps clear of ramming rows held above this level.
                if (const auto clear = m_level_clear_from.find(block.block_id); clear != m_level_clear_from.end())
                    block.cur_depth = std::max(block.cur_depth, clear->second + 0.5f * m_perimeter_width);
                if (block.cur_depth + EPSILON >= block.start_depth + block.layer_depths[m_cur_layer_id]-m_perimeter_width) {
                    continue;
                }
                int id = std::find_if(m_wipe_tower_blocks.begin(), m_wipe_tower_blocks.end(), [&](const WipeTowerBlock &b) { return &b == &block; }) - m_wipe_tower_blocks.begin();
                bool block_solid = block.layers_type[m_cur_layer_id] == WipeTowerLayerType::Contact || block.layers_type[m_cur_layer_id] == WipeTowerLayerType::Contact_UP ||
                                   block.layers_type[m_cur_layer_id] == WipeTowerLayerType::Solid;
                int finish_layer_filament = -1;
                if (block.last_filament_change_id != -1) {
                    finish_layer_filament = block.last_filament_change_id;
                } else if (block.last_nozzle_change_id != -1) {
                    finish_layer_filament = block.last_nozzle_change_id;
                }

                if (!layer.tool_changes.empty()) {
                    WipeTowerBlock * last_layer_finish_block = get_block_by_category(get_filament_category(layer.tool_changes.front().old_tool), false);
                    if (last_layer_finish_block && last_layer_finish_block->block_id == block.block_id && finish_layer_filament == -1)
                        finish_layer_filament = layer.tool_changes.front().old_tool;
                }

                if (finish_layer_filament == -1) {
                    finish_layer_filament = wall_idx;
                }
                // On a lagging level every block belongs to the arriving tool.
                if (layer.wall_owner >= 0)
                    finish_layer_filament = layer.wall_owner;
                if (const int stand_in = fine_stand_in(finish_layer_filament); stand_in >= 0)
                    finish_layer_filament = stand_in;
                // Cancel the block of the last layer
                if (!is_valid_last_layer(finish_layer_filament, m_cur_layer_id, layer.z)) continue;
                ToolChangeResult finish_block_tcr;
                if (block_solid) {
                    finish_block_tcr = finish_block_solid(block, finish_layer_filament, layer.extruder_fill, block.layers_type[m_cur_layer_id], wall_idx);
                    block.finish_depth[m_cur_layer_id] = block.start_depth + block.depth;
                }
                else {
                    finish_block_tcr = finish_block(block, finish_layer_filament, layer.extruder_fill);
                    block.finish_depth[m_cur_layer_id] = block.cur_depth;
                }

                bool has_inserted = false;
                {
                    auto fc_iter = std::find_if(layer_result.begin(), layer_result.end(),
                                                [&finish_layer_filament](const WipeTower::ToolChangeResult &item) { return item.new_tool == finish_layer_filament; });
                    if (fc_iter != layer_result.end()) {
                        *fc_iter = merge_tcr(*fc_iter, finish_block_tcr);
                        has_inserted = true;
                    }
                }

                if (block.last_filament_change_id == -1 && !has_inserted) {
                    auto nc_iter = std::find_if(layer_result.begin(), layer_result.end(),
                                                [&finish_layer_filament](const WipeTower::ToolChangeResult &item) { return item.initial_tool == finish_layer_filament; });
                    if (nc_iter != layer_result.end()) {
                        *nc_iter = merge_tcr(finish_block_tcr, *nc_iter);
                        has_inserted = true;
                    }
                }

                if (!has_inserted) {
                    if (finish_block_tcr.gcode.empty())
                        finish_block_tcr = finish_block_tcr;
                    else
                        finish_layer_tcr = merge_tcr(finish_layer_tcr, finish_block_tcr);
                }
            }
        }
        // record the contact layers of different categories
        if (layer_result.empty()) {
            // there is nothing to merge finish_layer with
            // An empty result still reports its level's Z, as the idle branch does, so it does not
            // read as a row at the bed.
            if (!is_valid_gcode(finish_layer_tcr.gcode)) {
                finish_layer_tcr.print_z       = m_part_z_pos > 0.f ? m_part_z_pos : m_z_pos;
                finish_layer_tcr.tower_z_start = m_z_pos;
                finish_layer_tcr.layer_height  = layer.height;
                finish_layer_tcr.initial_tool  = int(m_current_tool);
                finish_layer_tcr.new_tool      = int(m_current_tool);
            }
            layer_result.emplace_back(std::move(finish_layer_tcr));
        }
        else if (is_valid_gcode(finish_layer_tcr.gcode)) {
            if (insert_finish_layer_idx == -1)
                layer_result[0] = merge_tcr(finish_layer_tcr, layer_result[0]);
            else
                layer_result[insert_finish_layer_idx] = merge_tcr(layer_result[insert_finish_layer_idx], finish_layer_tcr);
        }

        if (only_generate_wall && !timelapse_wall.gcode.empty()) {
            layer_result.insert(layer_result.begin(), std::move(timelapse_wall));
        }
        if (layer.held_above) {
            for (ToolChangeResult &block : layer_result)
                for (StructuralEmission &emission : block.structural_emissions) {
                    emission.held_above_level = true;
                    emission.held_level       = true;
                    if (emission.role == StructuralRole::InteriorDeposit && emission.support_domain >= 2) {
                        float &held = m_held_road_z[int(emission.support_domain) - 2];
                        held = std::max(held, emission.z);
                    }
                }
            for (const WipeTowerBlock &block : m_wipe_tower_blocks)
                if (const auto held = m_held_road_z.find(block.block_id); held != m_held_road_z.end()) {
                    float &end = m_held_rows_end[block.block_id];
                    end = std::max(end, block.cur_depth);
                    m_held_spans[block.block_id].push_back({block.start_depth, block.cur_depth, held->second});
                }
        } else if (!layer.prime_only) {
            // A full level at or above a held road's top carries the tower past it.
            for (auto it = m_held_road_z.begin(); it != m_held_road_z.end();) {
                if (it->second <= layer.z + WT_EPSILON) {
                    m_held_rows_end.erase(it->first);
                    m_held_spans.erase(it->first);
                    it = m_held_road_z.erase(it);
                } else
                    ++ it;
            }
        }
        for (const HeldRamming &ramming : m_held_rammings) {
            float &held = m_held_road_z[ramming.block_id];
            held = std::max(held, ramming.z);
            float &end = m_held_rows_end[ramming.block_id];
            end = std::max(end, ramming.rows_end);
            m_held_spans[ramming.block_id].push_back({ramming.rows_start, ramming.rows_end, ramming.z});
        }
        m_held_rammings.clear();
        m_level_clear_from.clear();
        if (layer.catch_up && !result.empty() && !result.back().empty()) {
            // A catch-up layer merges into the visit below it and carries its own Z.
            std::vector<ToolChangeResult> &visit = result.back();
            for (ToolChangeResult &block : layer_result)
                if (is_valid_gcode(block.gcode))
                    visit.back() = merge_tcr(visit.back(), block);
            layer_result.clear();
        } else if (layer.joins_visit && !result.empty()) {
            // Remaining tool changes of the same part layer on the other nozzle: each keeps its own
            // result, in order, so the exporter pairs one result per tool change.
            std::vector<ToolChangeResult> &visit = result.back();
            for (ToolChangeResult &block : layer_result)
                visit.emplace_back(std::move(block));
            layer_result.clear();
        } else {
            result.emplace_back(std::move(layer_result));
        }
    }
    // Idle and prime-only lagging levels draw no outer wall.
    assert(m_outer_wall.size() <= m_plan.size());
}

#if 0
// Processes vector m_plan and calls respective functions to generate G-code for the wipe tower
// Resulting ToolChangeResults are appended into vector "result"
void WipeTower::generate(std::vector<std::vector<WipeTower::ToolChangeResult>> &result)
{
	if (m_plan.empty())
        return;

    m_extra_spacing = 1.f;

	plan_tower();
    // BBS
#if 0
    for (int i=0;i<5;++i) {
        save_on_last_wipe();
        plan_tower();
    }
#endif

    m_layer_info = m_plan.begin();

    // we don't know which extruder to start with - we'll set it according to the first toolchange
    for (const auto& layer : m_plan) {
        if (!layer.tool_changes.empty()) {
            m_current_tool = layer.tool_changes.front().old_tool;
            break;
        }
    }

    for (auto& used : m_used_filament_length) // reset used filament stats
        used = 0.f;

    m_old_temperature = -1; // reset last temperature written in the gcode

    std::vector<WipeTower::ToolChangeResult> layer_result;
    int index = 0;
	for (auto layer : m_plan)
	{
        m_cur_layer_id = index++;
        set_layer(layer.z, layer.height, 0, false/*layer.z == m_plan.front().z*/, layer.z == m_plan.back().z);
        // BBS
        //m_internal_rotation += 180.f;

        if (m_layer_info->depth < m_perimeter_width)
            continue;

        if (m_layer_info->depth < m_wipe_tower_depth - m_perimeter_width) {
            // align y shift to perimeter width
            float dy = m_extra_spacing * m_perimeter_width;
            m_y_shift = (m_wipe_tower_depth - m_layer_info->depth) / 2.f;
            m_y_shift = align_round(m_y_shift, dy);
        }

        // BBS: consider both soluable and support properties
        int idx = first_toolchange_to_nonsoluble_nonsupport (layer.tool_changes);
        ToolChangeResult finish_layer_tcr;
        ToolChangeResult timelapse_wall;

        if (idx == -1) {
            // if there is no toolchange switching to non-soluble, finish layer
            // will be called at the very beginning. That's the last possibility
            // where a nonsoluble tool can be.
            if (m_enable_timelapse_print) {
                timelapse_wall = only_generate_out_wall();
            }
            finish_layer_tcr = finish_layer(m_enable_timelapse_print ? false : true, layer.extruder_fill);
        }

        for (int i=0; i<int(layer.tool_changes.size()); ++i) {
            if (i == 0 && m_enable_timelapse_print) {
                timelapse_wall = only_generate_out_wall();
            }

            if (i == idx) {
                layer_result.emplace_back(tool_change(layer.tool_changes[i].new_tool, m_enable_timelapse_print ? false : true, false));
                // finish_layer will be called after this toolchange
                finish_layer_tcr = finish_layer(false, layer.extruder_fill);
            }
            else {
                if (idx == -1 && i == 0) {
                    layer_result.emplace_back(tool_change(layer.tool_changes[i].new_tool, false, true));
                } else {
                    layer_result.emplace_back(tool_change(layer.tool_changes[i].new_tool, false, false));
                }
            }
        }

        if (layer_result.empty()) {
            // there is nothing to merge finish_layer with
            layer_result.emplace_back(std::move(finish_layer_tcr));
        }
        else {
            if (idx == -1)
                layer_result[0] = merge_tcr(finish_layer_tcr, layer_result[0]);
            else if (is_valid_gcode(finish_layer_tcr.gcode))
                layer_result[idx] = merge_tcr(layer_result[idx], finish_layer_tcr);
        }

        if (m_enable_timelapse_print) {
            layer_result.insert(layer_result.begin(), std::move(timelapse_wall));
        }

		result.emplace_back(std::move(layer_result));
	}
}
#endif
WipeTower::ToolChangeResult WipeTower::only_generate_out_wall(bool is_new_mode)
{
    size_t old_tool = m_current_tool;

    WipeTowerWriter writer(m_layer_height, m_perimeter_width, m_gcode_flavor, m_filpar, m_enable_arc_fitting, m_travel_speed, m_lag_emit_block_z);
    writer.set_extrusion_flow(m_extrusion_flow)
        .set_z(m_z_pos)
        .set_initial_tool(m_current_tool)
        .set_y_shift(m_y_shift - (m_current_shape == SHAPE_REVERSED ? m_layer_info->toolchanges_depth() : 0.f));

    set_for_wipe_tower_writer(writer);

    // Slow down on the 1st layer.
    bool first_layer = is_first_layer();
    // BBS: speed up perimeter speed to 90mm/s for non-first layer
    float feedrate   = is_first_layer() ? std::min(first_layer_speed() * 60.f, m_max_speed) : std::min(60.0f * m_filpar[m_current_tool].max_e_speed / m_extrusion_flow, m_max_speed);
    float           fill_box_y = m_layer_info->toolchanges_depth() + m_perimeter_width;
    box_coordinates fill_box(Vec2f(m_perimeter_width, fill_box_y), m_wipe_tower_width - 2 * m_perimeter_width, m_layer_info->depth - fill_box_y);

    writer.set_initial_position((m_left_to_right ? fill_box.ru : fill_box.lu), // so there is never a diagonal travel
                                m_wipe_tower_width, m_wipe_tower_depth, m_internal_rotation);

    bool toolchanges_on_layer = m_layer_info->toolchanges_depth() > WT_EPSILON;

    // we are in one of the corners, travel to ld along the perimeter:
    // BBS: Delete some unnecessary travel
    //if (writer.x() > fill_box.ld.x() + EPSILON) writer.travel(fill_box.ld.x(), writer.y());
    //if (writer.y() > fill_box.ld.y() + EPSILON) writer.travel(writer.x(), fill_box.ld.y());
    writer.append_wipe_tower_start();
    // outer perimeter (always):
    // BBS

    float wipe_tower_depth = m_layer_info->depth + m_perimeter_width;
    if (is_new_mode && (m_enable_timelapse_print || m_enable_wrapping_detection))
        wipe_tower_depth = m_wipe_tower_depth;
    box_coordinates wt_box(Vec2f(0.f, (m_current_shape == SHAPE_REVERSED ? m_layer_info->toolchanges_depth() : 0.f)), m_wipe_tower_width, wipe_tower_depth);
    wt_box = align_perimeter(wt_box);
    Polygon outer_wall;
    //if (m_use_gap_wall)
    //    generate_support_wall(writer, wt_box, feedrate, first_layer);
    //else
    //    writer.rectangle(wt_box, feedrate);
    // Same continuous base wall on the timelapse/wrapping path.
    outer_wall = generate_support_wall_new(writer, wt_box, feedrate, first_layer, m_use_rib_wall, true,
                                           m_use_gap_wall && !is_tower_base_layer());
    m_outer_wall[m_z_pos].push_back(to_polyline(outer_wall));
    // Now prepare future wipe. box contains rectangle that was extruded last (ccw).

    // Vec2f target = (writer.pos() == wt_box.ld ? wt_box.rd : (writer.pos() == wt_box.rd ? wt_box.ru : (writer.pos() == wt_box.ru ? wt_box.lu : wt_box.ld)));
    //writer.add_wipe_point(writer.pos()).add_wipe_point(target);

    writer.add_wipe_path(outer_wall, m_filpar[m_current_tool].wipe_dist);
    writer.append(";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_End) + "\n");

    // Ask our writer about how much material was consumed.
    // Skip this in case the layer is sparse and config option to not print sparse layers is enabled.
    if (!m_no_sparse_layers || toolchanges_on_layer || m_layer_info->force_emit)
        if (m_current_tool < m_used_filament_length.size()) m_used_filament_length[m_current_tool] += writer.get_and_reset_used_filament_length();

    return construct_tcr(writer, false, old_tool, true, false, 0.f, false,
                         StructuralRole::TowerPerimeter);
}

Polygon WipeTower::generate_rib_polygon(const box_coordinates &wt_box)
{
    auto    get_current_layer_rib_len = [](float cur_height, float max_height, float max_len) -> float { return std::abs(max_height - cur_height) / max_height * max_len; };
    coord_t diagonal_width            = scaled(m_rib_width)/2;
    float   a = this->m_wipe_tower_width, b = this->m_wipe_tower_depth;
    Line    line_1(Point::new_scale(Vec2f{0, 0}), Point::new_scale(Vec2f{a, b}));
    Line    line_2(Point::new_scale(Vec2f{a, 0}), Point::new_scale(Vec2f{0, b}));
    float   diagonal_extra_length = std::max(0.f, m_rib_length - (float) unscaled(line_1.length())) / 2.f;
    diagonal_extra_length         = scaled(get_current_layer_rib_len(this->m_z_pos, this->m_wipe_tower_height, diagonal_extra_length));
    Point y_shift{0, scaled(this->m_y_shift)};

    line_1.extend(double(diagonal_extra_length));
    line_2.extend(double(diagonal_extra_length));
    line_1.translate(-y_shift);
    line_2.translate(-y_shift);

    Polygon poly_1 = generate_rectange(line_1, diagonal_width);
    Polygon poly_2 = generate_rectange(line_2, diagonal_width);
    Polygon poly;
    poly.points.push_back(Point::new_scale(wt_box.ld));
    poly.points.push_back(Point::new_scale(wt_box.rd));
    poly.points.push_back(Point::new_scale(wt_box.ru));
    poly.points.push_back(Point::new_scale(wt_box.lu));

    Polygons           p_1_2    = union_({poly_1, poly_2, poly});
    //Polygon            res_poly = p_1_2.front();
    //for (auto &p : res_poly.points) res.push_back(unscale(p).cast<float>());
    /*if (p_1_2.front().points.size() != 16)
        std::cout << "error " << std::endl;*/
    return p_1_2.front();
};

Polygon WipeTower::generate_support_wall_new(WipeTowerWriter &writer, const box_coordinates &wt_box, double feedrate, bool first_layer,bool rib_wall, bool extrude_perimeter, bool skip_points)
{
    auto get_closet_idx = [this, &writer](Polylines &pls) -> std::pair<int,int> {
        Vec2f anchor{writer.x(), writer.y()};
        int   closestIndex = -1;
        int   closestPl = -1;
        float minDistance  = std::numeric_limits<float>::max();
        for (int i = 0; i < pls.size(); ++i) {
            for (int j = 0; j < pls[i].size(); ++j) {
                float distance = (unscaled<float>(pls[i][j]) - anchor).squaredNorm();
                if (distance < minDistance) {
                    minDistance  = distance;
                    closestPl    = i;
                    closestIndex = j;
                }
            }
        }
        return {closestPl, closestIndex};
    };

    float retract_length = m_filpar[m_current_tool].retract_length;
    float retract_speed  = m_filpar[m_current_tool].retract_speed * 60;
    Polygon wall_polygon   = rib_wall ? generate_rib_polygon(wt_box) : generate_rectange_polygon(wt_box.ld, wt_box.ru);
    Polylines result_wall;
    Polygon   insert_skip_polygon;
    if (m_used_fillet) {
        if (!rib_wall && m_y_shift > EPSILON)// do nothing because the fillet will cause it to be suspended.
        {
        } else {
            wall_polygon           = rib_wall ? rounding_polygon(wall_polygon) : wall_polygon; // rectangle_wall do nothing
            Polygon wt_box_polygon = generate_rectange_polygon(wt_box.ld, wt_box.ru);
            wall_polygon           = union_({wall_polygon, wt_box_polygon}).front();
        }
    }
    if (!extrude_perimeter) return wall_polygon;

    if (skip_points) {
        result_wall = construct_gap_for_skip_points(wall_polygon, m_wall_skip_points[m_cur_layer_id], m_wipe_tower_width, 2.5 * m_perimeter_width, insert_skip_polygon);
    }
    else {
        result_wall.push_back(to_polyline(wall_polygon));
        insert_skip_polygon = wall_polygon;
    }
    writer.generate_path(result_wall, feedrate, retract_length, retract_speed,m_used_fillet);
    if (m_cur_layer_id == 0) {
        BoundingBox bbox = get_extents(result_wall);
        m_rib_offset     = Vec2f(-unscaled<float>(bbox.min.x()), -unscaled<float>(bbox.min.y()));
    }
    return insert_skip_polygon;
}

Polygon WipeTower::generate_support_wall(WipeTowerWriter &writer, const box_coordinates &wt_box, double feedrate, bool first_layer)
{
    float retract_length = m_filpar[m_current_tool].retract_length;
    float retract_speed  = m_filpar[m_current_tool].retract_speed *60 ;
    bool is_left  = false;
    bool is_right = false;
    for (auto pt : m_wall_skip_points[m_cur_layer_id]) {
        if (abs(pt.x()) < EPSILON) {
            is_left = true;
        } else if (abs(pt.x() - m_wipe_tower_width) < EPSILON) {
            is_right = true;
        }
    }

    if (is_left && is_right) {
        Vec2f *p = nullptr;
        p->x();
    }

    if (!is_left && !is_right) {
        Vec2f *p = nullptr;
        p->x();
    }

    //  3 -------------  2
    //    |           |
    //    |           |
    //  0 -------------  1

    int   index   = 0;
    Vec2f cur_pos = writer.pos();
    if (abs(cur_pos.x() - wt_box.ld.x()) > abs(cur_pos.x() - wt_box.rd.x())) {
        if (abs(cur_pos.y() - wt_box.ld.y()) > abs(cur_pos.y() - wt_box.lu.y())) {
            index = 2;
        } else {
            index = 1;
        }
    } else {
        if (abs(cur_pos.y() - wt_box.ld.y()) > abs(cur_pos.y() - wt_box.lu.y())) {
            index = 3;
        } else {
            index = 0;
        }
    }

    std::vector<Vec2f> points;
    points.emplace_back(wt_box.ld);
    points.emplace_back(wt_box.rd);
    points.emplace_back(wt_box.ru);
    points.emplace_back(wt_box.lu);

    writer.travel(points[index]);
    int extruded_nums = 0;
    while (extruded_nums < 4) {
        index = (index + 1) % 4;
        if (index == 2) {
            if (is_right) {
                std::vector<Segment> break_segments = remove_points_from_segment(Segment(wt_box.rd, wt_box.ru), m_wall_skip_points[m_cur_layer_id], 2.5 * m_perimeter_width);
                for (auto iter = break_segments.begin(); iter != break_segments.end(); ++iter) {
                    float dx  = iter->start.x() - writer.pos().x();
                    float dy  = iter->start.y() - writer.pos().y();
                    float len = std::sqrt(dx * dx + dy * dy);
                    if (len > 0) {
                        writer.retract(retract_length, retract_speed);
                        writer.travel(iter->start, 600.);
                        writer.retract(-retract_length, retract_speed);
                    } else
                        writer.travel(iter->start, 600.);

                    writer.extrude(iter->end, feedrate);
                }
                writer.travel(wt_box.ru, feedrate);
            } else {
                writer.extrude(wt_box.ru, feedrate);
            }
        } else if (index == 0) {
            if (is_left) {
                std::vector<Segment> break_segments = remove_points_from_segment(Segment(wt_box.ld, wt_box.lu), m_wall_skip_points[m_cur_layer_id], 2.5 * m_perimeter_width);
                for (auto iter = break_segments.rbegin(); iter != break_segments.rend(); ++iter) {
                    float dx  = iter->end.x() - writer.pos().x();
                    float dy  = iter->end.y() - writer.pos().y();
                    float len = std::sqrt(dx * dx + dy * dy);
                    if (len > 0) {
                        writer.retract(retract_length, retract_speed);
                        writer.travel(iter->end, 600.);
                        writer.retract(-retract_length, retract_speed);
                    } else
                        writer.travel(iter->end, 600.);

                    writer.extrude(iter->start, feedrate);
                }
                writer.travel(wt_box.ld, feedrate);
            } else {
                writer.extrude(wt_box.ld, feedrate);
            }
        } else {
            writer.extrude(points[index], feedrate);
        }
        extruded_nums++;
    }

    return Polygon();
}


bool WipeTower::get_floating_area(float &start_pos_y, float &end_pos_y) const {
    if (m_layer_info == m_plan.begin() || (m_layer_info - 1) == m_plan.begin())
        return false;

    if (!m_cur_block)
        return false;

    end_pos_y = m_cur_block->start_depth + m_cur_block->depth - m_perimeter_width;
    start_pos_y = m_cur_block->finish_depth[m_cur_layer_id - 1];

#if 0
    float last_layer_fill_box_y = (m_layer_info - 1)->toolchanges_depth() + m_perimeter_width;
    float last_layer_wipe_depth = (m_layer_info - 1)->depth;
    if (last_layer_wipe_depth - last_layer_fill_box_y <= 2 * m_perimeter_width)
        return false;

    start_pos_y = last_layer_fill_box_y + m_perimeter_width;
    end_pos_y   = last_layer_wipe_depth - m_perimeter_width;
#endif
    return true;
}

bool WipeTower::need_thick_bridge_flow(float pos_y) const {
    // The bridge road is 0.2 mm tall; a nozzle capped below that bridges at its own layer height.
    if (!tool_can_thicken(int(m_current_tool)))
        return false;
    if (m_layer_height >= 0.2)
        return false;

    float y_min = 0., y_max = 0.;
    if (get_floating_area(y_min, y_max)) {
        return pos_y > y_min && pos_y < y_max;
    }
    return false;
}

bool WipeTower::is_valid_last_layer(int tool, int layer_id, double layer_z) const
{
    int extruder_id = get_extruder_id(tool, layer_id);
    if (extruder_id < 0 || extruder_id >= m_printable_height.size()) return true;
    if (m_last_layer_id[extruder_id] == layer_id && layer_z > m_printable_height[extruder_id]) return false;
    return true;
}
float WipeTower::get_block_gap_width(int tool,bool is_nozzlechangle)
{
    //assert(m_block_infill_gap_width.count(m_filpar[tool].category));//The code contains logic that attempts to access non-existent blocks,
                                                                     // such as in case of involving two extruders with only a single head and a single layer,
                                                                     // some code will attempt to access the block's nozzle_change_gap_width, even though the block does not exist.
    if (!m_block_infill_gap_width.count(m_filpar[tool].category)) {
        return is_nozzlechangle ? m_nozzle_change_perimeter_width : m_perimeter_width;
    }
    return is_nozzlechangle ? m_block_infill_gap_width[m_filpar[tool].category].second : m_block_infill_gap_width[m_filpar[tool].category].first;

}

bool WipeTower::is_need_ramming(int filament_id_1, int filament_id_2, int layer_id) const
{
    return !m_multi_nozzle_group_result->are_filaments_same_nozzle(filament_id_1, filament_id_2, layer_id);
}
bool WipeTower::is_same_extruder(int filament_id_1, int filament_id_2, int layer_id) const
{
    return m_multi_nozzle_group_result->are_filaments_same_extruder(filament_id_1, filament_id_2, layer_id);
}

bool WipeTower::is_same_nozzle(int filament_id_1, int filament_id_2, int layer_id) const
{
    return m_multi_nozzle_group_result->are_filaments_same_nozzle(filament_id_1, filament_id_2, layer_id);
}

int WipeTower::get_nozzle_id(int filament_id, int layer_id) const { return m_multi_nozzle_group_result->get_nozzle_id(filament_id, layer_id); }

int WipeTower::get_extruder_id(int filament_id, int layer_id) const {
    return m_multi_nozzle_group_result->get_extruder_id(filament_id, layer_id);
}

} // namespace Slic3r
