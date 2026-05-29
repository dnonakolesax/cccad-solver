#include "cccad/solver/constraint_solver.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace cccad::solver {
namespace {

using cccad::solver::v1::CONSTRAINT_STATUS_ACTIVE;

constexpr double kDefaultTolerance = 1e-8;
constexpr double kDefaultStabilityWeight = 1e-12;
constexpr double kMinimumRadius = 1e-9;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;

bool IsActive(cccad::solver::v1::ConstraintStatus status) {
  return status == cccad::solver::v1::CONSTRAINT_STATUS_UNSPECIFIED ||
         status == CONSTRAINT_STATUS_ACTIVE;
}

bool IsFinite(double value) { return std::isfinite(value); }

bool IsReservedAxisEntityId(const std::string& entity_id) {
  return entity_id == "x-axis" || entity_id == "y-axis";
}

bool IsReservedAxisLine(const cccad::solver::v1::Entity& entity) {
  return entity.kind_case() == cccad::solver::v1::Entity::kLine &&
         IsReservedAxisEntityId(entity.id());
}

double Square(double value) { return value * value; }

double Distance(double ax, double ay, double bx, double by) {
  return std::sqrt(Square(bx - ax) + Square(by - ay));
}

double NormalizePositiveAngle(double value) {
  double normalized = std::fmod(value, kTwoPi);
  if (normalized < 0.0) {
    normalized += kTwoPi;
  }
  return normalized;
}

double NormalizeSignedAngle(double value) {
  double normalized = NormalizePositiveAngle(value);
  if (normalized > kPi) {
    normalized -= kTwoPi;
  }
  return normalized;
}

double ClampUnit(double value) { return std::max(-1.0, std::min(1.0, value)); }

struct LineGeometry {
  const SolverPoint* start = nullptr;
  const SolverPoint* end = nullptr;
  double dx = 0.0;
  double dy = 0.0;
  double length = 0.0;
};

std::optional<LineGeometry> GetLineGeometry(const SolverModel& model, const std::string& line_id) {
  const auto line_it = model.lines.find(line_id);
  if (line_it == model.lines.end()) {
    return std::nullopt;
  }
  const auto start_it = model.points.find(line_it->second.start_point_id);
  const auto end_it = model.points.find(line_it->second.end_point_id);
  if (start_it == model.points.end() || end_it == model.points.end()) {
    return std::nullopt;
  }
  LineGeometry geometry;
  geometry.start = &start_it->second;
  geometry.end = &end_it->second;
  geometry.dx = geometry.end->x - geometry.start->x;
  geometry.dy = geometry.end->y - geometry.start->y;
  geometry.length = std::sqrt(Square(geometry.dx) + Square(geometry.dy));
  return geometry;
}

std::optional<double> PointLineDistance(const SolverModel& model,
                                        const std::string& point_id,
                                        const std::string& line_id) {
  const auto point_it = model.points.find(point_id);
  const auto line = GetLineGeometry(model, line_id);
  if (point_it == model.points.end() || !line.has_value() || line->length < kMinimumRadius) {
    return std::nullopt;
  }
  const double cross = line->dx * (point_it->second.y - line->start->y) -
                       line->dy * (point_it->second.x - line->start->x);
  return std::abs(cross) / line->length;
}

std::optional<double> LineLineDistance(const LineGeometry& line_a,
                                       const LineGeometry& line_b) {
  if (line_a.length < kMinimumRadius || line_b.length < kMinimumRadius) {
    return std::nullopt;
  }
  const double cross = line_a.dx * (line_b.start->y - line_a.start->y) -
                       line_a.dy * (line_b.start->x - line_a.start->x);
  return std::abs(cross) / line_a.length;
}

std::optional<double> ParallelResidual(const LineGeometry& line_a,
                                       const LineGeometry& line_b) {
  const double scale = line_a.length * line_b.length;
  if (scale < kMinimumRadius) {
    return std::nullopt;
  }
  return (line_a.dx * line_b.dy - line_a.dy * line_b.dx) / scale;
}

std::optional<double> AngleDimensionResidual(
    const cccad::solver::v1::AngleDimension& angle_dimension,
    const SolverModel& model,
    double target) {
  const auto line_a = GetLineGeometry(model, angle_dimension.line_a_id());
  const auto line_b = GetLineGeometry(model, angle_dimension.line_b_id());
  if (!line_a.has_value() || !line_b.has_value()) {
    return std::nullopt;
  }
  const double scale = line_a->length * line_b->length;
  if (scale < kMinimumRadius) {
    return std::nullopt;
  }

  const double cross = line_a->dx * line_b->dy - line_a->dy * line_b->dx;
  const double dot = line_a->dx * line_b->dx + line_a->dy * line_b->dy;
  double measured = 0.0;
  switch (angle_dimension.orientation()) {
    case cccad::solver::v1::ANGLE_ORIENTATION_CW:
      measured = NormalizePositiveAngle(-std::atan2(cross, dot));
      break;
    case cccad::solver::v1::ANGLE_ORIENTATION_CCW:
      measured = NormalizePositiveAngle(std::atan2(cross, dot));
      break;
    case cccad::solver::v1::ANGLE_ORIENTATION_UNSPECIFIED:
      measured = std::acos(ClampUnit(dot / scale));
      break;
  }
  return NormalizeSignedAngle(measured - target);
}

double DimensionTarget(const cccad::solver::v1::Dimension& dimension,
                       const SolverModel& model,
                       double default_value) {
  if (const auto override_it = model.dimension_value_overrides.find(dimension.id());
      override_it != model.dimension_value_overrides.end()) {
    return override_it->second;
  }
  return default_value;
}

struct RoundGeometry {
  const SolverPoint* center = nullptr;
  double radius = 0.0;
};

std::optional<RoundGeometry> GetRoundGeometry(const SolverModel& model,
                                              const std::string& entity_id) {
  if (const auto circle_it = model.circles.find(entity_id); circle_it != model.circles.end()) {
    const auto center_it = model.points.find(circle_it->second.center_point_id);
    if (center_it == model.points.end()) {
      return std::nullopt;
    }
    return RoundGeometry{.center = &center_it->second, .radius = circle_it->second.radius};
  }

  if (const auto arc_it = model.arcs.find(entity_id); arc_it != model.arcs.end()) {
    const auto center_it = model.points.find(arc_it->second.center_point_id);
    const auto start_it = model.points.find(arc_it->second.start_point_id);
    if (center_it == model.points.end() || start_it == model.points.end()) {
      return std::nullopt;
    }
    return RoundGeometry{.center = &center_it->second,
                         .radius = Distance(center_it->second.x, center_it->second.y,
                                            start_it->second.x, start_it->second.y)};
  }

  return std::nullopt;
}

std::optional<double> LineRoundTangentResidual(const SolverModel& model,
                                               const std::string& line_id,
                                               const std::string& round_id) {
  const auto round = GetRoundGeometry(model, round_id);
  if (!round.has_value()) {
    return std::nullopt;
  }
  const auto line = GetLineGeometry(model, line_id);
  if (!line.has_value() || line->length < kMinimumRadius) {
    return std::nullopt;
  }
  const double cross = line->dx * (round->center->y - line->start->y) -
                       line->dy * (round->center->x - line->start->x);
  return std::abs(cross) / line->length - round->radius;
}

std::optional<double> RoundRoundTangentResidual(const SolverModel& model,
                                                const std::string& round_a_id,
                                                const std::string& round_b_id,
                                                cccad::solver::v1::TangentBranch branch) {
  const auto round_a = GetRoundGeometry(model, round_a_id);
  const auto round_b = GetRoundGeometry(model, round_b_id);
  if (!round_a.has_value() || !round_b.has_value()) {
    return std::nullopt;
  }
  const double center_distance = Distance(round_a->center->x, round_a->center->y,
                                          round_b->center->x, round_b->center->y);
  if (branch == cccad::solver::v1::TANGENT_BRANCH_INTERNAL) {
    return center_distance - std::abs(round_a->radius - round_b->radius);
  }
  return center_distance - (round_a->radius + round_b->radius);
}

std::optional<double> TangentResidual(const SolverModel& model,
                                      const cccad::solver::v1::TangentConstraint& tangent) {
  const bool a_is_line = model.lines.contains(tangent.entity_a_id());
  const bool b_is_line = model.lines.contains(tangent.entity_b_id());
  const bool a_is_round = GetRoundGeometry(model, tangent.entity_a_id()).has_value();
  const bool b_is_round = GetRoundGeometry(model, tangent.entity_b_id()).has_value();
  if (a_is_line && b_is_round) {
    return LineRoundTangentResidual(model, tangent.entity_a_id(), tangent.entity_b_id());
  }
  if (b_is_line && a_is_round) {
    return LineRoundTangentResidual(model, tangent.entity_b_id(), tangent.entity_a_id());
  }
  if (a_is_round && b_is_round) {
    return RoundRoundTangentResidual(model, tangent.entity_a_id(), tangent.entity_b_id(),
                                     tangent.branch());
  }
  return std::nullopt;
}

std::optional<double> ArcRadiusConsistencyResidual(const SolverModel& model,
                                                   const SolverArc& arc) {
  const auto center_it = model.points.find(arc.center_point_id);
  const auto start_it = model.points.find(arc.start_point_id);
  const auto end_it = model.points.find(arc.end_point_id);
  if (center_it == model.points.end() || start_it == model.points.end() ||
      end_it == model.points.end()) {
    return std::nullopt;
  }
  const double start_radius = Distance(center_it->second.x, center_it->second.y,
                                       start_it->second.x, start_it->second.y);
  const double end_radius = Distance(center_it->second.x, center_it->second.y,
                                     end_it->second.x, end_it->second.y);
  return end_radius - start_radius;
}

struct SolverVariable {
  enum class Kind { kPointX, kPointY, kCircleRadius };
  Kind kind = Kind::kPointX;
  std::string id;
};

bool ScopeContains(const std::unordered_set<std::string>* ids, const std::string& id) {
  return ids == nullptr || ids->contains(id);
}

std::unordered_set<std::string> MakeScopeSet(const std::vector<std::string>& ids) {
  return std::unordered_set<std::string>(ids.begin(), ids.end());
}

std::vector<SolverVariable> BuildVariables(const SolverModel& model,
                                           const std::unordered_set<std::string>* entity_scope = nullptr) {
  std::vector<SolverVariable> variables;
  std::vector<std::string> ids;

  ids.reserve(model.points.size());
  for (const auto& [id, point] : model.points) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  for (const auto& id : ids) {
    if (!ScopeContains(entity_scope, id)) {
      continue;
    }
    const auto& point = model.points.at(id);
    if (!point.lock_x) {
      variables.push_back({SolverVariable::Kind::kPointX, id});
    }
    if (!point.lock_y) {
      variables.push_back({SolverVariable::Kind::kPointY, id});
    }
  }

  ids.clear();
  ids.reserve(model.circles.size());
  for (const auto& [id, circle] : model.circles) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  for (const auto& id : ids) {
    if (!ScopeContains(entity_scope, id)) {
      continue;
    }
    const auto& circle = model.circles.at(id);
    if (!circle.lock_radius) {
      variables.push_back({SolverVariable::Kind::kCircleRadius, id});
    }
  }

  return variables;
}

std::vector<double> ReadVariables(const SolverModel& model, const std::vector<SolverVariable>& variables) {
  std::vector<double> x;
  x.reserve(variables.size());
  for (const auto& variable : variables) {
    switch (variable.kind) {
      case SolverVariable::Kind::kPointX:
        x.push_back(model.points.at(variable.id).x);
        break;
      case SolverVariable::Kind::kPointY:
        x.push_back(model.points.at(variable.id).y);
        break;
      case SolverVariable::Kind::kCircleRadius:
        x.push_back(model.circles.at(variable.id).radius);
        break;
    }
  }
  return x;
}

void WriteVariables(SolverModel* model, const std::vector<SolverVariable>& variables,
                    const std::vector<double>& x) {
  for (std::size_t index = 0; index < variables.size(); ++index) {
    const auto& variable = variables[index];
    switch (variable.kind) {
      case SolverVariable::Kind::kPointX:
        (*model).points[variable.id].x = x[index];
        break;
      case SolverVariable::Kind::kPointY:
        (*model).points[variable.id].y = x[index];
        break;
      case SolverVariable::Kind::kCircleRadius:
        (*model).circles[variable.id].radius = std::max(x[index], kMinimumRadius);
        break;
    }
  }
}

void AddScaledResidual(std::vector<double>* residuals, double value, double weight = 1.0) {
  const double safe_weight = std::isfinite(weight) && weight > 0.0 ? weight : 1.0;
  residuals->push_back(std::sqrt(safe_weight) * value);
}

struct ResidualDiagnosticSource {
  std::string code;
  std::string message;
  std::vector<std::string> entity_ids;
  std::vector<std::string> constraint_ids;
  std::vector<std::string> dimension_ids;
};

struct ResidualEntry {
  double value = 0.0;
  ResidualDiagnosticSource diagnostic;
};

void SortStrings(std::vector<std::string>* values) {
  std::sort(values->begin(), values->end());
  values->erase(std::unique(values->begin(), values->end()), values->end());
}

bool RepeatedStringsEqual(const google::protobuf::RepeatedPtrField<std::string>& lhs,
                          const google::protobuf::RepeatedPtrField<std::string>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (int index = 0; index < lhs.size(); ++index) {
    if (lhs.Get(index) != rhs.Get(index)) {
      return false;
    }
  }
  return true;
}

struct ProfilePoint {
  double x = 0.0;
  double y = 0.0;
};

struct ProfileLoopData {
  std::vector<std::string> entity_ids;
  std::vector<ProfilePoint> vertices;
  ProfilePoint center;
  double area = 0.0;
  double radius = 0.0;
  bool is_circle = false;
};

struct ProfileData {
  ProfileLoopData outer_loop;
  std::vector<ProfileLoopData> inner_loops;
  double area = 0.0;
};

double SignedPolygonArea(const std::vector<ProfilePoint>& vertices) {
  if (vertices.size() < 3) return 0.0;
  double sum = 0.0;
  for (std::size_t index = 0; index < vertices.size(); ++index) {
    const ProfilePoint& current = vertices[index];
    const ProfilePoint& next = vertices[(index + 1) % vertices.size()];
    sum += current.x * next.y - next.x * current.y;
  }
  return 0.5 * sum;
}

void RotateLoopToSmallestEntityId(ProfileLoopData* loop) {
  if (loop->entity_ids.empty()) return;
  const auto min_it = std::min_element(loop->entity_ids.begin(), loop->entity_ids.end());
  const std::size_t offset = static_cast<std::size_t>(min_it - loop->entity_ids.begin());
  std::rotate(loop->entity_ids.begin(), loop->entity_ids.begin() + offset, loop->entity_ids.end());
  if (!loop->vertices.empty() && loop->vertices.size() == loop->entity_ids.size()) {
    std::rotate(loop->vertices.begin(), loop->vertices.begin() + offset, loop->vertices.end());
  }
}

bool PointInPolygon(const ProfilePoint& point, const std::vector<ProfilePoint>& polygon) {
  if (polygon.size() < 3) return false;
  bool inside = false;
  for (std::size_t index = 0, previous = polygon.size() - 1; index < polygon.size();
       previous = index++) {
    const ProfilePoint& a = polygon[index];
    const ProfilePoint& b = polygon[previous];
    if ((a.y > point.y) != (b.y > point.y)) {
      const double x_intersection = (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x;
      if (point.x < x_intersection) inside = !inside;
    }
  }
  return inside;
}

ProfilePoint RepresentativePoint(const ProfileLoopData& loop) {
  if (loop.is_circle) return loop.center;
  return loop.vertices.empty() ? ProfilePoint{} : loop.vertices.front();
}

bool LoopContainsLoop(const ProfileLoopData& outer, const ProfileLoopData& inner) {
  if (outer.entity_ids == inner.entity_ids) return false;
  const ProfilePoint representative = RepresentativePoint(inner);
  if (outer.is_circle) {
    return Distance(outer.center.x, outer.center.y, representative.x, representative.y) <
           outer.radius - kDefaultTolerance;
  }
  return PointInPolygon(representative, outer.vertices);
}

std::vector<ProfileLoopData> BuildLineProfileLoops(const SolverModel& model) {
  struct LineEdge {
    std::string id;
    std::string start_point_id;
    std::string end_point_id;
  };

  std::vector<LineEdge> lines;
  lines.reserve(model.lines.size());
  for (const auto& [id, line] : model.lines) {
    if (line.start_point_id == line.end_point_id) continue;
    if (!model.points.contains(line.start_point_id) || !model.points.contains(line.end_point_id)) {
      continue;
    }
    lines.push_back({id, line.start_point_id, line.end_point_id});
  }
  std::sort(lines.begin(), lines.end(),
            [](const LineEdge& lhs, const LineEdge& rhs) { return lhs.id < rhs.id; });

  std::unordered_map<std::string, const LineEdge*> line_by_id;
  std::unordered_map<std::string, std::vector<std::string>> point_to_line_ids;
  for (const LineEdge& line : lines) {
    line_by_id[line.id] = &line;
    point_to_line_ids[line.start_point_id].push_back(line.id);
    point_to_line_ids[line.end_point_id].push_back(line.id);
  }
  for (auto& [unused, line_ids] : point_to_line_ids) {
    (void)unused;
    std::sort(line_ids.begin(), line_ids.end());
  }

  std::vector<ProfileLoopData> loops;
  std::unordered_set<std::string> visited_lines;
  for (const LineEdge& seed : lines) {
    if (visited_lines.contains(seed.id)) continue;

    std::vector<std::string> component_lines;
    std::vector<std::string> pending = {seed.id};
    std::unordered_set<std::string> component_line_set;
    std::unordered_set<std::string> component_points;
    while (!pending.empty()) {
      const std::string line_id = pending.back();
      pending.pop_back();
      if (!component_line_set.insert(line_id).second) continue;
      const LineEdge* line = line_by_id[line_id];
      component_lines.push_back(line_id);
      component_points.insert(line->start_point_id);
      component_points.insert(line->end_point_id);
      for (const std::string& point_id : {line->start_point_id, line->end_point_id}) {
        for (const std::string& next_line_id : point_to_line_ids[point_id]) {
          if (!component_line_set.contains(next_line_id)) pending.push_back(next_line_id);
        }
      }
    }
    for (const std::string& line_id : component_lines) visited_lines.insert(line_id);
    if (component_lines.size() < 3 || component_lines.size() != component_points.size()) continue;

    bool is_simple_cycle = true;
    for (const std::string& point_id : component_points) {
      if (point_to_line_ids[point_id].size() != 2) {
        is_simple_cycle = false;
        break;
      }
    }
    if (!is_simple_cycle) continue;

    std::sort(component_lines.begin(), component_lines.end());
    const LineEdge* start_line = line_by_id[component_lines.front()];
    const std::string start_point_id = std::min(start_line->start_point_id, start_line->end_point_id);
    std::string current_point_id = start_point_id;
    std::unordered_set<std::string> used_lines;
    ProfileLoopData loop;
    for (;;) {
      const auto& incident_lines = point_to_line_ids[current_point_id];
      const auto next_line_it = std::find_if(
          incident_lines.begin(), incident_lines.end(), [&](const std::string& line_id) {
            return component_line_set.contains(line_id) && !used_lines.contains(line_id);
          });
      if (next_line_it == incident_lines.end()) break;
      const LineEdge* line = line_by_id[*next_line_it];
      used_lines.insert(line->id);
      loop.entity_ids.push_back(line->id);
      const SolverPoint& point = model.points.at(current_point_id);
      loop.vertices.push_back({point.x, point.y});
      current_point_id = line->start_point_id == current_point_id ? line->end_point_id
                                                                  : line->start_point_id;
      if (current_point_id == start_point_id) break;
    }
    if (used_lines.size() != component_lines.size() || current_point_id != start_point_id) continue;

    const double signed_area = SignedPolygonArea(loop.vertices);
    loop.area = std::abs(signed_area);
    if (loop.area <= kDefaultTolerance) continue;
    if (signed_area < 0.0) {
      std::reverse(loop.entity_ids.begin(), loop.entity_ids.end());
      std::reverse(loop.vertices.begin(), loop.vertices.end());
    }
    RotateLoopToSmallestEntityId(&loop);
    loops.push_back(std::move(loop));
  }
  return loops;
}

std::vector<ProfileLoopData> BuildCircleProfileLoops(const SolverModel& model) {
  std::vector<ProfileLoopData> loops;
  for (const auto& [id, circle] : model.circles) {
    const auto center_it = model.points.find(circle.center_point_id);
    if (center_it == model.points.end() || circle.radius <= kMinimumRadius ||
        !IsFinite(circle.radius)) {
      continue;
    }
    ProfileLoopData loop;
    loop.entity_ids = {id};
    loop.center = {center_it->second.x, center_it->second.y};
    loop.radius = circle.radius;
    loop.area = kPi * circle.radius * circle.radius;
    loop.is_circle = true;
    loops.push_back(std::move(loop));
  }
  std::sort(loops.begin(), loops.end(), [](const ProfileLoopData& lhs, const ProfileLoopData& rhs) {
    return lhs.entity_ids.front() < rhs.entity_ids.front();
  });
  return loops;
}

std::vector<ProfileData> BuildProfiles(const SolverModel& model) {
  std::vector<ProfileLoopData> loops = BuildLineProfileLoops(model);
  std::vector<ProfileLoopData> circle_loops = BuildCircleProfileLoops(model);
  loops.insert(loops.end(), circle_loops.begin(), circle_loops.end());
  std::sort(loops.begin(), loops.end(), [](const ProfileLoopData& lhs, const ProfileLoopData& rhs) {
    if (lhs.area != rhs.area) return lhs.area > rhs.area;
    return lhs.entity_ids.front() < rhs.entity_ids.front();
  });

  std::vector<int> parent(loops.size(), -1);
  for (std::size_t index = 0; index < loops.size(); ++index) {
    for (std::size_t candidate = 0; candidate < loops.size(); ++candidate) {
      if (candidate == index || loops[candidate].area <= loops[index].area) continue;
      if (!LoopContainsLoop(loops[candidate], loops[index])) continue;
      if (parent[index] < 0 ||
          loops[candidate].area < loops[static_cast<std::size_t>(parent[index])].area) {
        parent[index] = static_cast<int>(candidate);
      }
    }
  }

  std::vector<ProfileData> profiles;
  std::unordered_map<std::size_t, std::size_t> profile_by_outer_loop;
  for (std::size_t index = 0; index < loops.size(); ++index) {
    if (parent[index] >= 0) continue;
    ProfileData profile;
    profile.outer_loop = loops[index];
    profile.area = loops[index].area;
    profile_by_outer_loop[index] = profiles.size();
    profiles.push_back(std::move(profile));
  }
  for (std::size_t index = 0; index < loops.size(); ++index) {
    if (parent[index] < 0 || parent[static_cast<std::size_t>(parent[index])] >= 0) continue;
    const std::size_t outer_index = static_cast<std::size_t>(parent[index]);
    auto profile_it = profile_by_outer_loop.find(outer_index);
    if (profile_it == profile_by_outer_loop.end()) continue;
    ProfileData& profile = profiles[profile_it->second];
    profile.area -= loops[index].area;
    profile.inner_loops.push_back(loops[index]);
  }

  for (ProfileData& profile : profiles) {
    std::sort(profile.inner_loops.begin(), profile.inner_loops.end(),
              [](const ProfileLoopData& lhs, const ProfileLoopData& rhs) {
                return lhs.entity_ids.front() < rhs.entity_ids.front();
              });
  }
  std::sort(profiles.begin(), profiles.end(), [](const ProfileData& lhs, const ProfileData& rhs) {
    return lhs.outer_loop.entity_ids.front() < rhs.outer_loop.entity_ids.front();
  });
  return profiles;
}

void AddResidualEntry(std::vector<ResidualEntry>* residuals,
                      double value,
                      ResidualDiagnosticSource diagnostic,
                      double weight = 1.0) {
  const double safe_weight = std::isfinite(weight) && weight > 0.0 ? weight : 1.0;
  residuals->push_back({.value = std::sqrt(safe_weight) * value,
                        .diagnostic = std::move(diagnostic)});
}

void AddArcResiduals(const SolverModel& model,
                     std::vector<double>* residuals,
                     const std::unordered_set<std::string>* entity_scope = nullptr) {
  for (const auto& [arc_id, arc] : model.arcs) {
    if (!ScopeContains(entity_scope, arc_id)) {
      continue;
    }
    const auto residual = ArcRadiusConsistencyResidual(model, arc);
    if (residual.has_value()) {
      AddScaledResidual(residuals, *residual);
    }
  }
}

void AddDiagnosticArcResiduals(const SolverModel& model,
                               std::vector<ResidualEntry>* residuals,
                               const std::unordered_set<std::string>* entity_scope = nullptr) {
  for (const auto& [arc_id, arc] : model.arcs) {
    if (!ScopeContains(entity_scope, arc_id)) {
      continue;
    }
    const auto residual = ArcRadiusConsistencyResidual(model, arc);
    if (!residual.has_value()) {
      continue;
    }
    AddResidualEntry(residuals, *residual,
                     {.code = "entity_residual",
                      .message = "arc radius consistency residual is above tolerance",
                      .entity_ids = {arc_id, arc.center_point_id, arc.start_point_id,
                                     arc.end_point_id}});
  }
}

void AddDiagnosticResiduals(const cccad::solver::v1::Constraint& constraint,
                            const SolverModel& model,
                            std::vector<ResidualEntry>* residuals) {
  if (!IsActive(constraint.status())) {
    return;
  }

  switch (constraint.kind_case()) {
    case cccad::solver::v1::Constraint::kCoincident: {
      const auto point_a_it = model.points.find(constraint.coincident().point_a_id());
      const auto point_b_it = model.points.find(constraint.coincident().point_b_id());
      if (point_a_it == model.points.end() || point_b_it == model.points.end()) {
        return;
      }
      const ResidualDiagnosticSource diagnostic{
          .code = "constraint_residual",
          .message = "coincident constraint residual is above tolerance",
          .entity_ids = {constraint.coincident().point_a_id(), constraint.coincident().point_b_id()},
          .constraint_ids = {constraint.id()}};
      AddResidualEntry(residuals, point_a_it->second.x - point_b_it->second.x, diagnostic);
      AddResidualEntry(residuals, point_a_it->second.y - point_b_it->second.y, diagnostic);
      break;
    }
    case cccad::solver::v1::Constraint::kHorizontal: {
      const auto line_it = model.lines.find(constraint.horizontal().line_id());
      if (line_it == model.lines.end()) {
        return;
      }
      const auto start_it = model.points.find(line_it->second.start_point_id);
      const auto end_it = model.points.find(line_it->second.end_point_id);
      if (start_it == model.points.end() || end_it == model.points.end()) {
        return;
      }
      AddResidualEntry(residuals, end_it->second.y - start_it->second.y,
                       {.code = "constraint_residual",
                        .message = "horizontal constraint residual is above tolerance",
                        .entity_ids = {constraint.horizontal().line_id()},
                        .constraint_ids = {constraint.id()}});
      break;
    }
    case cccad::solver::v1::Constraint::kVertical: {
      const auto line_it = model.lines.find(constraint.vertical().line_id());
      if (line_it == model.lines.end()) {
        return;
      }
      const auto start_it = model.points.find(line_it->second.start_point_id);
      const auto end_it = model.points.find(line_it->second.end_point_id);
      if (start_it == model.points.end() || end_it == model.points.end()) {
        return;
      }
      AddResidualEntry(residuals, end_it->second.x - start_it->second.x,
                       {.code = "constraint_residual",
                        .message = "vertical constraint residual is above tolerance",
                        .entity_ids = {constraint.vertical().line_id()},
                        .constraint_ids = {constraint.id()}});
      break;
    }
    case cccad::solver::v1::Constraint::kParallel:
    case cccad::solver::v1::Constraint::kPerpendicular: {
      const auto line_a = GetLineGeometry(
          model, constraint.kind_case() == cccad::solver::v1::Constraint::kParallel
                     ? constraint.parallel().line_a_id()
                     : constraint.perpendicular().line_a_id());
      const auto line_b = GetLineGeometry(
          model, constraint.kind_case() == cccad::solver::v1::Constraint::kParallel
                     ? constraint.parallel().line_b_id()
                     : constraint.perpendicular().line_b_id());
      if (!line_a.has_value() || !line_b.has_value()) {
        return;
      }
      const double scale = line_a->length * line_b->length;
      if (scale < kMinimumRadius) {
        return;
      }
      const bool parallel = constraint.kind_case() == cccad::solver::v1::Constraint::kParallel;
      AddResidualEntry(residuals,
                       parallel ? (line_a->dx * line_b->dy - line_a->dy * line_b->dx) / scale
                                : (line_a->dx * line_b->dx + line_a->dy * line_b->dy) / scale,
                       {.code = "constraint_residual",
                        .message = parallel
                                       ? "parallel constraint residual is above tolerance"
                                       : "perpendicular constraint residual is above tolerance",
                        .entity_ids = {parallel ? constraint.parallel().line_a_id()
                                                : constraint.perpendicular().line_a_id(),
                                       parallel ? constraint.parallel().line_b_id()
                                                : constraint.perpendicular().line_b_id()},
                        .constraint_ids = {constraint.id()}});
      break;
    }
    case cccad::solver::v1::Constraint::kEqual: {
      if (constraint.equal().kind() == cccad::solver::v1::EQUAL_KIND_LENGTH) {
        const auto line_a = GetLineGeometry(model, constraint.equal().entity_a_id());
        const auto line_b = GetLineGeometry(model, constraint.equal().entity_b_id());
        if (!line_a.has_value() || !line_b.has_value()) {
          return;
        }
        AddResidualEntry(residuals, line_a->length - line_b->length,
                         {.code = "constraint_residual",
                          .message = "equal length constraint residual is above tolerance",
                          .entity_ids = {constraint.equal().entity_a_id(),
                                         constraint.equal().entity_b_id()},
                          .constraint_ids = {constraint.id()}});
      } else if (constraint.equal().kind() == cccad::solver::v1::EQUAL_KIND_RADIUS) {
        const auto round_a = GetRoundGeometry(model, constraint.equal().entity_a_id());
        const auto round_b = GetRoundGeometry(model, constraint.equal().entity_b_id());
        if (!round_a.has_value() || !round_b.has_value()) {
          return;
        }
        AddResidualEntry(residuals, round_a->radius - round_b->radius,
                         {.code = "constraint_residual",
                          .message = "equal radius constraint residual is above tolerance",
                          .entity_ids = {constraint.equal().entity_a_id(),
                                         constraint.equal().entity_b_id()},
                          .constraint_ids = {constraint.id()}});
      }
      break;
    }
    case cccad::solver::v1::Constraint::kMidpoint: {
      const auto midpoint_it = model.points.find(constraint.midpoint().midpoint_id());
      const auto point_a_it = model.points.find(constraint.midpoint().point_a_id());
      const auto point_b_it = model.points.find(constraint.midpoint().point_b_id());
      if (midpoint_it == model.points.end() || point_a_it == model.points.end() ||
          point_b_it == model.points.end()) {
        return;
      }
      const ResidualDiagnosticSource diagnostic{
          .code = "constraint_residual",
          .message = "midpoint constraint residual is above tolerance",
          .entity_ids = {constraint.midpoint().midpoint_id(), constraint.midpoint().point_a_id(),
                         constraint.midpoint().point_b_id()},
          .constraint_ids = {constraint.id()}};
      AddResidualEntry(residuals,
                       midpoint_it->second.x - (point_a_it->second.x + point_b_it->second.x) * 0.5,
                       diagnostic);
      AddResidualEntry(residuals,
                       midpoint_it->second.y - (point_a_it->second.y + point_b_it->second.y) * 0.5,
                       diagnostic);
      break;
    }
    case cccad::solver::v1::Constraint::kConcentric: {
      const auto round_a = GetRoundGeometry(model, constraint.concentric().circle_a_id());
      const auto round_b = GetRoundGeometry(model, constraint.concentric().circle_b_id());
      if (!round_a.has_value() || !round_b.has_value()) {
        return;
      }
      const ResidualDiagnosticSource diagnostic{
          .code = "constraint_residual",
          .message = "concentric constraint residual is above tolerance",
          .entity_ids = {constraint.concentric().circle_a_id(),
                         constraint.concentric().circle_b_id()},
          .constraint_ids = {constraint.id()}};
      AddResidualEntry(residuals, round_a->center->x - round_b->center->x, diagnostic);
      AddResidualEntry(residuals, round_a->center->y - round_b->center->y, diagnostic);
      break;
    }
    case cccad::solver::v1::Constraint::kTangent: {
      const auto residual = TangentResidual(model, constraint.tangent());
      if (!residual.has_value()) {
        return;
      }
      AddResidualEntry(residuals, *residual,
                       {.code = "constraint_residual",
                        .message = "tangent constraint residual is above tolerance",
                        .entity_ids = {constraint.tangent().entity_a_id(),
                                       constraint.tangent().entity_b_id()},
                        .constraint_ids = {constraint.id()}});
      break;
    }
    case cccad::solver::v1::Constraint::kFixed:
    case cccad::solver::v1::Constraint::KIND_NOT_SET:
      break;
  }
}

void AddDiagnosticDimensionResiduals(const cccad::solver::v1::Dimension& dimension,
                                     const SolverModel& model,
                                     std::vector<ResidualEntry>* residuals) {
  if (!IsActive(dimension.status()) || !dimension.driving()) {
    return;
  }

  switch (dimension.kind_case()) {
    case cccad::solver::v1::Dimension::kDistance: {
      const double target = DimensionTarget(dimension, model, dimension.distance().value());
      std::optional<double> measured;
      if (dimension.distance().ref_kind() == cccad::solver::v1::DISTANCE_REFERENCE_KIND_UNSPECIFIED ||
          dimension.distance().ref_kind() == cccad::solver::v1::DISTANCE_REFERENCE_KIND_POINT_POINT) {
        const auto point_a_it = model.points.find(dimension.distance().ref_a_id());
        const auto point_b_it = model.points.find(dimension.distance().ref_b_id());
        if (point_a_it == model.points.end() || point_b_it == model.points.end()) {
          return;
        }
        measured = Distance(point_a_it->second.x, point_a_it->second.y,
                            point_b_it->second.x, point_b_it->second.y);
      } else if (dimension.distance().ref_kind() ==
                 cccad::solver::v1::DISTANCE_REFERENCE_KIND_POINT_LINE) {
        measured = PointLineDistance(model, dimension.distance().ref_a_id(),
                                     dimension.distance().ref_b_id());
      } else if (dimension.distance().ref_kind() ==
                 cccad::solver::v1::DISTANCE_REFERENCE_KIND_LINE_LINE) {
        const auto line_a = GetLineGeometry(model, dimension.distance().ref_a_id());
        const auto line_b = GetLineGeometry(model, dimension.distance().ref_b_id());
        if (!line_a.has_value() || !line_b.has_value()) {
          return;
        }
        measured = LineLineDistance(*line_a, *line_b);
        if (const auto parallel = ParallelResidual(*line_a, *line_b); parallel.has_value()) {
          AddResidualEntry(residuals, *parallel,
                           {.code = "dimension_residual",
                            .message = "line-line distance dimension parallel residual is above tolerance",
                            .entity_ids = {dimension.distance().ref_a_id(),
                                           dimension.distance().ref_b_id()},
                            .dimension_ids = {dimension.id()}});
        }
      }
      if (!measured.has_value()) {
        return;
      }
      AddResidualEntry(residuals,
                       *measured - target,
                       {.code = "dimension_residual",
                        .message = "distance dimension residual is above tolerance",
                        .entity_ids = {dimension.distance().ref_a_id(), dimension.distance().ref_b_id()},
                        .dimension_ids = {dimension.id()}});
      break;
    }
    case cccad::solver::v1::Dimension::kRadius: {
      const auto round = GetRoundGeometry(model, dimension.radius().entity_id());
      if (!round.has_value()) {
        return;
      }
      const double target = DimensionTarget(dimension, model, dimension.radius().value());
      AddResidualEntry(residuals, round->radius - target,
                       {.code = "dimension_residual",
                        .message = "radius dimension residual is above tolerance",
                        .entity_ids = {dimension.radius().entity_id()},
                        .dimension_ids = {dimension.id()}});
      break;
    }
    case cccad::solver::v1::Dimension::kDiameter: {
      const auto round = GetRoundGeometry(model, dimension.diameter().entity_id());
      if (!round.has_value()) {
        return;
      }
      const double target = DimensionTarget(dimension, model, dimension.diameter().value()) * 0.5;
      AddResidualEntry(residuals, round->radius - target,
                       {.code = "dimension_residual",
                        .message = "diameter dimension residual is above tolerance",
                        .entity_ids = {dimension.diameter().entity_id()},
                        .dimension_ids = {dimension.id()}});
      break;
    }
    case cccad::solver::v1::Dimension::kAngle: {
      const double target = DimensionTarget(dimension, model, dimension.angle().value_rad());
      const auto residual = AngleDimensionResidual(dimension.angle(), model, target);
      if (!residual.has_value()) {
        return;
      }
      AddResidualEntry(residuals, *residual,
                       {.code = "dimension_residual",
                        .message = "angle dimension residual is above tolerance",
                        .entity_ids = {dimension.angle().line_a_id(), dimension.angle().line_b_id()},
                        .dimension_ids = {dimension.id()}});
      break;
    }
    case cccad::solver::v1::Dimension::KIND_NOT_SET:
      break;
  }
}

void AddConstraintResiduals(const cccad::solver::v1::Constraint& constraint,
                            const SolverModel& model, std::vector<double>* residuals) {
  if (!IsActive(constraint.status())) {
    return;
  }

  switch (constraint.kind_case()) {
    case cccad::solver::v1::Constraint::kCoincident: {
      const auto point_a_it = model.points.find(constraint.coincident().point_a_id());
      const auto point_b_it = model.points.find(constraint.coincident().point_b_id());
      if (point_a_it == model.points.end() || point_b_it == model.points.end()) {
        return;
      }
      AddScaledResidual(residuals, point_a_it->second.x - point_b_it->second.x);
      AddScaledResidual(residuals, point_a_it->second.y - point_b_it->second.y);
      break;
    }
    case cccad::solver::v1::Constraint::kHorizontal: {
      const auto line_it = model.lines.find(constraint.horizontal().line_id());
      if (line_it == model.lines.end()) {
        return;
      }
      const auto start_it = model.points.find(line_it->second.start_point_id);
      const auto end_it = model.points.find(line_it->second.end_point_id);
      if (start_it == model.points.end() || end_it == model.points.end()) {
        return;
      }
      AddScaledResidual(residuals, end_it->second.y - start_it->second.y);
      break;
    }
    case cccad::solver::v1::Constraint::kVertical: {
      const auto line_it = model.lines.find(constraint.vertical().line_id());
      if (line_it == model.lines.end()) {
        return;
      }
      const auto start_it = model.points.find(line_it->second.start_point_id);
      const auto end_it = model.points.find(line_it->second.end_point_id);
      if (start_it == model.points.end() || end_it == model.points.end()) {
        return;
      }
      AddScaledResidual(residuals, end_it->second.x - start_it->second.x);
      break;
    }
    case cccad::solver::v1::Constraint::kParallel:
    case cccad::solver::v1::Constraint::kPerpendicular: {
      const auto line_a = GetLineGeometry(
          model, constraint.kind_case() == cccad::solver::v1::Constraint::kParallel
                     ? constraint.parallel().line_a_id()
                     : constraint.perpendicular().line_a_id());
      const auto line_b = GetLineGeometry(
          model, constraint.kind_case() == cccad::solver::v1::Constraint::kParallel
                     ? constraint.parallel().line_b_id()
                     : constraint.perpendicular().line_b_id());
      if (!line_a.has_value() || !line_b.has_value()) {
        return;
      }
      const double scale = line_a->length * line_b->length;
      if (scale < kMinimumRadius) {
        return;
      }
      if (constraint.kind_case() == cccad::solver::v1::Constraint::kParallel) {
        AddScaledResidual(residuals,
                          (line_a->dx * line_b->dy - line_a->dy * line_b->dx) / scale);
      } else {
        AddScaledResidual(residuals,
                          (line_a->dx * line_b->dx + line_a->dy * line_b->dy) / scale);
      }
      break;
    }
    case cccad::solver::v1::Constraint::kEqual: {
      if (constraint.equal().kind() == cccad::solver::v1::EQUAL_KIND_LENGTH) {
        const auto line_a = GetLineGeometry(model, constraint.equal().entity_a_id());
        const auto line_b = GetLineGeometry(model, constraint.equal().entity_b_id());
        if (!line_a.has_value() || !line_b.has_value()) {
          return;
        }
        AddScaledResidual(residuals, line_a->length - line_b->length);
      } else if (constraint.equal().kind() == cccad::solver::v1::EQUAL_KIND_RADIUS) {
        const auto round_a = GetRoundGeometry(model, constraint.equal().entity_a_id());
        const auto round_b = GetRoundGeometry(model, constraint.equal().entity_b_id());
        if (!round_a.has_value() || !round_b.has_value()) {
          return;
        }
        AddScaledResidual(residuals, round_a->radius - round_b->radius);
      }
      break;
    }
    case cccad::solver::v1::Constraint::kMidpoint: {
      const auto midpoint_it = model.points.find(constraint.midpoint().midpoint_id());
      const auto point_a_it = model.points.find(constraint.midpoint().point_a_id());
      const auto point_b_it = model.points.find(constraint.midpoint().point_b_id());
      if (midpoint_it == model.points.end() || point_a_it == model.points.end() ||
          point_b_it == model.points.end()) {
        return;
      }
      AddScaledResidual(residuals,
                        midpoint_it->second.x - (point_a_it->second.x + point_b_it->second.x) * 0.5);
      AddScaledResidual(residuals,
                        midpoint_it->second.y - (point_a_it->second.y + point_b_it->second.y) * 0.5);
      break;
    }
    case cccad::solver::v1::Constraint::kConcentric: {
      const auto round_a = GetRoundGeometry(model, constraint.concentric().circle_a_id());
      const auto round_b = GetRoundGeometry(model, constraint.concentric().circle_b_id());
      if (!round_a.has_value() || !round_b.has_value()) {
        return;
      }
      AddScaledResidual(residuals, round_a->center->x - round_b->center->x);
      AddScaledResidual(residuals, round_a->center->y - round_b->center->y);
      break;
    }
    case cccad::solver::v1::Constraint::kTangent: {
      const auto residual = TangentResidual(model, constraint.tangent());
      if (residual.has_value()) {
        AddScaledResidual(residuals, *residual);
      }
      break;
    }
    case cccad::solver::v1::Constraint::kFixed:
    case cccad::solver::v1::Constraint::KIND_NOT_SET:
      break;
  }
}

void AddDimensionResiduals(const cccad::solver::v1::Dimension& dimension,
                           const SolverModel& model, std::vector<double>* residuals) {
  if (!IsActive(dimension.status()) || !dimension.driving()) {
    return;
  }

  switch (dimension.kind_case()) {
    case cccad::solver::v1::Dimension::kDistance: {
      const double target = DimensionTarget(dimension, model, dimension.distance().value());
      if (dimension.distance().ref_kind() == cccad::solver::v1::DISTANCE_REFERENCE_KIND_UNSPECIFIED ||
          dimension.distance().ref_kind() == cccad::solver::v1::DISTANCE_REFERENCE_KIND_POINT_POINT) {
        const auto point_a_it = model.points.find(dimension.distance().ref_a_id());
        const auto point_b_it = model.points.find(dimension.distance().ref_b_id());
        if (point_a_it == model.points.end() || point_b_it == model.points.end()) {
          return;
        }
        AddScaledResidual(residuals, Distance(point_a_it->second.x, point_a_it->second.y,
                                              point_b_it->second.x, point_b_it->second.y) -
                                          target);
      } else if (dimension.distance().ref_kind() ==
                 cccad::solver::v1::DISTANCE_REFERENCE_KIND_POINT_LINE) {
        const auto measured = PointLineDistance(model, dimension.distance().ref_a_id(),
                                                dimension.distance().ref_b_id());
        if (measured.has_value()) {
          AddScaledResidual(residuals, *measured - target);
        }
      } else if (dimension.distance().ref_kind() ==
                 cccad::solver::v1::DISTANCE_REFERENCE_KIND_LINE_LINE) {
        const auto line_a = GetLineGeometry(model, dimension.distance().ref_a_id());
        const auto line_b = GetLineGeometry(model, dimension.distance().ref_b_id());
        if (!line_a.has_value() || !line_b.has_value()) {
          return;
        }
        if (const auto parallel = ParallelResidual(*line_a, *line_b); parallel.has_value()) {
          AddScaledResidual(residuals, *parallel);
        }
        if (const auto measured = LineLineDistance(*line_a, *line_b); measured.has_value()) {
          AddScaledResidual(residuals, *measured - target);
        }
      }
      break;
    }
    case cccad::solver::v1::Dimension::kRadius: {
      const auto round = GetRoundGeometry(model, dimension.radius().entity_id());
      if (!round.has_value()) {
        return;
      }
      const double target = DimensionTarget(dimension, model, dimension.radius().value());
      AddScaledResidual(residuals, round->radius - target);
      break;
    }
    case cccad::solver::v1::Dimension::kDiameter: {
      const auto round = GetRoundGeometry(model, dimension.diameter().entity_id());
      if (!round.has_value()) {
        return;
      }
      const double target = DimensionTarget(dimension, model, dimension.diameter().value()) * 0.5;
      AddScaledResidual(residuals, round->radius - target);
      break;
    }
    case cccad::solver::v1::Dimension::kAngle: {
      const double target = DimensionTarget(dimension, model, dimension.angle().value_rad());
      const auto residual = AngleDimensionResidual(dimension.angle(), model, target);
      if (residual.has_value()) {
        AddScaledResidual(residuals, *residual);
      }
      break;
    }
    case cccad::solver::v1::Dimension::KIND_NOT_SET:
      break;
  }
}

std::vector<double> EvaluateResiduals(const cccad::solver::v1::SketchModel& proto_model,
                                      const SolverModel& model,
                                      const cccad::solver::v1::SolverOptions& options,
                                      bool include_soft_residuals = true,
                                      const std::unordered_set<std::string>* constraint_scope = nullptr,
                                      const std::unordered_set<std::string>* dimension_scope = nullptr,
                                      const std::unordered_set<std::string>* entity_scope = nullptr) {
  std::vector<double> residuals;

  AddArcResiduals(model, &residuals, entity_scope);

  for (const auto& constraint : proto_model.constraints()) {
    if (!ScopeContains(constraint_scope, constraint.id())) {
      continue;
    }
    AddConstraintResiduals(constraint, model, &residuals);
  }
  for (const auto* constraint : model.extra_constraints) {
    AddConstraintResiduals(*constraint, model, &residuals);
  }
  for (const auto& dimension : proto_model.dimensions()) {
    if (!ScopeContains(dimension_scope, dimension.id())) {
      continue;
    }
    AddDimensionResiduals(dimension, model, &residuals);
  }

  if (!include_soft_residuals) {
    return residuals;
  }

  const double stability_weight = options.stability_weight() > 0.0 && IsFinite(options.stability_weight())
                                      ? options.stability_weight()
                                      : kDefaultStabilityWeight;
  for (const auto& [id, point] : model.points) {
    if (!point.lock_x) {
      AddScaledResidual(&residuals, point.x - point.x0, stability_weight);
    }
    if (!point.lock_y) {
      AddScaledResidual(&residuals, point.y - point.y0, stability_weight);
    }
  }
  for (const auto& [id, circle] : model.circles) {
    if (!circle.lock_radius) {
      AddScaledResidual(&residuals, circle.radius - circle.radius0, stability_weight);
    }
  }

  const double default_intent_weight =
      options.soft_intent_weight() > 0.0 && IsFinite(options.soft_intent_weight())
          ? options.soft_intent_weight()
          : 100.0;
  for (const auto& anchor : model.intent_anchors) {
    const auto point_it = model.points.find(anchor.point_id);
    if (point_it == model.points.end()) {
      continue;
    }
    const double weight = anchor.weight > 0.0 && IsFinite(anchor.weight) ? anchor.weight : default_intent_weight;
    AddScaledResidual(&residuals, point_it->second.x - anchor.x, weight);
    AddScaledResidual(&residuals, point_it->second.y - anchor.y, weight);
  }

  return residuals;
}

std::vector<ResidualEntry> EvaluateDiagnosticResiduals(
    const cccad::solver::v1::SketchModel& proto_model,
    const SolverModel& model,
    const cccad::solver::v1::SolverOptions& options,
    const std::unordered_set<std::string>* constraint_scope = nullptr,
    const std::unordered_set<std::string>* dimension_scope = nullptr,
    const std::unordered_set<std::string>* entity_scope = nullptr) {
  (void)options;
  std::vector<ResidualEntry> residuals;
  AddDiagnosticArcResiduals(model, &residuals, entity_scope);
  for (const auto& constraint : proto_model.constraints()) {
    if (!ScopeContains(constraint_scope, constraint.id())) {
      continue;
    }
    AddDiagnosticResiduals(constraint, model, &residuals);
  }
  for (const auto* constraint : model.extra_constraints) {
    AddDiagnosticResiduals(*constraint, model, &residuals);
  }
  for (const auto& dimension : proto_model.dimensions()) {
    if (!ScopeContains(dimension_scope, dimension.id())) {
      continue;
    }
    AddDiagnosticDimensionResiduals(dimension, model, &residuals);
  }
  return residuals;
}

double SquaredNorm(const std::vector<double>& values) {
  double result = 0.0;
  for (const double value : values) {
    result += value * value;
  }
  return result;
}

bool SolveLinearSystem(std::vector<std::vector<double>> a, std::vector<double> b,
                       std::vector<double>* x) {
  const std::size_t n = b.size();
  x->assign(n, 0.0);
  if (n == 0) {
    return true;
  }

  for (std::size_t col = 0; col < n; ++col) {
    std::size_t pivot = col;
    double best = std::abs(a[col][col]);
    for (std::size_t row = col + 1; row < n; ++row) {
      const double candidate = std::abs(a[row][col]);
      if (candidate > best) {
        best = candidate;
        pivot = row;
      }
    }
    if (best < 1e-18 || !std::isfinite(best)) {
      return false;
    }
    if (pivot != col) {
      std::swap(a[pivot], a[col]);
      std::swap(b[pivot], b[col]);
    }

    const double diagonal = a[col][col];
    for (std::size_t j = col; j < n; ++j) {
      a[col][j] /= diagonal;
    }
    b[col] /= diagonal;

    for (std::size_t row = 0; row < n; ++row) {
      if (row == col) {
        continue;
      }
      const double factor = a[row][col];
      if (factor == 0.0) {
        continue;
      }
      for (std::size_t j = col; j < n; ++j) {
        a[row][j] -= factor * a[col][j];
      }
      b[row] -= factor * b[col];
    }
  }

  *x = std::move(b);
  return true;
}

std::vector<std::vector<double>> BuildJacobian(
    const cccad::solver::v1::SketchModel& proto_model,
    const SolverModel& model,
    const cccad::solver::v1::SolverOptions& options,
    const std::vector<SolverVariable>& variables,
    const std::vector<double>& x,
    const std::vector<double>& residuals,
    const std::unordered_set<std::string>* constraint_scope = nullptr,
    const std::unordered_set<std::string>* dimension_scope = nullptr,
    const std::unordered_set<std::string>* entity_scope = nullptr) {
  const std::size_t rows = residuals.size();
  const std::size_t cols = variables.size();
  std::vector<std::vector<double>> jacobian(rows, std::vector<double>(cols, 0.0));

  for (std::size_t col = 0; col < cols; ++col) {
    std::vector<double> x_eps = x;
    const double h = 1e-6 * (1.0 + std::abs(x[col]));
    x_eps[col] += h;

    SolverModel perturbed = model;
    WriteVariables(&perturbed, variables, x_eps);
    const std::vector<double> residuals_eps =
        EvaluateResiduals(proto_model, perturbed, options, false, constraint_scope,
                          dimension_scope, entity_scope);

    for (std::size_t row = 0; row < rows; ++row) {
      jacobian[row][col] = (residuals_eps[row] - residuals[row]) / h;
    }
  }

  return jacobian;
}

int32_t MatrixRank(std::vector<std::vector<double>> matrix, double tolerance) {
  if (matrix.empty() || matrix.front().empty()) {
    return 0;
  }
  const std::size_t rows = matrix.size();
  const std::size_t cols = matrix.front().size();
  std::size_t rank = 0;

  for (std::size_t col = 0; col < cols && rank < rows; ++col) {
    std::size_t pivot = rank;
    double best = std::abs(matrix[pivot][col]);
    for (std::size_t row = rank + 1; row < rows; ++row) {
      const double candidate = std::abs(matrix[row][col]);
      if (candidate > best) {
        best = candidate;
        pivot = row;
      }
    }
    if (best <= tolerance || !std::isfinite(best)) {
      continue;
    }

    if (pivot != rank) {
      std::swap(matrix[pivot], matrix[rank]);
    }

    const double diagonal = matrix[rank][col];
    for (std::size_t j = col; j < cols; ++j) {
      matrix[rank][j] /= diagonal;
    }

    for (std::size_t row = 0; row < rows; ++row) {
      if (row == rank) {
        continue;
      }
      const double factor = matrix[row][col];
      if (factor == 0.0) {
        continue;
      }
      for (std::size_t j = col; j < cols; ++j) {
        matrix[row][j] -= factor * matrix[rank][j];
      }
    }
    ++rank;
  }

  return static_cast<int32_t>(rank);
}

int32_t EstimateRankDegreesOfFreedom(
    const cccad::solver::v1::SketchModel& proto_model,
    const SolverModel& model,
    const cccad::solver::v1::SolverOptions& options,
    const std::unordered_set<std::string>* entity_scope,
    const std::unordered_set<std::string>* constraint_scope,
    const std::unordered_set<std::string>* dimension_scope,
    int32_t* rank_out = nullptr) {
  SolverModel scratch = model;
  const std::vector<SolverVariable> variables = BuildVariables(scratch, entity_scope);
  if (variables.empty()) {
    if (rank_out != nullptr) {
      *rank_out = 0;
    }
    return 0;
  }

  const std::vector<double> x = ReadVariables(scratch, variables);
  WriteVariables(&scratch, variables, x);
  const std::vector<double> residuals =
      EvaluateResiduals(proto_model, scratch, options, false, constraint_scope,
                        dimension_scope, entity_scope);
  const double rank_tolerance =
      options.tolerance() > 0.0 && IsFinite(options.tolerance()) ? options.tolerance() * 10.0
                                                                 : 1e-7;
  const int32_t rank = residuals.empty()
                           ? 0
                           : MatrixRank(BuildJacobian(proto_model, scratch, options, variables, x,
                                                      residuals, constraint_scope,
                                                      dimension_scope, entity_scope),
                                        rank_tolerance);
  if (rank_out != nullptr) {
    *rank_out = rank;
  }
  return static_cast<int32_t>(variables.size()) - rank;
}

}  // namespace

SolverModel BuildSolverModel(const cccad::solver::v1::SketchModel& model) {
  SolverModel result;

  for (const auto& entity : model.entities()) {
    if (IsReservedAxisLine(entity)) {
      continue;
    }
    result.entity_kinds.emplace(entity.id(), entity.kind_case());
    switch (entity.kind_case()) {
      case cccad::solver::v1::Entity::kPoint: {
        SolverPoint point;
        point.id = entity.id();
        point.x = entity.point().x();
        point.y = entity.point().y();
        point.x0 = point.x;
        point.y0 = point.y;
        point.lock_x = entity.point().fixed();
        point.lock_y = entity.point().fixed();
        result.points.emplace(point.id, std::move(point));
        break;
      }
      case cccad::solver::v1::Entity::kLine: {
        SolverLine line;
        line.id = entity.id();
        line.start_point_id = entity.line().start_point_id();
        line.end_point_id = entity.line().end_point_id();
        result.lines.emplace(line.id, std::move(line));
        break;
      }
      case cccad::solver::v1::Entity::kCircle: {
        SolverCircle circle;
        circle.id = entity.id();
        circle.center_point_id = entity.circle().center_point_id();
        circle.radius = std::max(entity.circle().radius(), kMinimumRadius);
        circle.radius0 = circle.radius;
        result.circles.emplace(circle.id, std::move(circle));
        break;
      }
      case cccad::solver::v1::Entity::kArc: {
        SolverArc arc;
        arc.id = entity.id();
        arc.center_point_id = entity.arc().center_point_id();
        arc.start_point_id = entity.arc().start_point_id();
        arc.end_point_id = entity.arc().end_point_id();
        result.arcs.emplace(arc.id, std::move(arc));
        break;
      }
      case cccad::solver::v1::Entity::KIND_NOT_SET:
        break;
    }
  }

  auto lock_point = [&](const std::string& point_id) {
    auto point_it = result.points.find(point_id);
    if (point_it != result.points.end()) {
      point_it->second.lock_x = true;
      point_it->second.lock_y = true;
    }
  };

  auto lock_entity = [&](const std::string& entity_id) {
    const auto kind_it = result.entity_kinds.find(entity_id);
    if (kind_it == result.entity_kinds.end()) {
      return;
    }
    switch (kind_it->second) {
      case cccad::solver::v1::Entity::kPoint:
        lock_point(entity_id);
        break;
      case cccad::solver::v1::Entity::kLine: {
        const auto line_it = result.lines.find(entity_id);
        if (line_it != result.lines.end()) {
          lock_point(line_it->second.start_point_id);
          lock_point(line_it->second.end_point_id);
        }
        break;
      }
      case cccad::solver::v1::Entity::kCircle: {
        const auto circle_it = result.circles.find(entity_id);
        if (circle_it != result.circles.end()) {
          lock_point(circle_it->second.center_point_id);
          circle_it->second.lock_radius = true;
        }
        break;
      }
      case cccad::solver::v1::Entity::kArc: {
        const auto arc_it = result.arcs.find(entity_id);
        if (arc_it != result.arcs.end()) {
          lock_point(arc_it->second.center_point_id);
          lock_point(arc_it->second.start_point_id);
          lock_point(arc_it->second.end_point_id);
        }
        break;
      }
      case cccad::solver::v1::Entity::KIND_NOT_SET:
        break;
    }
  };

  for (const auto& constraint : model.constraints()) {
    if (IsActive(constraint.status()) && constraint.kind_case() == cccad::solver::v1::Constraint::kFixed) {
      lock_entity(constraint.fixed().entity_id());
    }
  }

  return result;
}

SolverResult SolveModelWithScopes(
    const cccad::solver::v1::SketchModel& proto_model,
    SolverModel initial_model,
    const cccad::solver::v1::SolverOptions& options,
    int32_t default_max_iterations,
    const std::unordered_set<std::string>* entity_scope,
    const std::unordered_set<std::string>* constraint_scope,
    const std::unordered_set<std::string>* dimension_scope) {
  SolverResult result;
  result.model = std::move(initial_model);
  const int32_t max_iterations = options.max_iterations() > 0 ? options.max_iterations()
                                                              : default_max_iterations;
  const double tolerance = options.tolerance() > 0.0 && IsFinite(options.tolerance())
                               ? options.tolerance()
                               : kDefaultTolerance;
  auto finalize = [&]() {
    const std::vector<double> hard_residuals =
        EvaluateResiduals(proto_model, result.model, options, false, constraint_scope,
                          dimension_scope, entity_scope);
    const double hard_residual_norm = std::sqrt(SquaredNorm(hard_residuals));
    result.converged = hard_residual_norm <= std::max(1e-5, tolerance * 10.0);
    result.degrees_of_freedom =
        EstimateRankDegreesOfFreedom(proto_model, result.model, options, entity_scope,
                                     constraint_scope, dimension_scope, &result.jacobian_rank);
    const double diagnostic_threshold = std::max(1e-5, tolerance * 10.0);
    result.residual_diagnostics.clear();
    for (auto& residual : EvaluateDiagnosticResiduals(proto_model, result.model, options,
                                                      constraint_scope, dimension_scope,
                                                      entity_scope)) {
      if (std::abs(residual.value) <= diagnostic_threshold) {
        continue;
      }
      SortStrings(&residual.diagnostic.entity_ids);
      SortStrings(&residual.diagnostic.constraint_ids);
      SortStrings(&residual.diagnostic.dimension_ids);
      auto* diagnostic = &result.residual_diagnostics.emplace_back();
      diagnostic->set_level(cccad::solver::v1::SOLVER_DIAGNOSTIC_LEVEL_ERROR);
      diagnostic->set_code(residual.diagnostic.code);
      diagnostic->set_message(residual.diagnostic.message);
      for (const auto& entity_id : residual.diagnostic.entity_ids) {
        diagnostic->add_entity_ids(entity_id);
      }
      for (const auto& constraint_id : residual.diagnostic.constraint_ids) {
        diagnostic->add_constraint_ids(constraint_id);
      }
      for (const auto& dimension_id : residual.diagnostic.dimension_ids) {
        diagnostic->add_dimension_ids(dimension_id);
      }
    }
    std::sort(result.residual_diagnostics.begin(), result.residual_diagnostics.end(),
              [](const auto& lhs, const auto& rhs) {
                if (lhs.code() != rhs.code()) return lhs.code() < rhs.code();
                if (lhs.message() != rhs.message()) return lhs.message() < rhs.message();
                if (!RepeatedStringsEqual(lhs.entity_ids(), rhs.entity_ids())) {
                  return std::lexicographical_compare(lhs.entity_ids().begin(), lhs.entity_ids().end(),
                                                      rhs.entity_ids().begin(), rhs.entity_ids().end());
                }
                if (!RepeatedStringsEqual(lhs.constraint_ids(), rhs.constraint_ids())) {
                  return std::lexicographical_compare(lhs.constraint_ids().begin(), lhs.constraint_ids().end(),
                                                      rhs.constraint_ids().begin(), rhs.constraint_ids().end());
                }
                return std::lexicographical_compare(lhs.dimension_ids().begin(), lhs.dimension_ids().end(),
                                                    rhs.dimension_ids().begin(), rhs.dimension_ids().end());
              });
  };

  const std::vector<SolverVariable> variables = BuildVariables(result.model, entity_scope);
  if (variables.empty()) {
    const auto residuals = EvaluateResiduals(proto_model, result.model, options, true,
                                            constraint_scope, dimension_scope, entity_scope);
    result.residual_norm = std::sqrt(SquaredNorm(residuals));
    result.converged = result.residual_norm <= std::max(1e-7, tolerance * 10.0);
    finalize();
    return result;
  }

  std::vector<double> x = ReadVariables(result.model, variables);
  double lambda = 1e-3;

  for (int32_t iteration = 0; iteration < max_iterations; ++iteration) {
    WriteVariables(&result.model, variables, x);
    const std::vector<double> residuals =
        EvaluateResiduals(proto_model, result.model, options, true, constraint_scope,
                          dimension_scope, entity_scope);
    const double current_cost = SquaredNorm(residuals);
    result.residual_norm = std::sqrt(current_cost);
    result.iterations = iteration + 1;
    if (result.residual_norm <= tolerance) {
      result.converged = true;
      finalize();
      return result;
    }

    const std::size_t m = residuals.size();
    const std::size_t n = variables.size();
    std::vector<std::vector<double>> jt_j(n, std::vector<double>(n, 0.0));
    std::vector<double> jt_r(n, 0.0);

    for (std::size_t col = 0; col < n; ++col) {
      std::vector<double> x_eps = x;
      const double h = 1e-6 * (1.0 + std::abs(x[col]));
      x_eps[col] += h;
      SolverModel perturbed = result.model;
      WriteVariables(&perturbed, variables, x_eps);
      const std::vector<double> residuals_eps =
          EvaluateResiduals(proto_model, perturbed, options, true, constraint_scope,
                            dimension_scope, entity_scope);

      std::vector<double> j_col(m, 0.0);
      for (std::size_t row = 0; row < m; ++row) {
        j_col[row] = (residuals_eps[row] - residuals[row]) / h;
        jt_r[col] += j_col[row] * residuals[row];
      }

      for (std::size_t other = 0; other <= col; ++other) {
        std::vector<double> x_eps_other = x;
        const double h_other = 1e-6 * (1.0 + std::abs(x[other]));
        x_eps_other[other] += h_other;
        SolverModel perturbed_other = result.model;
        WriteVariables(&perturbed_other, variables, x_eps_other);
        const std::vector<double> residuals_eps_other =
            EvaluateResiduals(proto_model, perturbed_other, options, true, constraint_scope,
                              dimension_scope, entity_scope);

        double value = 0.0;
        for (std::size_t row = 0; row < m; ++row) {
          const double j_other = (residuals_eps_other[row] - residuals[row]) / h_other;
          value += j_col[row] * j_other;
        }
        jt_j[col][other] = value;
        jt_j[other][col] = value;
      }
    }

    for (std::size_t i = 0; i < n; ++i) {
      jt_j[i][i] += lambda;
    }

    std::vector<double> rhs(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
      rhs[i] = -jt_r[i];
    }

    std::vector<double> delta;
    if (!SolveLinearSystem(jt_j, rhs, &delta)) {
      lambda *= 10.0;
      continue;
    }

    std::vector<double> candidate = x;
    double step_norm = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      candidate[i] += delta[i];
      step_norm += delta[i] * delta[i];
    }

    SolverModel candidate_model = result.model;
    WriteVariables(&candidate_model, variables, candidate);
    const double candidate_cost =
        SquaredNorm(EvaluateResiduals(proto_model, candidate_model, options, true,
                                      constraint_scope, dimension_scope, entity_scope));
    if (candidate_cost <= current_cost) {
      x = std::move(candidate);
      result.model = std::move(candidate_model);
      lambda = std::max(lambda * 0.3, 1e-12);
      if (std::sqrt(step_norm) <= tolerance * (1.0 + std::sqrt(SquaredNorm(x)))) {
        result.residual_norm = std::sqrt(candidate_cost);
        result.converged = result.residual_norm <= std::max(1e-7, tolerance * 10.0);
        finalize();
        return result;
      }
    } else {
      lambda = std::min(lambda * 10.0, 1e12);
    }
  }

  WriteVariables(&result.model, variables, x);
  result.residual_norm =
      std::sqrt(SquaredNorm(EvaluateResiduals(proto_model, result.model, options, true,
                                             constraint_scope, dimension_scope, entity_scope)));
  result.converged = result.residual_norm <= std::max(1e-7, tolerance * 10.0);
  finalize();
  return result;
}

SolverResult SolveModel(const cccad::solver::v1::SketchModel& proto_model,
                        SolverModel initial_model,
                        const cccad::solver::v1::SolverOptions& options,
                        int32_t default_max_iterations) {
  return SolveModelWithScopes(proto_model, std::move(initial_model), options,
                              default_max_iterations, nullptr, nullptr, nullptr);
}

SolverResult SolveModelScoped(const cccad::solver::v1::SketchModel& proto_model,
                              SolverModel initial_model,
                              const cccad::solver::v1::SolverOptions& options,
                              int32_t default_max_iterations,
                              const std::vector<std::string>& entity_ids,
                              const std::vector<std::string>& constraint_ids,
                              const std::vector<std::string>& dimension_ids) {
  const std::unordered_set<std::string> entity_scope = MakeScopeSet(entity_ids);
  const std::unordered_set<std::string> constraint_scope = MakeScopeSet(constraint_ids);
  const std::unordered_set<std::string> dimension_scope = MakeScopeSet(dimension_ids);
  return SolveModelWithScopes(proto_model, std::move(initial_model), options,
                              default_max_iterations, &entity_scope, &constraint_scope,
                              &dimension_scope);
}

int32_t EstimateSolverDegreesOfFreedom(
    const cccad::solver::v1::SketchModel& proto_model,
    const SolverModel& model,
    const cccad::solver::v1::SolverOptions& options,
    const std::vector<std::string>& entity_ids,
    const std::vector<std::string>& constraint_ids,
    const std::vector<std::string>& dimension_ids) {
  const std::unordered_set<std::string> entity_scope = MakeScopeSet(entity_ids);
  const std::unordered_set<std::string> constraint_scope = MakeScopeSet(constraint_ids);
  const std::unordered_set<std::string> dimension_scope = MakeScopeSet(dimension_ids);
  return EstimateRankDegreesOfFreedom(proto_model, model, options,
                                      entity_ids.empty() ? nullptr : &entity_scope,
                                      constraint_ids.empty() ? nullptr : &constraint_scope,
                                      dimension_ids.empty() ? nullptr : &dimension_scope);
}

std::vector<cccad::solver::v1::SolverDiagnostic> BuildResidualDiagnostics(
    const cccad::solver::v1::SketchModel& proto_model,
    const SolverModel& model,
    const cccad::solver::v1::SolverOptions& options) {
  const double tolerance =
      options.tolerance() > 0.0 && IsFinite(options.tolerance()) ? options.tolerance() : kDefaultTolerance;
  const double diagnostic_threshold = std::max(1e-5, tolerance * 10.0);
  std::vector<cccad::solver::v1::SolverDiagnostic> diagnostics;

  for (auto& residual : EvaluateDiagnosticResiduals(proto_model, model, options)) {
    if (std::abs(residual.value) <= diagnostic_threshold) {
      continue;
    }
    auto diagnostic = cccad::solver::v1::SolverDiagnostic{};
    diagnostic.set_level(cccad::solver::v1::SOLVER_DIAGNOSTIC_LEVEL_ERROR);
    diagnostic.set_code(residual.diagnostic.code);
    diagnostic.set_message(residual.diagnostic.message);
    SortStrings(&residual.diagnostic.entity_ids);
    SortStrings(&residual.diagnostic.constraint_ids);
    SortStrings(&residual.diagnostic.dimension_ids);
    for (const auto& id : residual.diagnostic.entity_ids) {
      diagnostic.add_entity_ids(id);
    }
    for (const auto& id : residual.diagnostic.constraint_ids) {
      diagnostic.add_constraint_ids(id);
    }
    for (const auto& id : residual.diagnostic.dimension_ids) {
      diagnostic.add_dimension_ids(id);
    }
    diagnostics.push_back(std::move(diagnostic));
  }

  std::sort(diagnostics.begin(), diagnostics.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.code() != rhs.code()) return lhs.code() < rhs.code();
    if (lhs.constraint_ids_size() != rhs.constraint_ids_size()) {
      return lhs.constraint_ids_size() < rhs.constraint_ids_size();
    }
    if (lhs.constraint_ids_size() > 0 && lhs.constraint_ids(0) != rhs.constraint_ids(0)) {
      return lhs.constraint_ids(0) < rhs.constraint_ids(0);
    }
    if (lhs.dimension_ids_size() > 0 && rhs.dimension_ids_size() > 0 &&
        lhs.dimension_ids(0) != rhs.dimension_ids(0)) {
      return lhs.dimension_ids(0) < rhs.dimension_ids(0);
    }
    return lhs.message() < rhs.message();
  });
  diagnostics.erase(std::unique(diagnostics.begin(), diagnostics.end(),
                                [](const auto& lhs, const auto& rhs) {
                                  return lhs.code() == rhs.code() &&
                                         lhs.message() == rhs.message() &&
                                         RepeatedStringsEqual(lhs.entity_ids(), rhs.entity_ids()) &&
                                         RepeatedStringsEqual(lhs.constraint_ids(), rhs.constraint_ids()) &&
                                         RepeatedStringsEqual(lhs.dimension_ids(), rhs.dimension_ids());
                                }),
                    diagnostics.end());
  return diagnostics;
}

void WriteSolverSolution(const cccad::solver::v1::SketchModel& proto_model,
                         const SolverModel& model,
                         cccad::solver::v1::SketchSolution* solution) {
  solution->Clear();
  std::vector<const cccad::solver::v1::Entity*> entities;
  entities.reserve(static_cast<std::size_t>(proto_model.entities_size()));
  for (const auto& entity : proto_model.entities()) {
    if (IsReservedAxisLine(entity)) {
      continue;
    }
    entities.push_back(&entity);
  }
  std::sort(entities.begin(), entities.end(), [](const auto* lhs, const auto* rhs) {
    return lhs->id() < rhs->id();
  });

  for (const auto* entity : entities) {
    auto* solved = solution->add_entities();
    solved->set_id(entity->id());
    switch (entity->kind_case()) {
      case cccad::solver::v1::Entity::kPoint: {
        const auto point_it = model.points.find(entity->id());
        solved->mutable_point()->set_x(point_it != model.points.end() ? point_it->second.x : entity->point().x());
        solved->mutable_point()->set_y(point_it != model.points.end() ? point_it->second.y : entity->point().y());
        break;
      }
      case cccad::solver::v1::Entity::kLine:
        solved->mutable_line()->set_start_point_id(entity->line().start_point_id());
        solved->mutable_line()->set_end_point_id(entity->line().end_point_id());
        break;
      case cccad::solver::v1::Entity::kCircle: {
        const auto circle_it = model.circles.find(entity->id());
        solved->mutable_circle()->set_center_point_id(entity->circle().center_point_id());
        solved->mutable_circle()->set_radius(circle_it != model.circles.end()
                                                 ? circle_it->second.radius
                                                 : entity->circle().radius());
        break;
      }
      case cccad::solver::v1::Entity::kArc:
        solved->mutable_arc()->set_center_point_id(entity->arc().center_point_id());
        solved->mutable_arc()->set_start_point_id(entity->arc().start_point_id());
        solved->mutable_arc()->set_end_point_id(entity->arc().end_point_id());
        solved->mutable_arc()->set_clockwise(entity->arc().clockwise());
        solved->mutable_arc()->set_branch(entity->arc().branch());
        break;
      case cccad::solver::v1::Entity::KIND_NOT_SET:
        break;
    }
  }

  std::vector<ProfileData> profiles = BuildProfiles(model);
  for (std::size_t index = 0; index < profiles.size(); ++index) {
    const ProfileData& profile_data = profiles[index];
    auto* profile = solution->add_profiles();
    profile->set_id("profile_" + std::to_string(index + 1));
    for (const std::string& entity_id : profile_data.outer_loop.entity_ids) {
      profile->mutable_outer_loop()->add_entity_ids(entity_id);
    }
    for (const ProfileLoopData& inner_loop_data : profile_data.inner_loops) {
      auto* inner_loop = profile->add_inner_loops();
      for (const std::string& entity_id : inner_loop_data.entity_ids) {
        inner_loop->add_entity_ids(entity_id);
      }
    }
    profile->set_area(profile_data.area);
    profile->set_valid_for_extrude(profile_data.area > kDefaultTolerance &&
                                   !profile_data.outer_loop.entity_ids.empty());
  }
}

}  // namespace cccad::solver
