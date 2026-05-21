#include "cccad/solver/constraint_graph.h"

#include <algorithm>
#include <deque>
#include <string>
#include <utility>

namespace cccad::solver {
namespace {

bool IsActive(cccad::solver::v1::ConstraintStatus status) {
  return status == cccad::solver::v1::CONSTRAINT_STATUS_UNSPECIFIED ||
         status == cccad::solver::v1::CONSTRAINT_STATUS_ACTIVE;
}

template <typename Map>
void AddEdge(Map* map, const std::string& key, const std::string& value) {
  if (key.empty() || value.empty()) {
    return;
  }
  (*map)[key].push_back(value);
}

std::vector<std::string> PointIdsForEntity(const cccad::solver::v1::Entity& entity) {
  switch (entity.kind_case()) {
    case cccad::solver::v1::Entity::kPoint:
      return {entity.id()};
    case cccad::solver::v1::Entity::kLine:
      return {entity.line().start_point_id(), entity.line().end_point_id()};
    case cccad::solver::v1::Entity::kCircle:
      return {entity.circle().center_point_id()};
    case cccad::solver::v1::Entity::kArc:
      return {entity.arc().center_point_id(), entity.arc().start_point_id(), entity.arc().end_point_id()};
    case cccad::solver::v1::Entity::KIND_NOT_SET:
      return {};
  }
  return {};
}

}  // namespace

void SortUnique(std::vector<std::string>* values) {
  std::sort(values->begin(), values->end());
  values->erase(std::unique(values->begin(), values->end()), values->end());
}

ConstraintGraph::ConstraintGraph(const cccad::solver::v1::SketchModel& model) : model_(model) {
  for (const auto& entity : model_.entities()) {
    entities_[entity.id()] = &entity;
  }
  for (const auto& constraint : model_.constraints()) {
    constraints_[constraint.id()] = &constraint;
  }
  for (const auto& dimension : model_.dimensions()) {
    dimensions_[dimension.id()] = &dimension;
  }

  for (const auto& entity : model_.entities()) {
    auto point_ids = PointIdsForEntity(entity);
    SortUnique(&point_ids);
    entity_to_points_[entity.id()] = point_ids;
    for (const auto& point_id : point_ids) {
      AddEdge(&point_to_entities_, point_id, entity.id());
    }
  }

  for (const auto& constraint : model_.constraints()) {
    if (!IsActive(constraint.status())) {
      continue;
    }
    auto entity_ids = ConstraintEntityIds(constraint);
    SortUnique(&entity_ids);
    constraint_to_entities_[constraint.id()] = entity_ids;
    for (const auto& entity_id : entity_ids) {
      AddEdge(&entity_to_constraints_, entity_id, constraint.id());
    }
  }

  for (const auto& dimension : model_.dimensions()) {
    if (!IsActive(dimension.status())) {
      continue;
    }
    auto entity_ids = DimensionEntityIds(dimension);
    SortUnique(&entity_ids);
    dimension_to_entities_[dimension.id()] = entity_ids;
    for (const auto& entity_id : entity_ids) {
      AddEdge(&entity_to_dimensions_, entity_id, dimension.id());
    }
  }

  for (auto* map : {&point_to_entities_, &entity_to_constraints_, &constraint_to_entities_,
                    &entity_to_dimensions_, &dimension_to_entities_}) {
    for (auto& [_, values] : *map) {
      SortUnique(&values);
    }
  }
}

std::vector<std::string> ConstraintGraph::EntityPointIds(const std::string& entity_id) const {
  if (const auto it = entity_to_points_.find(entity_id); it != entity_to_points_.end()) {
    return it->second;
  }
  return {};
}

std::vector<std::string> ConstraintGraph::ConstraintEntityIds(
    const cccad::solver::v1::Constraint& constraint) const {
  switch (constraint.kind_case()) {
    case cccad::solver::v1::Constraint::kCoincident:
      return {constraint.coincident().point_a_id(), constraint.coincident().point_b_id()};
    case cccad::solver::v1::Constraint::kHorizontal:
      return {constraint.horizontal().line_id()};
    case cccad::solver::v1::Constraint::kVertical:
      return {constraint.vertical().line_id()};
    case cccad::solver::v1::Constraint::kParallel:
      return {constraint.parallel().line_a_id(), constraint.parallel().line_b_id()};
    case cccad::solver::v1::Constraint::kPerpendicular:
      return {constraint.perpendicular().line_a_id(), constraint.perpendicular().line_b_id()};
    case cccad::solver::v1::Constraint::kTangent:
      return {constraint.tangent().entity_a_id(), constraint.tangent().entity_b_id()};
    case cccad::solver::v1::Constraint::kEqual:
      return {constraint.equal().entity_a_id(), constraint.equal().entity_b_id()};
    case cccad::solver::v1::Constraint::kFixed:
      return {constraint.fixed().entity_id()};
    case cccad::solver::v1::Constraint::kMidpoint:
      return {constraint.midpoint().midpoint_id(), constraint.midpoint().point_a_id(),
              constraint.midpoint().point_b_id()};
    case cccad::solver::v1::Constraint::kConcentric:
      return {constraint.concentric().circle_a_id(), constraint.concentric().circle_b_id()};
    case cccad::solver::v1::Constraint::KIND_NOT_SET:
      return {};
  }
  return {};
}

std::vector<std::string> ConstraintGraph::DimensionEntityIds(
    const cccad::solver::v1::Dimension& dimension) const {
  switch (dimension.kind_case()) {
    case cccad::solver::v1::Dimension::kDistance:
      return {dimension.distance().ref_a_id(), dimension.distance().ref_b_id()};
    case cccad::solver::v1::Dimension::kRadius:
      return {dimension.radius().entity_id()};
    case cccad::solver::v1::Dimension::kDiameter:
      return {dimension.diameter().entity_id()};
    case cccad::solver::v1::Dimension::kAngle:
      return {dimension.angle().line_a_id(), dimension.angle().line_b_id()};
    case cccad::solver::v1::Dimension::KIND_NOT_SET:
      return {};
  }
  return {};
}

ConstraintComponentData ConstraintGraph::TraverseFromEntities(
    const std::vector<std::string>& entity_ids,
    const std::vector<std::string>& constraint_ids,
    const std::vector<std::string>& dimension_ids) const {
  std::unordered_set<std::string> visited_entities;
  std::unordered_set<std::string> visited_constraints;
  std::unordered_set<std::string> visited_dimensions;
  std::unordered_set<std::string> visited_points;
  std::deque<std::string> entity_queue;
  std::deque<std::string> constraint_queue;
  std::deque<std::string> dimension_queue;

  auto add_entity = [&](const std::string& id) {
    if (!id.empty() && entities_.contains(id) && visited_entities.insert(id).second) {
      entity_queue.push_back(id);
    }
  };
  auto add_constraint = [&](const std::string& id) {
    if (!id.empty() && constraints_.contains(id) && visited_constraints.insert(id).second) {
      constraint_queue.push_back(id);
    }
  };
  auto add_dimension = [&](const std::string& id) {
    if (!id.empty() && dimensions_.contains(id) && visited_dimensions.insert(id).second) {
      dimension_queue.push_back(id);
    }
  };
  auto add_point = [&](const std::string& point_id) {
    if (!point_id.empty() && visited_points.insert(point_id).second) {
      if (const auto it = point_to_entities_.find(point_id); it != point_to_entities_.end()) {
        for (const auto& entity_id : it->second) {
          add_entity(entity_id);
        }
      }
    }
  };

  for (const auto& id : entity_ids) add_entity(id);
  for (const auto& id : constraint_ids) add_constraint(id);
  for (const auto& id : dimension_ids) add_dimension(id);

  while (!entity_queue.empty() || !constraint_queue.empty() || !dimension_queue.empty()) {
    while (!entity_queue.empty()) {
      const std::string entity_id = entity_queue.front();
      entity_queue.pop_front();

      for (const auto& point_id : EntityPointIds(entity_id)) {
        add_point(point_id);
      }
      if (const auto it = entity_to_constraints_.find(entity_id); it != entity_to_constraints_.end()) {
        for (const auto& constraint_id : it->second) add_constraint(constraint_id);
      }
      if (const auto it = entity_to_dimensions_.find(entity_id); it != entity_to_dimensions_.end()) {
        for (const auto& dimension_id : it->second) add_dimension(dimension_id);
      }
    }

    while (!constraint_queue.empty()) {
      const std::string constraint_id = constraint_queue.front();
      constraint_queue.pop_front();
      if (const auto it = constraint_to_entities_.find(constraint_id); it != constraint_to_entities_.end()) {
        for (const auto& entity_id : it->second) add_entity(entity_id);
      }
    }

    while (!dimension_queue.empty()) {
      const std::string dimension_id = dimension_queue.front();
      dimension_queue.pop_front();
      if (const auto it = dimension_to_entities_.find(dimension_id); it != dimension_to_entities_.end()) {
        for (const auto& entity_id : it->second) add_entity(entity_id);
      }
    }
  }

  ConstraintComponentData component;
  component.entity_ids.assign(visited_entities.begin(), visited_entities.end());
  component.constraint_ids.assign(visited_constraints.begin(), visited_constraints.end());
  component.dimension_ids.assign(visited_dimensions.begin(), visited_dimensions.end());
  SortUnique(&component.entity_ids);
  SortUnique(&component.constraint_ids);
  SortUnique(&component.dimension_ids);
  component.degrees_of_freedom = EstimateDegreesOfFreedom(model_, component.entity_ids,
                                                          component.constraint_ids,
                                                          component.dimension_ids);
  component.status = StatusForDegreesOfFreedom(component.degrees_of_freedom);
  return component;
}

std::vector<ConstraintComponentData> ConstraintGraph::Components() const {
  std::vector<ConstraintComponentData> components;
  std::unordered_set<std::string> covered_entities;

  std::vector<std::string> all_entity_ids;
  all_entity_ids.reserve(entities_.size());
  for (const auto& [id, _] : entities_) {
    all_entity_ids.push_back(id);
  }
  SortUnique(&all_entity_ids);

  int index = 0;
  for (const auto& entity_id : all_entity_ids) {
    if (covered_entities.contains(entity_id)) {
      continue;
    }
    auto component = TraverseFromEntities({entity_id}, {}, {});
    for (const auto& id : component.entity_ids) {
      covered_entities.insert(id);
    }
    component.id = "component:" + std::to_string(index++);
    components.push_back(std::move(component));
  }
  return components;
}

ConstraintComponentData ConstraintGraph::ComponentForSeed(const ConstraintGraphSeed& seed) const {
  auto component = TraverseFromEntities(seed.entity_ids, seed.constraint_ids, seed.dimension_ids);
  component.id = "affected:0";
  return component;
}

ConstraintGraphSeed ConstraintGraph::SeedForConstraint(const cccad::solver::v1::Constraint& constraint) const {
  return {.entity_ids = ConstraintEntityIds(constraint)};
}

ConstraintGraphSeed ConstraintGraph::SeedForIntent(const cccad::solver::v1::UserIntent& intent) const {
  switch (intent.kind_case()) {
    case cccad::solver::v1::UserIntent::kMovePoint:
      return {.entity_ids = {intent.move_point().point_id()}};
    case cccad::solver::v1::UserIntent::kMoveEntity:
      return {.entity_ids = {intent.move_entity().entity_id()}};
    case cccad::solver::v1::UserIntent::kSetDimension:
      return {.dimension_ids = {intent.set_dimension().dimension_id()}};
    case cccad::solver::v1::UserIntent::kAddConstraint:
      return SeedForConstraint(intent.add_constraint().constraint());
    case cccad::solver::v1::UserIntent::KIND_NOT_SET:
      return {};
  }
  return {};
}

int32_t ConstraintGraph::EstimateDegreesOfFreedom(const cccad::solver::v1::SketchModel& model,
                                                  const std::vector<std::string>& entity_ids,
                                                  const std::vector<std::string>& constraint_ids,
                                                  const std::vector<std::string>& dimension_ids) {
  std::unordered_set<std::string> entity_set(entity_ids.begin(), entity_ids.end());
  std::unordered_set<std::string> constraint_set(constraint_ids.begin(), constraint_ids.end());
  std::unordered_set<std::string> dimension_set(dimension_ids.begin(), dimension_ids.end());

  int32_t degrees = 0;
  for (const auto& entity : model.entities()) {
    if (!entity_set.contains(entity.id())) {
      continue;
    }
    switch (entity.kind_case()) {
      case cccad::solver::v1::Entity::kPoint:
        degrees += entity.point().fixed() ? 0 : 2;
        break;
      case cccad::solver::v1::Entity::kCircle:
        degrees += 1;
        break;
      case cccad::solver::v1::Entity::kLine:
      case cccad::solver::v1::Entity::kArc:
      case cccad::solver::v1::Entity::KIND_NOT_SET:
        break;
    }
  }

  for (const auto& constraint : model.constraints()) {
    if (constraint_set.contains(constraint.id()) && IsActive(constraint.status())) {
      degrees -= 1;
    }
  }
  for (const auto& dimension : model.dimensions()) {
    if (dimension_set.contains(dimension.id()) && IsActive(dimension.status()) && dimension.driving()) {
      degrees -= 1;
    }
  }
  return degrees;
}

cccad::solver::v1::SolveStatus ConstraintGraph::StatusForDegreesOfFreedom(int32_t degrees_of_freedom) {
  if (degrees_of_freedom < 0) {
    return cccad::solver::v1::SOLVE_STATUS_OVER_CONSTRAINED;
  }
  if (degrees_of_freedom == 0) {
    return cccad::solver::v1::SOLVE_STATUS_FULLY_CONSTRAINED;
  }
  return cccad::solver::v1::SOLVE_STATUS_UNDER_CONSTRAINED;
}

void CopyComponentToProto(const ConstraintComponentData& component,
                          cccad::solver::v1::ConstraintComponent* proto) {
  proto->set_id(component.id);
  for (const auto& id : component.entity_ids) proto->add_entity_ids(id);
  for (const auto& id : component.constraint_ids) proto->add_constraint_ids(id);
  for (const auto& id : component.dimension_ids) proto->add_dimension_ids(id);
  proto->set_degrees_of_freedom(component.degrees_of_freedom);
  proto->set_status(component.status);
}

}  // namespace cccad::solver
