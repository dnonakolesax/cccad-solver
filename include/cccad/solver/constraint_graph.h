#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "sketch_solver.pb.h"

namespace cccad::solver {

struct ConstraintComponentData {
  std::string id;
  std::vector<std::string> entity_ids;
  std::vector<std::string> constraint_ids;
  std::vector<std::string> dimension_ids;
  int32_t degrees_of_freedom = 0;
  cccad::solver::v1::SolveStatus status = cccad::solver::v1::SOLVE_STATUS_UNSPECIFIED;
};

struct ConstraintGraphSeed {
  std::vector<std::string> entity_ids;
  std::vector<std::string> constraint_ids;
  std::vector<std::string> dimension_ids;
};

class ConstraintGraph {
 public:
  explicit ConstraintGraph(const cccad::solver::v1::SketchModel& model);

  std::vector<ConstraintComponentData> Components() const;
  ConstraintComponentData ComponentForSeed(const ConstraintGraphSeed& seed) const;
  ConstraintGraphSeed SeedForIntent(const cccad::solver::v1::UserIntent& intent) const;
  ConstraintGraphSeed SeedForConstraint(const cccad::solver::v1::Constraint& constraint) const;

  static int32_t EstimateDegreesOfFreedom(const cccad::solver::v1::SketchModel& model,
                                          const std::vector<std::string>& entity_ids,
                                          const std::vector<std::string>& constraint_ids,
                                          const std::vector<std::string>& dimension_ids);
  static cccad::solver::v1::SolveStatus StatusForDegreesOfFreedom(int32_t degrees_of_freedom);

 private:
  std::vector<std::string> EntityPointIds(const std::string& entity_id) const;
  std::vector<std::string> ConstraintEntityIds(const cccad::solver::v1::Constraint& constraint) const;
  std::vector<std::string> DimensionEntityIds(const cccad::solver::v1::Dimension& dimension) const;
  ConstraintComponentData TraverseFromEntities(const std::vector<std::string>& entity_ids,
                                               const std::vector<std::string>& constraint_ids,
                                               const std::vector<std::string>& dimension_ids) const;

  const cccad::solver::v1::SketchModel& model_;
  std::unordered_map<std::string, const cccad::solver::v1::Entity*> entities_;
  std::unordered_map<std::string, const cccad::solver::v1::Constraint*> constraints_;
  std::unordered_map<std::string, const cccad::solver::v1::Dimension*> dimensions_;
  std::unordered_map<std::string, std::vector<std::string>> entity_to_points_;
  std::unordered_map<std::string, std::vector<std::string>> point_to_entities_;
  std::unordered_map<std::string, std::vector<std::string>> entity_to_constraints_;
  std::unordered_map<std::string, std::vector<std::string>> constraint_to_entities_;
  std::unordered_map<std::string, std::vector<std::string>> entity_to_dimensions_;
  std::unordered_map<std::string, std::vector<std::string>> dimension_to_entities_;
};

void SortUnique(std::vector<std::string>* values);
void CopyComponentToProto(const ConstraintComponentData& component,
                          cccad::solver::v1::ConstraintComponent* proto);

}  // namespace cccad::solver
