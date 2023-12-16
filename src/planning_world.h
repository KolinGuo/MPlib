#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "articulated_model.h"
#include "macros_utils.h"
#include "types.h"

namespace mplib {

// WorldCollisionResultTplPtr
MPLIB_STRUCT_TEMPLATE_FORWARD(WorldCollisionResultTpl);

template <typename S>
struct WorldCollisionResultTpl {
  fcl::CollisionResult<S> res;
  std::string collision_type, object_name1, object_name2, link_name1,
      link_name2;
};

// Common Type Alias ==========================================================
using WorldCollisionResultf = WorldCollisionResultTpl<float>;
using WorldCollisionResultd = WorldCollisionResultTpl<double>;
using WorldCollisionResultfPtr = WorldCollisionResultTplPtr<float>;
using WorldCollisionResultdPtr = WorldCollisionResultTplPtr<double>;

// PlanningWorldTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(PlanningWorldTpl);

template <typename S>
class PlanningWorldTpl {
 private:
  using CollisionRequest = fcl::CollisionRequest<S>;
  using CollisionResult = fcl::CollisionResult<S>;

  using CollisionGeometry = fcl::CollisionGeometry<S>;
  using CollisionGeometryPtr = fcl::CollisionGeometryPtr<S>;

  using CollisionObject = fcl::CollisionObject<S>;
  using CollisionObjectPtr = fcl::CollisionObjectPtr<S>;

  using DynamicAABBTreeCollisionManager =
      fcl::DynamicAABBTreeCollisionManager<S>;
  using BroadPhaseCollisionManagerPtr = fcl::BroadPhaseCollisionManagerPtr<S>;

  using ArticulatedModel = ArticulatedModelTpl<S>;
  using ArticulatedModelPtr = ArticulatedModelTplPtr<S>;

  using WorldCollisionResult = WorldCollisionResultTpl<S>;
  using WorldCollisionResultPtr = WorldCollisionResultTplPtr<S>;

  std::vector<ArticulatedModelPtr> articulations_;
  std::vector<std::string> articulation_names_;
  std::unordered_map<std::string, CollisionObjectPtr> normal_objects_;

  int move_articulation_id_, attach_link_id_;
  CollisionObjectPtr point_cloud_, attached_tool_;
  bool has_point_cloud_, use_point_cloud_, has_attach_, use_attach_;
  Transform3<S> attach_to_link_pose_;
  // BroadPhaseCollisionManagerPtr normal_manager;

 public:
  PlanningWorldTpl(std::vector<ArticulatedModelPtr> const &articulations,
                   std::vector<std::string> const &articulation_names,
                   std::vector<CollisionObjectPtr> const &normal_objects = {},
                   std::vector<std::string> const &normal_object_names = {},
                   int const &plan_articulation_id = 0);
  // std::vector<bool> const &articulation_flags);

  const std::vector<ArticulatedModelPtr> &getArticulations() const {
    return articulations_;
  }

  const std::vector<std::string> &getArticulationNames() const {
    return articulation_names_;
  }

  bool hasArticulation(std::string const &name) const {
    for (const auto &n : articulation_names_)
      if (n == name) return true;
    return false;
  }

  const ArticulatedModelPtr getArticulation(std::string const &name) const {
    for (size_t i = 0; i < articulations_.size(); i++)
      if (articulation_names_[i] == name) return articulations_[i];
    return nullptr;
  }

  /**
   * @brief Gets names of all normal objects in world
   *
   * @return vector of normal object names
   */
  std::vector<std::string> getNormalObjectNames() const {
    std::vector<std::string> ret;
    for (const auto &object : normal_objects_) ret.push_back(object.first);
    return ret;
  }

  /**
   * @brief Gets the normal object with given name
   *
   * @return pointer to the FCL collision object with given name
   */
  const CollisionObjectPtr getNormalObject(std::string const &name) const {
    const auto it = normal_objects_.find(name);

    if (it != normal_objects_.end())
      return it->second;
    else
      return nullptr;
  }

  bool hasNormalObject(std::string const &name) const {
    return normal_objects_.find(name) != normal_objects_.end();
  }

  void setMoveArticulationId(int const &id) { move_articulation_id_ = id; }

  const int &getMoveArticulationId() const { return move_articulation_id_; }

  void setUsePointCloud(bool const &use) { use_point_cloud_ = use; }

  void updatePointCloud(MatrixX3<S> const &vertices, double const &resolution);

  void setUseAttach(bool const &use) {
    use_attach_ = use;
    if (!use) removeAttach();
  }

  /**
   * @brief remove attach object so nothing won't be anything on the end
   * effector when use_attach is set to true again
   */
  void removeAttach() { has_attach_ = false; }

  /**
   * @brief attach or update the attached object
   * @param p_geom shared ptr to a collision object
   * @param link_id id of the link to which the object is attached
   * @param pose the pose of the attached object w.r.t. the link it's attached
   * to
   */
  void updateAttachedTool(CollisionGeometryPtr const &p_geom,
                          int const &link_id, Vector7<S> const &pose);

  void updateAttachedSphere(S const &radius, int const &link_id,
                            const Vector7<S> &pose);

  void updateAttachedBox(Vector3<S> const &size, int const &link_id,
                         Vector7<S> const &pose);

  void updateAttachedMesh(std::string const &mesh_path, int const &link_id,
                          Vector7<S> const &pose);

  void printAttachedToolPose() const {
    auto tmp1 = attached_tool_.get()->getTranslation();
    auto tmp2 = attached_tool_.get()->getRotation();
    std::cout << tmp1 << ' ' << tmp2 << std::endl;
  }

  void addArticulation(std::string const &name,
                       ArticulatedModelPtr const &model) {
    articulations_.push_back(model);
    articulation_names_.push_back(name);
  }

  bool removeArticulation(std::string const &name) {
    for (size_t i = 0; i < articulations_.size(); i++)
      if (articulation_names_[i] == name) {
        articulations_.erase(articulations_.begin() + i);
        return true;
      }
    return false;
  }

  /**
   * @brief Adds a normal object with given name to world
   *
   * @param object name
   * @param FCL collision object pointer
   */
  void addNormalObject(std::string const &name,
                       CollisionObjectPtr const &collision_object) {
    normal_objects_[name] = collision_object;
  }

  bool removeNormalObject(std::string const &name) {
    const auto it = normal_objects_.find(name);

    if (it != normal_objects_.end()) {
      normal_objects_.erase(it);
      return true;
    }
    return false;
  }

  void setQpos(int const &index, VectorX<S> const &qpos) const;

  void setQposAll(VectorX<S> const &qpos) const;

  //   bool collide_among_normal_objects()=0;

  bool collide() const;

  std::vector<WorldCollisionResult> selfCollide(
      size_t const &index,
      CollisionRequest const &request = CollisionRequest()) const;
  std::vector<WorldCollisionResult> collideWithOthers(
      size_t const &index,
      CollisionRequest const &request = CollisionRequest()) const;
  std::vector<WorldCollisionResult> collideFull(
      size_t const &index,
      CollisionRequest const &request = CollisionRequest()) const;
};

// Common Type Alias ==========================================================
using PlanningWorldf = PlanningWorldTpl<float>;
using PlanningWorldd = PlanningWorldTpl<double>;
using PlanningWorldfPtr = PlanningWorldTplPtr<float>;
using PlanningWorlddPtr = PlanningWorldTplPtr<double>;

// Explicit Template Instantiation Declaration ================================
#define DECLARE_TEMPLATE_PLANNING_WORLD(S)          \
  extern template class WorldCollisionResultTpl<S>; \
  extern template class PlanningWorldTpl<S>

DECLARE_TEMPLATE_PLANNING_WORLD(float);
DECLARE_TEMPLATE_PLANNING_WORLD(double);

}  // namespace mplib
