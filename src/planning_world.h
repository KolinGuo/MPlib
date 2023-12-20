#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "articulated_model.h"
#include "attached_body.h"
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
 public:
  // Common type alias
  using CollisionRequest = fcl::CollisionRequest<S>;
  using CollisionResult = fcl::CollisionResult<S>;
  using CollisionGeometryPtr = fcl::CollisionGeometryPtr<S>;
  using CollisionObject = fcl::CollisionObject<S>;
  using CollisionObjectPtr = fcl::CollisionObjectPtr<S>;
  using DynamicAABBTreeCollisionManager =
      fcl::DynamicAABBTreeCollisionManager<S>;
  using BroadPhaseCollisionManagerPtr = fcl::BroadPhaseCollisionManagerPtr<S>;

  using WorldCollisionResult = WorldCollisionResultTpl<S>;
  using ArticulatedModelPtr = ArticulatedModelTplPtr<S>;
  using AttachedBody = AttachedBodyTpl<S>;
  using AttachedBodyPtr = AttachedBodyTplPtr<S>;

  /**
   * @brief Constructs a planning world with the given articulations (always
   *  planned) and normal objects
   */
  PlanningWorldTpl(std::vector<ArticulatedModelPtr> const &articulations,
                   std::vector<std::string> const &articulation_names,
                   std::vector<CollisionObjectPtr> const &normal_objects = {},
                   std::vector<std::string> const &normal_object_names = {});

  /// @brief Gets names of all articulations in world (unordered)
  std::vector<std::string> getArticulationNames() const;

  /// @brief Gets the articulation (ArticulatedModelPtr) with given name
  std::vector<ArticulatedModelPtr> const &getPlannedArticulations() const {
    return planned_articulations_;
  }

  /// @brief Gets the articulation (ArticulatedModelPtr) with given name
  ArticulatedModelPtr getArticulation(std::string const &name) const {
    auto it = articulations_.find(name);
    return it != articulations_.end() ? it->second : nullptr;
  }

  /// @brief Whether articulation with given name exists
  bool hasArticulation(std::string const &name) const {
    return articulations_.find(name) != articulations_.end();
  }

  /**
   * @brief Adds an articulation (ArticulatedModelPtr) with given name to world
   * @param planned: whether the articulation is being planned
   */
  void addArticulation(std::string const &name,
                       ArticulatedModelPtr const &model, bool planned = false);

  /**
   * @brief Removes the articulation with given name if exists
   * @returns true if success, false if articulation with given name does not
   *  exist
   */
  bool removeArticulation(std::string const &name);

  /// @brief Whether articulation with given name exists and is being planned
  bool isArticulationPlanned(std::string const &name) const;

  /**
   * @brief Sets articulation with given name as being planned
   * @returns true if success, false if articulation with given name does not
   *  exist
   */
  bool setArticulationPlanned(std::string const &name, bool planned);

  /// @brief Gets names of all normal objects in world (unordered)
  std::vector<std::string> getNormalObjectNames() const;

  /// @brief Gets the normal object (CollisionObjectPtr) with given name
  CollisionObjectPtr getNormalObject(std::string const &name) const {
    auto it = normal_objects_.find(name);
    return it != normal_objects_.end() ? it->second : nullptr;
  }

  /// @brief Whether normal object with given name exists
  bool hasNormalObject(std::string const &name) const {
    return normal_objects_.find(name) != normal_objects_.end();
  }

  /// @brief Adds a normal object (CollisionObjectPtr) with given name to world
  void addNormalObject(std::string const &name,
                       CollisionObjectPtr const &collision_object) {
    normal_objects_[name] = collision_object;
  }

  /// @brief Adds a point cloud as a normal object with given name to world
  void addPointCloud(std::string const &name, MatrixX3<S> const &vertices,
                     double resolution = 0.01);

  /**
   * @brief Removes (and detaches) the normal object with given name if exists
   * @returns true if success, false if normal object with given name does not
   *  exist
   */
  bool removeNormalObject(std::string const &name);

  /// @brief Whether normal object with given name exists and is attached
  bool isNormalObjectAttached(std::string const &name) const;

  /**
   * @brief Attaches existing normal object to specified link of articulation
   * @param name: normal object name to attach
   * @param art_id: index of the planned articulation to attach to
   * @param link_id: index of the link of the planned articulation to attach to
   * @param pose: attached pose (relative pose from attached link to object)
   * @throws std::out_of_range if normal object does not already exists
   */
  void attachObject(std::string const &name, int art_id, int link_id,
                    Vector7<S> const &pose);

  /// @brief Attaches given object (w/ p_geom) to specified link of articulation
  void attachObject(std::string const &name, CollisionGeometryPtr const &p_geom,
                    int art_id, int link_id, Vector7<S> const &pose);

  /// @brief Attaches given sphere to specified link of articulation
  void attachSphere(S radius, int art_id, int link_id, Vector7<S> const &pose);

  /// @brief Attaches given box to specified link of articulation
  void attachBox(Vector3<S> const &size, int art_id, int link_id,
                 Vector7<S> const &pose);

  /// @brief Attaches given mesh to specified link of articulation
  void attachMesh(std::string const &mesh_path, int art_id, int link_id,
                  Vector7<S> const &pose);

  /**
   * @brief Detaches object with given name
   * @param also_remove: whether to also remove object from world
   * @returns true if success, false if normal object with given name does not
   *  exist
   */
  bool detachObject(std::string const &name, bool also_remove = false);

  /// @brief Prints global pose of all attached bodies
  void printAttachedBodyPose() const;

  /// @brief Set qpos of articulation with given name
  void setQpos(std::string const &name, VectorX<S> const &qpos) const;

  /// @brief Set qpos of all planned articulations
  void setQposAll(VectorX<S> const &state) const;

  /// @brief Check full collision and return only a boolean indicating collision
  bool collide(CollisionRequest const &request = CollisionRequest()) const {
    return collideFull(request).size() > 0;
  }

  /**
   * @brief Check self collision (including planned articulation self-collision,
   *  planned articulation-attach collision, attach-attach collision)
   */
  std::vector<WorldCollisionResult> selfCollide(
      CollisionRequest const &request = CollisionRequest()) const;

  /**
   * @brief Check collision with other scene bodies (planned articulations with
   * attached objects collide against unplanned articulations and scene objects)
   */
  std::vector<WorldCollisionResult> collideWithOthers(
      CollisionRequest const &request = CollisionRequest()) const;

  /// @brief Check full collision (calls selfCollide() and collideWithOthers())
  std::vector<WorldCollisionResult> collideFull(
      CollisionRequest const &request = CollisionRequest()) const;

 private:
  std::unordered_map<std::string, ArticulatedModelPtr> articulations_;
  std::unordered_map<std::string, CollisionObjectPtr> normal_objects_;

  std::vector<ArticulatedModelPtr> planned_articulations_;
  std::vector<AttachedBodyPtr> attached_bodies_;

  // TODO: Switch to BroadPhaseCollision
  // BroadPhaseCollisionManagerPtr normal_manager;

  /// @brief Update attached bodies global pose using current state
  void updateAttachedBodiesPose() const {
    for (const auto &attached_body : attached_bodies_)
      attached_body->updatePose();
  }
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
