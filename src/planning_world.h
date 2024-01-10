#pragma once

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "articulated_model.h"
#include "attached_body.h"
#include "collision_matrix.h"
#include "macros_utils.h"
#include "types.h"

namespace mplib {

// WorldCollisionResultTplPtr
MPLIB_STRUCT_TEMPLATE_FORWARD(WorldCollisionResultTpl);

template <typename S>
struct WorldCollisionResultTpl {
  fcl::CollisionResult<S> res;
  std::string collision_type, object_name1, object_name2, link_name1, link_name2;
};

// Common Type Alias ==========================================================
using WorldCollisionResultf = WorldCollisionResultTpl<float>;
using WorldCollisionResultd = WorldCollisionResultTpl<double>;
using WorldCollisionResultfPtr = WorldCollisionResultTplPtr<float>;
using WorldCollisionResultdPtr = WorldCollisionResultTplPtr<double>;

// WorldDistanceResultTplPtr
MPLIB_STRUCT_TEMPLATE_FORWARD(WorldDistanceResultTpl);

template <typename S>
struct WorldDistanceResultTpl {
  fcl::DistanceResult<S> res;
  S min_distance;
  std::string distance_type, object_name1, object_name2, link_name1, link_name2;

  WorldDistanceResultTpl() : min_distance(std::numeric_limits<S>::max()) {}
};

// Common Type Alias ==========================================================
using WorldDistanceResultf = WorldDistanceResultTpl<float>;
using WorldDistanceResultd = WorldDistanceResultTpl<double>;
using WorldDistanceResultfPtr = WorldDistanceResultTplPtr<float>;
using WorldDistanceResultdPtr = WorldDistanceResultTplPtr<double>;

// PlanningWorldTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(PlanningWorldTpl);

template <typename S>
class PlanningWorldTpl {
 public:
  // Common type alias
  using CollisionRequest = fcl::CollisionRequest<S>;
  using CollisionResult = fcl::CollisionResult<S>;
  using DistanceRequest = fcl::DistanceRequest<S>;
  using DistanceResult = fcl::DistanceResult<S>;
  using CollisionGeometryPtr = fcl::CollisionGeometryPtr<S>;
  using CollisionObject = fcl::CollisionObject<S>;
  using CollisionObjectPtr = fcl::CollisionObjectPtr<S>;
  using DynamicAABBTreeCollisionManager = fcl::DynamicAABBTreeCollisionManager<S>;
  using BroadPhaseCollisionManagerPtr = fcl::BroadPhaseCollisionManagerPtr<S>;

  using WorldCollisionResult = WorldCollisionResultTpl<S>;
  using WorldDistanceResult = WorldDistanceResultTpl<S>;
  using ArticulatedModelPtr = ArticulatedModelTplPtr<S>;
  using AttachedBody = AttachedBodyTpl<S>;
  using AttachedBodyPtr = AttachedBodyTplPtr<S>;

  /**
   * @brief Constructs a planning world with the given articulations (always
   *  planned) and normal objects
   */
  PlanningWorldTpl(const std::vector<ArticulatedModelPtr> &articulations,
                   const std::vector<std::string> &articulation_names,
                   const std::vector<CollisionObjectPtr> &normal_objects = {},
                   const std::vector<std::string> &normal_object_names = {});

  /// @brief Gets names of all articulations in world (unordered)
  std::vector<std::string> getArticulationNames() const;

  /// @brief Gets all planned articulations (ArticulatedModelPtr)
  std::vector<ArticulatedModelPtr> getPlannedArticulations() const;

  /// @brief Gets the articulation (ArticulatedModelPtr) with given name
  ArticulatedModelPtr getArticulation(const std::string &name) const {
    auto it = articulations_.find(name);
    return it != articulations_.end() ? it->second : nullptr;
  }

  /// @brief Whether articulation with given name exists
  bool hasArticulation(const std::string &name) const {
    return articulations_.find(name) != articulations_.end();
  }

  /**
   * @brief Adds an articulation (ArticulatedModelPtr) with given name to world
   * @param planned: whether the articulation is being planned
   */
  void addArticulation(const std::string &name, const ArticulatedModelPtr &model,
                       bool planned = false);

  /**
   * @brief Removes the articulation with given name if exists. Updates acm_
   * @returns true if success, false if articulation with given name does not
   *  exist
   */
  bool removeArticulation(const std::string &name);

  /// @brief Whether articulation with given name is being planned
  bool isArticulationPlanned(const std::string &name) const {
    return planned_articulations_.find(name) != planned_articulations_.end();
  }

  /**
   * @brief Sets articulation with given name as being planned
   * @throws std::out_of_range if articulation with given name does not exist
   */
  void setArticulationPlanned(const std::string &name, bool planned);

  /// @brief Gets names of all normal objects in world (unordered)
  std::vector<std::string> getNormalObjectNames() const;

  /// @brief Gets the normal object (CollisionObjectPtr) with given name
  CollisionObjectPtr getNormalObject(const std::string &name) const {
    auto it = normal_objects_.find(name);
    return it != normal_objects_.end() ? it->second : nullptr;
  }

  /// @brief Whether normal object with given name exists
  bool hasNormalObject(const std::string &name) const {
    return normal_objects_.find(name) != normal_objects_.end();
  }

  /// @brief Adds a normal object (CollisionObjectPtr) with given name to world
  void addNormalObject(const std::string &name,
                       const CollisionObjectPtr &collision_object) {
    normal_objects_[name] = collision_object;
  }

  /// @brief Adds a point cloud as a normal object with given name to world
  void addPointCloud(const std::string &name, const MatrixX3<S> &vertices,
                     double resolution = 0.01);

  /**
   * @brief Removes (and detaches) the normal object with given name if exists.
   *  Updates acm_
   * @returns true if success, false if normal object with given name does not
   *  exist
   */
  bool removeNormalObject(const std::string &name);

  /// @brief Whether normal object with given name is attached
  bool isNormalObjectAttached(const std::string &name) const {
    return attached_bodies_.find(name) != attached_bodies_.end();
  }

  /// @brief Gets the attached normal object (AttachedBodyPtr) with given name
  AttachedBodyPtr getAttachedObject(const std::string &name) const {
    auto it = attached_bodies_.find(name);
    return it != attached_bodies_.end() ? it->second : nullptr;
  }

  /**
   * @brief Attaches existing normal object to specified link of articulation.
   *  If the object is currently attached, disallow collision between the object
   *  and previous touch_links.
   *  Updates acm_ to allow collisions between attached object and touch_links.
   * @param name: normal object name to attach
   * @param art_name: name of the planned articulation to attach to
   * @param link_id: index of the link of the planned articulation to attach to
   * @param pose: attached pose (relative pose from attached link to object)
   * @param touch_links: link names that the attached object touches
   * @throws std::out_of_range if normal object with given name does not exist
   *  or if planned articulation with given name does not exist
   */
  void attachObject(const std::string &name, const std::string &art_name, int link_id,
                    const Vector7<S> &pose,
                    const std::vector<std::string> &touch_links);

  /**
   * @brief Attaches existing normal object to specified link of articulation.
   *  If the object is not currently attached, automatically sets touch_links as
   *  the name of self links that collide with the object in the current state.
   *  Updates acm_ to allow collisions between attached object and touch_links.
   *  If the object is already attached, the touch_links of the attached object
   *  is preserved and acm_ remains unchanged.
   * @throws std::out_of_range if normal object with given name does not exist
   *  or if planned articulation with given name does not exist
   */
  void attachObject(const std::string &name, const std::string &art_name, int link_id,
                    const Vector7<S> &pose);

  /**
   * @brief Attaches given object (w/ p_geom) to specified link of articulation.
   *  This is done by removing normal object and then adding and attaching
   *  object. As a result, all previous acm_ entries with the object are removed
   * @param touch_links: link names that the attached object touches
   */
  void attachObject(const std::string &name, const CollisionGeometryPtr &p_geom,
                    const std::string &art_name, int link_id, const Vector7<S> &pose,
                    const std::vector<std::string> &touch_links);

  /**
   * @brief Attaches given object (w/ p_geom) to specified link of articulation.
   *  This is done by removing normal object and then adding and attaching
   *  object. As a result, all previous acm_ entries with the object are removed
   */
  void attachObject(const std::string &name, const CollisionGeometryPtr &p_geom,
                    const std::string &art_name, int link_id, const Vector7<S> &pose);

  /// @brief Attaches given sphere to specified link of articulation
  void attachSphere(S radius, const std::string &art_name, int link_id,
                    const Vector7<S> &pose);

  /// @brief Attaches given box to specified link of articulation
  void attachBox(const Vector3<S> &size, const std::string &art_name, int link_id,
                 const Vector7<S> &pose);

  /// @brief Attaches given mesh to specified link of articulation
  void attachMesh(const std::string &mesh_path, const std::string &art_name,
                  int link_id, const Vector7<S> &pose);

  /**
   * @brief Detaches object with given name. Updates acm_ to disallow collision
   *  between the object and touch_links
   * @param also_remove: whether to also remove object from world
   * @returns true if success, false if the object with given name is not
   *  attached
   */
  bool detachObject(const std::string &name, bool also_remove = false);

  /// @brief Prints global pose of all attached bodies
  void printAttachedBodyPose() const;

  /// @brief Set qpos of articulation with given name
  void setQpos(const std::string &name, const VectorX<S> &qpos) const;

  /// @brief Set qpos of all planned articulations
  void setQposAll(const VectorX<S> &state) const;

  /// @brief Get pointer to allowed collision matrix to modify
  AllowedCollisionMatrixPtr getAllowedCollisionMatrix() const { return acm_; }

  /// @brief Check full collision and return only a boolean indicating collision
  bool collide(const CollisionRequest &request = CollisionRequest()) const {
    return collideFull(request).size() > 0;
  }

  /**
   * @brief Check self collision (including planned articulation self-collision,
   *  planned articulation-attach collision, attach-attach collision)
   */
  std::vector<WorldCollisionResult> selfCollide(
      const CollisionRequest &request = CollisionRequest()) const;

  /**
   * @brief Check collision with other scene bodies (planned articulations with
   * attached objects collide against unplanned articulations and scene objects)
   */
  std::vector<WorldCollisionResult> collideWithOthers(
      const CollisionRequest &request = CollisionRequest()) const;

  /// @brief Check full collision (calls selfCollide() and collideWithOthers())
  std::vector<WorldCollisionResult> collideFull(
      const CollisionRequest &request = CollisionRequest()) const;

  /// @brief Returns the minimum distance to collision in current state
  S distance(const DistanceRequest &request = DistanceRequest()) const {
    return distanceFull().min_distance;
  }

  /// @brief The min distance to self-collision given the robot in current state
  WorldDistanceResult distanceSelf(
      const DistanceRequest &request = DistanceRequest()) const;

  /// @brief Compute the min distance between a robot and the world
  WorldDistanceResult distanceOthers(
      const DistanceRequest &request = DistanceRequest()) const;

  /**
   * @brief Compute the min distance to collision (calls distanceSelf() and
   *  distanceOthers())
   */
  WorldDistanceResult distanceFull(
      const DistanceRequest &request = DistanceRequest()) const;

 private:
  std::unordered_map<std::string, ArticulatedModelPtr> articulations_;
  std::unordered_map<std::string, CollisionObjectPtr> normal_objects_;

  // TODO: can planned_articulations_ be unordered_map? (setQposAll)
  std::map<std::string, ArticulatedModelPtr> planned_articulations_;
  std::unordered_map<std::string, AttachedBodyPtr> attached_bodies_;

  AllowedCollisionMatrixPtr acm_;

  // TODO: Switch to BroadPhaseCollision
  // BroadPhaseCollisionManagerPtr normal_manager;

  /// @brief Update attached bodies global pose using current state
  void updateAttachedBodiesPose() const {
    for (const auto &[name, attached_body] : attached_bodies_)
      attached_body->updatePose();
  }

  /// @brief Filter collisions using acm_
  std::vector<WorldCollisionResult> filterCollisions(
      const std::vector<WorldCollisionResult> &collisions) const;
};

// Common Type Alias ==========================================================
using PlanningWorldf = PlanningWorldTpl<float>;
using PlanningWorldd = PlanningWorldTpl<double>;
using PlanningWorldfPtr = PlanningWorldTplPtr<float>;
using PlanningWorlddPtr = PlanningWorldTplPtr<double>;

// Explicit Template Instantiation Declaration ================================
#define DECLARE_TEMPLATE_PLANNING_WORLD(S)          \
  extern template class WorldCollisionResultTpl<S>; \
  extern template class WorldDistanceResultTpl<S>;  \
  extern template class PlanningWorldTpl<S>

DECLARE_TEMPLATE_PLANNING_WORLD(float);
DECLARE_TEMPLATE_PLANNING_WORLD(double);

}  // namespace mplib
