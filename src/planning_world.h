#pragma once

#include <vector>

#include "articulated_model.h"
#include "fcl_model.h"
#include "macros_utils.hpp"

template <typename DATATYPE>
struct WorldCollisionResultTpl {
  fcl::CollisionResult<DATATYPE> res;
  // size_t object_id1, object_id2;
  std::string collision_type, object_name1, object_name2, link_name1,
      link_name2;
};

template <typename T>
using WorldCollisionResultTplPtr = std::shared_ptr<WorldCollisionResultTpl<T>>;

using WorldCollisionResultd = WorldCollisionResultTpl<double>;
using WorldCollisionResultf = WorldCollisionResultTpl<float>;
using WorldCollisionResultdPtr = WorldCollisionResultTplPtr<double>;
using WorldCollisionResultfPtr = WorldCollisionResultTplPtr<float>;

template <typename DATATYPE>
class PlanningWorldTpl {
 private:
  DEFINE_TEMPLATE_EIGEN(DATATYPE);
  using CollisionRequest = fcl::CollisionRequest<DATATYPE>;
  using CollisionResult = fcl::CollisionResult<DATATYPE>;

  using CollisionGeometry = fcl::CollisionGeometry<DATATYPE>;
  using CollisionGeometryPtr = std::shared_ptr<CollisionGeometry>;

  using CollisionObject = fcl::CollisionObject<DATATYPE>;
  using CollisionObjectPtr = std::shared_ptr<CollisionObject>;

  using DynamicAABBTreeCollisionManager =
      fcl::DynamicAABBTreeCollisionManager<DATATYPE>;
  using BroadPhaseCollisionManagerPtr =
      std::shared_ptr<fcl::BroadPhaseCollisionManager<DATATYPE>>;

  using ArticulatedModel = ArticulatedModelTpl<DATATYPE>;
  using ArticulatedModelPtr = ArticulatedModelTplPtr<DATATYPE>;

  using WorldCollisionResult = WorldCollisionResultTpl<DATATYPE>;
  using WorldCollisionResultPtr = WorldCollisionResultTplPtr<DATATYPE>;

  std::vector<ArticulatedModelPtr> articulations_;
  // std::vector<bool> articulation_flags;
  std::vector<CollisionObjectPtr> normal_objects_;  // without articulation

  std::vector<std::string> articulation_names_;
  std::vector<std::string> normal_object_names_;
  int move_articulation_id_, attach_link_id_;
  CollisionObjectPtr point_cloud_, attached_tool_;
  bool has_point_cloud_, use_point_cloud_, has_attach_, use_attach_;
  Transform3 attach_to_link_pose_;
  // BroadPhaseCollisionManagerPtr normal_manager;

 public:
  PlanningWorldTpl(std::vector<ArticulatedModelPtr> const &articulations,
                   std::vector<std::string> const &articulation_names,
                   std::vector<CollisionObjectPtr> const &normal_objects,
                   std::vector<std::string> const &normal_object_names,
                   int plan_articulation_id = 0);
  // std::vector<bool> const &articulation_flags);

  std::vector<ArticulatedModelPtr> &getArticulations(void) {
    return articulations_;
  }

  std::vector<CollisionObjectPtr> &getNormalObjects(void) {
    return normal_objects_;
  }

  std::vector<std::string> &getArticulationNames() {
    return articulation_names_;
  }

  std::vector<std::string> &getNormalObjectNames() {
    return normal_object_names_;
  }

  void setMoveArticulationId(int id) { move_articulation_id_ = id; }

  int getMoveArticulationId() { return move_articulation_id_; }

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

  void setUsePointCloud(bool const &use) { use_point_cloud_ = use; }

  void updatePointCloud(Matrixx3 const &vertices, double const &resolution);

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
  void updateAttachedTool(CollisionGeometryPtr p_geom, int link_id,
                          Vector7 const &pose);

  void updateAttachedSphere(DATATYPE radius, int link_id, const Vector7 &pose);

  void updateAttachedBox(Vector3 const &size, int link_id, Vector7 const &pose);

  void updateAttachedMesh(std::string const &mesh_path, int link_id,
                          Vector7 const &pose);

  void printAttachedToolPose() {
    auto tmp1 = attached_tool_.get()->getTranslation();
    auto tmp2 = attached_tool_.get()->getRotation();
    std::cout << tmp1 << ' ' << tmp2 << std::endl;
  }

  // std::vector<bool> &getArticulationFlags(void) { return articulation_flags;
  // }

  void addArticulation(
      ArticulatedModelPtr const &model,
      std::string const &name) {  // bool const &planning = true) {
    articulations_.push_back(model);
    articulation_names_.push_back(name);
    // articulation_flags.push_back(planning);
  }

  void addArticulations(std::vector<ArticulatedModelPtr> const &models,
                        std::vector<std::string> const
                            &names) {  // std::vector<bool> const &planning) {
    articulations_.insert(articulations_.end(), models.begin(), models.end());
    articulation_names_.insert(articulation_names_.end(), names.begin(),
                              names.end());
    // articulation_flags.insert(articulation_flags.end(), planning.begin(),
    // planning.end());
  }

  void addNormalObject(CollisionObjectPtr const &collision_object,
                       std::string const &name) {
    normal_objects_.push_back(collision_object);
    normal_object_names_.push_back(name);
  }

  void addNormalObjects(
      std::vector<CollisionObjectPtr> const &collision_objects,
      std::vector<std::string> const &names) {
    normal_objects_.insert(normal_objects_.end(), collision_objects.begin(),
                           collision_objects.end());
    normal_object_names_.insert(normal_object_names_.end(), names.begin(),
                               names.end());
  }

  void setQpos(int const &index, VectorX const &qpos);

  void setQposAll(VectorX const &qpos);

  //   bool collide_among_normal_objects()=0;

  bool collide();

  // std::vector<WorldCollisionResult> collideFull(void);
  std::vector<WorldCollisionResult> selfCollide(
      int index, CollisionRequest const &request = CollisionRequest());
  std::vector<WorldCollisionResult> collideWithOthers(
      int index, CollisionRequest const &request = CollisionRequest());
  std::vector<WorldCollisionResult> collideFull(
      int index, CollisionRequest const &request = CollisionRequest());
};

template <typename T>
using PlanningWorldTplPtr = std::shared_ptr<PlanningWorldTpl<T>>;

using PlanningWorldd = PlanningWorldTpl<double>;
using PlanningWorldf = PlanningWorldTpl<float>;
using PlanningWorlddPtr = PlanningWorldTplPtr<double>;
using PlanningWorldfPtr = PlanningWorldTplPtr<float>;
