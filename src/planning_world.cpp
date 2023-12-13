#include "planning_world.h"

#include "fcl/geometry/octree/octree.h"
#include "fcl_model.h"
#include "macros_utils.h"
#include "urdf_utils.h"

#define DEFINE_TEMPLATE_PW(DATATYPE)         \
  template class PlanningWorldTpl<DATATYPE>; \
  template class WorldCollisionResultTpl<DATATYPE>;

DEFINE_TEMPLATE_PW(double)

DEFINE_TEMPLATE_PW(float)

template <typename DATATYPE>
PlanningWorldTpl<DATATYPE>::PlanningWorldTpl(
    std::vector<ArticulatedModelPtr> const &articulations,
    std::vector<std::string> const &articulation_names,
    std::vector<CollisionObjectPtr> const &normal_objects,
    std::vector<std::string> const &normal_object_names,
    int const &move_articulation_id)
    : articulations_(articulations),
      articulation_names_(articulation_names),
      move_articulation_id_(move_articulation_id),
      has_point_cloud_(false),
      use_point_cloud_(false),
      has_attach_(false),
      use_attach_(false) {
  // articulation_flags(articulation_flags) {}
  ASSERT(articulations.size() == articulation_names.size(),
         "articulations and articulation_names should have the same size")
  ASSERT(normal_objects.size() == normal_object_names.size(),
         "normal_objects and normal_object_names should have the same size")
  for (size_t i = 0; i < normal_objects.size(); i++) {
    normal_objects_[normal_object_names[i]] = normal_objects[i];
  }
}

template <typename DATATYPE>
void PlanningWorldTpl<DATATYPE>::updatePointCloud(Matrixx3 const &vertices,
                                                  double const &resolution) {
  octomap::OcTree *tree = new octomap::OcTree(resolution);
  for (size_t i = 0; i < vertices.rows(); i++)
    tree->updateNode(
        octomap::point3d(vertices(i, 0), vertices(i, 1), vertices(i, 2)), true);

  auto tree_ptr = std::shared_ptr<const octomap::OcTree>(tree);
  auto pose = Transform3::Identity();
  point_cloud_ = std::make_shared<CollisionObject>(
      std::make_shared<fcl::OcTree<DATATYPE>>(tree_ptr), pose);
  has_point_cloud_ = true;
}

template <typename DATATYPE>
void PlanningWorldTpl<DATATYPE>::updateAttachedTool(
    CollisionGeometryPtr const &p_geom, int const &link_id,
    Vector7 const &pose) {
  attach_link_id_ = link_id;
  // linear here means the upper left 3x3 matrix, which is not necessarily a
  // rotation matrix if scaling is involved
  attach_to_link_pose_.linear() =
      Quaternion(pose[3], pose[4], pose[5], pose[6]).matrix();
  attach_to_link_pose_.translation() = pose.head(3);
  attached_tool_ =
      std::make_shared<CollisionObject>(p_geom, attach_to_link_pose_);
  has_attach_ = true;
}

template <typename DATATYPE>
void PlanningWorldTpl<DATATYPE>::updateAttachedSphere(DATATYPE const &radius,
                                                      int const &link_id,
                                                      const Vector7 &pose) {
  CollisionGeometryPtr collision_geometry =
      std::make_shared<fcl::Sphere<DATATYPE>>(radius);
  updateAttachedTool(collision_geometry, link_id, pose);
}

template <typename DATATYPE>
void PlanningWorldTpl<DATATYPE>::updateAttachedBox(Vector3 const &size,
                                                   int const &link_id,
                                                   Vector7 const &pose) {
  CollisionGeometryPtr collision_geometry =
      std::make_shared<fcl::Box<DATATYPE>>(size[0], size[1], size[2]);
  updateAttachedTool(collision_geometry, link_id, pose);
}

template <typename DATATYPE>
void PlanningWorldTpl<DATATYPE>::updateAttachedMesh(
    std::string const &mesh_path, int const &link_id, Vector7 const &pose) {
  CollisionGeometryPtr collision_geometry =
      load_mesh_as_BVH(mesh_path, Vector3(1, 1, 1));
  updateAttachedTool(collision_geometry, link_id, pose);
}

template <typename DATATYPE>
void PlanningWorldTpl<DATATYPE>::setQpos(int const &index,
                                         VectorX const &state) const {
  articulations_[index]->setQpos(state);
}

template <typename DATATYPE>
void PlanningWorldTpl<DATATYPE>::setQposAll(VectorX const &state) const {
  size_t total_dim = 0;
  for (size_t i = 0; i < articulations_.size(); i++) {
    auto n = articulations_[i]->getQposDim();
    auto segment =
        state.segment(total_dim, total_dim + n);  // [total_dim, total_dim + n)
    ASSERT(segment.size() == n, "Bug with size " +
                                    std::to_string(segment.size()) + " " +
                                    std::to_string(n));
    setQpos(i, segment);
    total_dim += n;
  }
  ASSERT(total_dim == state.size(), "State dimension is not correct");
}

template <typename DATATYPE>
std::vector<WorldCollisionResultTpl<DATATYPE>>
PlanningWorldTpl<DATATYPE>::selfCollide(int const &index,
                                        CollisionRequest const &request) const {
  std::vector<WorldCollisionResult> ret;
  auto fcl_model = articulations_[index]->getFCLModel();
  auto results = fcl_model.collideFull(request);
  auto CollisionLinkNames = fcl_model.getCollisionLinkNames();
  auto CollisionPairs = fcl_model.getCollisionPairs();
  for (size_t j = 0; j < results.size(); j++)
    if (results[j].isCollision()) {
      WorldCollisionResult tmp;
      auto x = CollisionPairs[j].first, y = CollisionPairs[j].second;
      tmp.res = results[j];
      tmp.object_name1 = articulation_names_[index];
      tmp.object_name2 = articulation_names_[index];
      tmp.collision_type = "self";
      tmp.link_name1 = CollisionLinkNames[x];
      tmp.link_name2 = CollisionLinkNames[y];
      ret.push_back(tmp);
    }
  return ret;
}

template <typename DATATYPE>
std::vector<WorldCollisionResultTpl<DATATYPE>>
PlanningWorldTpl<DATATYPE>::collideWithOthers(
    int const &index, CollisionRequest const &request) const {
  std::vector<WorldCollisionResult> ret;
  auto pinocchio_model = articulations_[index]->getPinocchioModel();
  auto fcl_model = articulations_[index]->getFCLModel();
  auto CollisionObjects = fcl_model.getCollisionObjects();
  auto CollisionLinkNames = fcl_model.getCollisionLinkNames();

  // collision with other articulations
  for (size_t i = 0; i < articulations_.size(); i++) {
    if (i == index) continue;
    auto fcl_model1 = articulations_[i]->getFCLModel();
    auto CollisionObjects1 = fcl_model.getCollisionObjects();
    auto CollisionLinkNames1 = fcl_model.getCollisionLinkNames();
    for (size_t j = 0; j < CollisionObjects.size(); j++)
      for (size_t k = 0; k < CollisionObjects1.size(); k++) {
        CollisionResult result;
        result.clear();
        fcl::collide(CollisionObjects[j].get(), CollisionObjects1[k].get(),
                     request, result);
        if (result.isCollision()) {
          WorldCollisionResult tmp;
          tmp.res = result;
          tmp.object_name1 = articulation_names_[index];
          tmp.object_name2 = articulation_names_[i];
          tmp.collision_type = "articulation";
          tmp.link_name1 = CollisionLinkNames[j];
          tmp.link_name2 = CollisionLinkNames[k];
          ret.push_back(tmp);
        }
      }
  }

  // collision with normal objects
  for (size_t i = 0; i < CollisionObjects.size(); i++)
    for (const auto &[name, obj] : normal_objects_) {
      CollisionResult result;
      result.clear();
      fcl::collide(CollisionObjects[i].get(), obj.get(), request, result);
      if (result.isCollision()) {
        WorldCollisionResult tmp;
        tmp.res = result;
        tmp.object_name1 = articulation_names_[index];
        tmp.object_name2 = name;
        tmp.collision_type = "normal_object";
        tmp.link_name1 = CollisionLinkNames[i];
        tmp.link_name2 = name;
        ret.push_back(tmp);
      }
    }

  if (use_point_cloud_) {
    if (has_point_cloud_ == false) {
      std::cout << "No Point Cloud Provided!" << std::endl;
    } else {
      for (size_t i = 0; i < CollisionObjects.size(); i++) {
        CollisionResult result;
        result.clear();
        fcl::collide(CollisionObjects[i].get(), point_cloud_.get(), request,
                     result);
        if (result.isCollision()) {
          WorldCollisionResult tmp;
          tmp.res = result;
          tmp.object_name1 = articulation_names_[index];
          tmp.object_name2 = "point_cloud";
          tmp.collision_type = "point_cloud";
          tmp.link_name1 = CollisionLinkNames[i];
          tmp.link_name2 = "point_cloud";
          ret.push_back(tmp);
        }
      }
    }
  }
  if (use_attach_ &&
      use_point_cloud_) {  // TODO: attached box with other articulated objects
    if (has_attach_ == false) {
      std::cout << "No Attached Box Provided!" << std::endl;
    } else if (has_point_cloud_ == false) {
      std::cout << "No Point Cloud Provided!" << std::endl;
    } else {  // currently, only collide with the point cloud, only support one
              // articulation
      Vector7 link_pose = pinocchio_model.getLinkPose(attach_link_id_);
      Transform3 pose;
      pose.linear() =
          Quaternion(link_pose[3], link_pose[4], link_pose[5], link_pose[6])
              .matrix();
      pose.translation() = link_pose.head(3);
      pose = pose * attach_to_link_pose_;
      // std::cout << "attached box pose: " << pose << std::endl;
      attached_tool_.get()->setTransform(pose);

      CollisionResult result;
      result.clear();
      fcl::collide(attached_tool_.get(), point_cloud_.get(), request, result);
      if (result.isCollision()) {
        WorldCollisionResult tmp;
        tmp.res = result;
        tmp.object_name1 = "attached_box";
        tmp.object_name2 = "point_cloud";
        tmp.collision_type = "attach";
        tmp.link_name1 = "attached_box";
        tmp.link_name2 = "point_cloud";
        ret.push_back(tmp);
      }
    }
  }
  return ret;
}

template <typename DATATYPE>
std::vector<WorldCollisionResultTpl<DATATYPE>>
PlanningWorldTpl<DATATYPE>::collideFull(int const &index,
                                        CollisionRequest const &request) const {
  std::vector<WorldCollisionResult> ret1 = selfCollide(index, request);
  std::vector<WorldCollisionResult> ret2 = collideWithOthers(index, request);
  ret1.insert(ret1.end(), ret2.begin(), ret2.end());
  return ret1;
}

template <typename DATATYPE>
bool PlanningWorldTpl<DATATYPE>::collide() const {
  std::vector<WorldCollisionResult> ret = collideFull(0, CollisionRequest());
  return ret.size() > 0;
}
