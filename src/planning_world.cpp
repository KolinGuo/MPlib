#include "planning_world.h"

#include <octomap/OcTree.h>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>

#include "macros_utils.h"
#include "math_utils.h"
#include "urdf_utils.h"

namespace mplib {

// Explicit Template Instantiation Definition =================================
#define DEFINE_TEMPLATE_PLANNING_WORLD(S)    \
  template class WorldCollisionResultTpl<S>; \
  template class PlanningWorldTpl<S>

DEFINE_TEMPLATE_PLANNING_WORLD(float);
DEFINE_TEMPLATE_PLANNING_WORLD(double);

template <typename S>
PlanningWorldTpl<S>::PlanningWorldTpl(
    std::vector<ArticulatedModelPtr> const &articulations,
    std::vector<std::string> const &articulation_names,
    std::vector<CollisionObjectPtr> const &normal_objects,
    std::vector<std::string> const &normal_object_names) {
  ASSERT(articulations.size() == articulation_names.size(),
         "articulations and articulation_names should have the same size");
  ASSERT(normal_objects.size() == normal_object_names.size(),
         "normal_objects and normal_object_names should have the same size");
  for (size_t i = 0; i < articulations.size(); i++) {
    articulations[i]->setName(articulation_names[i]);
    articulations_[articulation_names[i]] = articulations[i];
    planned_articulations_.push_back(articulations[i]);
  }
  for (size_t i = 0; i < normal_objects.size(); i++) {
    normal_objects_[normal_object_names[i]] = normal_objects[i];
  }
}

template <typename S>
std::vector<std::string> PlanningWorldTpl<S>::getArticulationNames() const {
  std::vector<std::string> names;
  for (const auto &art : articulations_) names.push_back(art.first);
  return names;
}

template <typename S>
void PlanningWorldTpl<S>::addArticulation(std::string const &name,
                                          ArticulatedModelPtr const &model,
                                          bool planned) {
  model->setName(name);
  articulations_[name] = model;
  setArticulationPlanned(name, planned);
}

template <typename S>
bool PlanningWorldTpl<S>::removeArticulation(std::string const &name) {
  auto it = articulations_.find(name);
  if (it == articulations_.end()) return false;
  auto art = it->second;
  articulations_.erase(it);

  auto it2 = std::find(planned_articulations_.begin(),
                       planned_articulations_.end(), art);
  if (it2 != planned_articulations_.end()) planned_articulations_.erase(it2);
  return true;
}

template <typename S>
bool PlanningWorldTpl<S>::isArticulationPlanned(std::string const &name) const {
  auto it = articulations_.find(name);
  return it != articulations_.end() &&
         std::find(planned_articulations_.begin(), planned_articulations_.end(),
                   it->second) != planned_articulations_.end();
}

template <typename S>
bool PlanningWorldTpl<S>::setArticulationPlanned(std::string const &name,
                                                 bool planned) {
  auto it = articulations_.find(name);
  if (it == articulations_.end()) return false;

  auto art = it->second;
  auto it2 = std::find(planned_articulations_.begin(),
                       planned_articulations_.end(), art);
  if (planned && it2 == planned_articulations_.end())
    planned_articulations_.push_back(art);
  else if (!planned && it2 != planned_articulations_.end())
    planned_articulations_.erase(it2);
  return true;
}

template <typename S>
std::vector<std::string> PlanningWorldTpl<S>::getNormalObjectNames() const {
  std::vector<std::string> names;
  for (const auto &object : normal_objects_) names.push_back(object.first);
  return names;
}

template <typename S>
void PlanningWorldTpl<S>::addPointCloud(std::string const &name,
                                        MatrixX3<S> const &vertices,
                                        double resolution) {
  auto tree = std::make_shared<octomap::OcTree>(resolution);
  for (const auto &row : vertices.rowwise())
    tree->updateNode(octomap::point3d(row(0), row(1), row(2)), true);
  auto obj =
      std::make_shared<CollisionObject>(std::make_shared<fcl::OcTree<S>>(tree));
  addNormalObject(name, obj);
}

template <typename S>
bool PlanningWorldTpl<S>::removeNormalObject(std::string const &name) {
  auto it = normal_objects_.find(name);
  if (it == normal_objects_.end()) return false;
  auto obj = it->second;
  normal_objects_.erase(it);

  auto it2 = std::find_if(
      attached_bodies_.begin(), attached_bodies_.end(),
      [&](AttachedBodyPtr const &p) { return p->getObject() == obj; });
  if (it2 != attached_bodies_.end()) attached_bodies_.erase(it2);
  return true;
}

template <typename S>
bool PlanningWorldTpl<S>::isNormalObjectAttached(
    std::string const &name) const {
  auto it = normal_objects_.find(name);
  return it != normal_objects_.end() &&
         std::find_if(attached_bodies_.begin(), attached_bodies_.end(),
                      [&](AttachedBodyPtr const &p) {
                        return p->getObject() == it->second;
                      }) != attached_bodies_.end();
}

template <typename S>
void PlanningWorldTpl<S>::attachObject(std::string const &name, int art_id,
                                       int link_id, Vector7<S> const &pose) {
  auto obj = normal_objects_.at(name);
  auto it = std::find_if(
      attached_bodies_.begin(), attached_bodies_.end(),
      [&](AttachedBodyPtr const &p) { return p->getObject() == obj; });
  if (it != attached_bodies_.end()) attached_bodies_.erase(it);

  auto body = std::make_shared<AttachedBody>(
      name, obj, planned_articulations_.at(art_id), link_id,
      posevec_to_transform(pose));
  attached_bodies_.push_back(body);
}

template <typename S>
void PlanningWorldTpl<S>::attachObject(std::string const &name,
                                       CollisionGeometryPtr const &p_geom,
                                       int art_id, int link_id,
                                       Vector7<S> const &pose) {
  removeNormalObject(name);
  addNormalObject(name, std::make_shared<CollisionObject>(p_geom));
  attachObject(name, art_id, link_id, pose);
}

template <typename S>
void PlanningWorldTpl<S>::attachSphere(S radius, int art_id, int link_id,
                                       Vector7<S> const &pose) {
  // FIXME: Use art_name/link_name to avoid changes
  auto name =
      std::to_string(art_id) + "_" + std::to_string(link_id) + "_sphere";
  attachObject(name, std::make_shared<fcl::Sphere<S>>(radius), art_id, link_id,
               pose);
}

template <typename S>
void PlanningWorldTpl<S>::attachBox(Vector3<S> const &size, int art_id,
                                    int link_id, Vector7<S> const &pose) {
  // FIXME: Use art_name/link_name to avoid changes
  auto name = std::to_string(art_id) + "_" + std::to_string(link_id) + "_box";
  attachObject(name, std::make_shared<fcl::Box<S>>(size), art_id, link_id,
               pose);
}

template <typename S>
void PlanningWorldTpl<S>::attachMesh(std::string const &mesh_path, int art_id,
                                     int link_id, Vector7<S> const &pose) {
  // FIXME: Use art_name/link_name to avoid changes
  auto name = std::to_string(art_id) + "_" + std::to_string(link_id) + "_mesh";
  attachObject(name, load_mesh_as_BVH(mesh_path, Vector3<S>(1, 1, 1)), art_id,
               link_id, pose);
}

template <typename S>
bool PlanningWorldTpl<S>::detachObject(std::string const &name,
                                       bool also_remove) {
  auto it = normal_objects_.find(name);
  if (it == normal_objects_.end()) return false;
  auto obj = it->second;
  if (also_remove) normal_objects_.erase(it);

  auto it2 = std::find_if(
      attached_bodies_.begin(), attached_bodies_.end(),
      [&](AttachedBodyPtr const &p) { return p->getObject() == obj; });
  if (it2 != attached_bodies_.end()) attached_bodies_.erase(it2);
  return true;
}

template <typename S>
void PlanningWorldTpl<S>::printAttachedBodyPose() const {
  for (const auto &body : attached_bodies_)
    std::cout << body->getName() << " global pose:\n"
              << body->getGlobalPose().matrix() << std::endl;
}

template <typename S>
void PlanningWorldTpl<S>::setQpos(std::string const &name,
                                  VectorX<S> const &qpos) const {
  articulations_.at(name)->setQpos(qpos);
}

template <typename S>
void PlanningWorldTpl<S>::setQposAll(VectorX<S> const &state) const {
  size_t i = 0;
  for (const auto &art : planned_articulations_) {
    auto n = art->getQposDim();
    auto qpos = state.segment(i, n);  // [i, i + n)
    ASSERT(static_cast<size_t>(qpos.size()) == n,
           "Bug with size " + std::to_string(qpos.size()) + " " +
               std::to_string(n));
    art->setQpos(qpos);
    i += n;
  }
  ASSERT(i == static_cast<size_t>(state.size()),
         "State dimension is not correct");
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::selfCollide(
    CollisionRequest const &request) const {
  std::vector<WorldCollisionResult> ret;
  CollisionResult result;

  updateAttachedBodiesPose();

  // Collision involving planned articulation
  for (const auto &art : planned_articulations_) {
    auto art_name = art->getName();
    auto fcl_model = art->getFCLModel();
    auto col_objs = fcl_model->getCollisionObjects();
    auto col_link_names = fcl_model->getCollisionLinkNames();
    auto col_pairs = fcl_model->getCollisionPairs();

    // Articulation self-collision
    auto results = fcl_model->collideFull(request);
    for (size_t i = 0; i < results.size(); i++)
      if (results[i].isCollision()) {
        WorldCollisionResult tmp;
        auto x = col_pairs[i].first, y = col_pairs[i].second;
        tmp.res = results[i];
        tmp.object_name1 = art_name;
        tmp.object_name2 = art_name;
        tmp.collision_type = "self";
        tmp.link_name1 = col_link_names[x];
        tmp.link_name2 = col_link_names[y];
        ret.push_back(tmp);
      }

    // Articulation collide with attached_bodies_
    for (const auto &attached_body : attached_bodies_) {
      auto attached_obj = attached_body->getObject();
      auto attached_body_name = attached_body->getName();
      for (size_t i = 0; i < col_objs.size(); i++) {
        result.clear();
        ::fcl::collide(attached_obj.get(), col_objs[i].get(), request, result);
        if (result.isCollision()) {
          WorldCollisionResult tmp;
          tmp.res = result;
          tmp.object_name1 = art_name;
          tmp.object_name2 = attached_body_name;
          tmp.collision_type = "self_attach";
          tmp.link_name1 = col_link_names[i];
          tmp.link_name2 = attached_body_name;
          ret.push_back(tmp);
        }
      }
    }
  }

  // Collision among attached_bodies_
  for (size_t i = 0; i < attached_bodies_.size(); i++)
    for (size_t j = 0; j < i; j++) {
      result.clear();
      ::fcl::collide(attached_bodies_[i]->getObject().get(),
                     attached_bodies_[j]->getObject().get(), request, result);
      if (result.isCollision()) {
        auto name1 = attached_bodies_[i]->getName(),
             name2 = attached_bodies_[j]->getName();
        WorldCollisionResult tmp;
        tmp.res = result;
        tmp.object_name1 = name1;
        tmp.object_name2 = name2;
        tmp.collision_type = "attach_attach";
        tmp.link_name1 = name1;
        tmp.link_name2 = name2;
        ret.push_back(tmp);
      }
    }
  return ret;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::collideWithOthers(
    CollisionRequest const &request) const {
  std::vector<WorldCollisionResult> ret;
  CollisionResult result;

  updateAttachedBodiesPose();

  // Collect unplanned articulations, not attached scene objects
  std::vector<ArticulatedModelPtr> unplanned_articulations;
  std::unordered_map<std::string, CollisionObjectPtr> scene_objects;
  for (const auto &[name, art] : articulations_) {
    auto it2 = std::find(planned_articulations_.begin(),
                         planned_articulations_.end(), art);
    if (it2 == planned_articulations_.end())
      unplanned_articulations.push_back(art);
  }
  for (const auto &[name, obj] : normal_objects_) {
    auto it2 = std::find_if(
        attached_bodies_.begin(), attached_bodies_.end(),
        [&](AttachedBodyPtr const &p) { return p->getObject() == obj; });
    if (it2 == attached_bodies_.end()) scene_objects[name] = obj;
  }

  // Collision involving planned articulation
  for (const auto &art : planned_articulations_) {
    auto art_name = art->getName();
    auto fcl_model = art->getFCLModel();
    auto col_objs = fcl_model->getCollisionObjects();
    auto col_link_names = fcl_model->getCollisionLinkNames();

    // Collision with unplanned articulation
    for (const auto &art2 : unplanned_articulations) {
      auto art_name2 = art2->getName();
      auto fcl_model2 = art2->getFCLModel();
      auto col_objs2 = fcl_model2->getCollisionObjects();
      auto col_link_names2 = fcl_model2->getCollisionLinkNames();

      for (size_t i = 0; i < col_objs.size(); i++)
        for (size_t j = 0; j < col_objs2.size(); j++) {
          result.clear();
          ::fcl::collide(col_objs[i].get(), col_objs2[j].get(), request,
                         result);
          if (result.isCollision()) {
            WorldCollisionResult tmp;
            tmp.res = result;
            tmp.object_name1 = art_name;
            tmp.object_name2 = art_name2;
            tmp.collision_type = "articulation_articulation";
            tmp.link_name1 = col_link_names[i];
            tmp.link_name2 = col_link_names2[j];
            ret.push_back(tmp);
          }
        }
    }

    // Collision with scene objects
    for (const auto &[name, obj] : scene_objects)
      for (size_t i = 0; i < col_objs.size(); i++) {
        result.clear();
        ::fcl::collide(col_objs[i].get(), obj.get(), request, result);
        if (result.isCollision()) {
          WorldCollisionResult tmp;
          tmp.res = result;
          tmp.object_name1 = art_name;
          tmp.object_name2 = name;
          tmp.collision_type = "articulation_sceneobject";
          tmp.link_name1 = col_link_names[i];
          tmp.link_name2 = name;
          ret.push_back(tmp);
        }
      }
  }

  // Collision involving attached_bodies_
  for (const auto &attached_body : attached_bodies_) {
    auto attached_obj = attached_body->getObject();
    auto attached_body_name = attached_body->getName();

    // Collision with unplanned articulation
    for (const auto &art2 : unplanned_articulations) {
      auto art_name2 = art2->getName();
      auto fcl_model2 = art2->getFCLModel();
      auto col_objs2 = fcl_model2->getCollisionObjects();
      auto col_link_names2 = fcl_model2->getCollisionLinkNames();

      for (size_t i = 0; i < col_objs2.size(); i++) {
        result.clear();
        ::fcl::collide(attached_obj.get(), col_objs2[i].get(), request, result);
        if (result.isCollision()) {
          WorldCollisionResult tmp;
          tmp.res = result;
          tmp.object_name1 = attached_body_name;
          tmp.object_name2 = art_name2;
          tmp.collision_type = "attach_articulation";
          tmp.link_name1 = attached_body_name;
          tmp.link_name2 = col_link_names2[i];
          ret.push_back(tmp);
        }
      }
    }

    // Collision with scene objects
    for (const auto &[name, obj] : scene_objects) {
      result.clear();
      ::fcl::collide(attached_obj.get(), obj.get(), request, result);
      if (result.isCollision()) {
        WorldCollisionResult tmp;
        tmp.res = result;
        tmp.object_name1 = attached_body_name;
        tmp.object_name2 = name;
        tmp.collision_type = "attach_sceneobject";
        tmp.link_name1 = attached_body_name;
        tmp.link_name2 = name;
        ret.push_back(tmp);
      }
    }
  }
  return ret;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::collideFull(
    CollisionRequest const &request) const {
  auto ret1 = selfCollide(request);
  auto ret2 = collideWithOthers(request);
  ret1.insert(ret1.end(), ret2.begin(), ret2.end());
  return ret1;
}

}  // namespace mplib
