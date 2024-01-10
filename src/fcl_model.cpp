#include "fcl_model.h"

#include <algorithm>

#include <boost/filesystem/path.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <urdf_parser/urdf_parser.h>

#include "macros_utils.h"
#include "urdf_utils.h"

namespace mplib::fcl {

// Explicit Template Instantiation Definition =================================
#define DEFINE_TEMPLATE_FCL_MODEL(S) template class FCLModelTpl<S>

DEFINE_TEMPLATE_FCL_MODEL(float);
DEFINE_TEMPLATE_FCL_MODEL(double);

template <typename S>
FCLModelTpl<S>::FCLModelTpl(const urdf::ModelInterfaceSharedPtr &urdfTree,
                            const std::string &package_dir, bool verbose, bool convex)
    : use_convex_(convex), verbose_(verbose) {
  init(urdfTree, package_dir);
}

template <typename S>
FCLModelTpl<S>::FCLModelTpl(const std::string &urdf_filename, bool verbose, bool convex)
    : use_convex_(convex), verbose_(verbose) {
  auto found = urdf_filename.find_last_of("/\\");
  auto urdf_dir = urdf_filename.substr(0, found);
  urdf::ModelInterfaceSharedPtr urdfTree = urdf::parseURDFFile(urdf_filename);
  init(urdfTree, urdf_dir);
}

template <typename S>
std::unique_ptr<FCLModelTpl<S>> FCLModelTpl<S>::createFromURDFString(
    const std::string &urdf_string,
    const std::vector<std::pair<std::string, std::vector<CollisionObjectPtr<S>>>>
        &collision_links,
    bool verbose) {
  auto urdf = urdf::parseURDF(urdf_string);
  // package_dir is not needed since urdf_string contains no visual/collision elements
  auto fcl_model = std::make_unique<FCLModelTpl<S>>(urdf, "", verbose, false);

  for (const auto &[link_name, collision_objs] : collision_links)
    for (const auto &collision_obj : collision_objs) {
      fcl_model->collision_objects_.push_back(collision_obj);
      fcl_model->collision_link_names_.push_back(link_name);
      // fcl_model->parent_link_names_.push_back(parent_link_name);  // FIXME: remove
      fcl_model->collision_origin2link_poses_.push_back(collision_obj->getTransform());
    }
  // setLinkOrder with unique collision link names as user_link_names
  auto user_link_names = fcl_model->collision_link_names_;
  auto last = std::unique(user_link_names.begin(), user_link_names.end());
  user_link_names.erase(last, user_link_names.end());
  fcl_model->setLinkOrder(user_link_names);

  // We assume that the collisions between objects on the same link can be ignored.
  for (size_t i = 0; i < fcl_model->collision_link_names_.size(); i++)
    for (size_t j = 0; j < i; j++)
      if (fcl_model->collision_link_names_[i] != fcl_model->collision_link_names_[j])
        fcl_model->collision_pairs_.push_back(std::make_pair(j, i));

  return fcl_model;
}

template <typename S>
void FCLModelTpl<S>::setLinkOrder(const std::vector<std::string> &names) {
  user_link_names_ = names;
  collision_link_user_indices_ = {};
  for (size_t i = 0; i < collision_link_names_.size(); i++) {
    if (verbose_) std::cout << collision_link_names_[i] << " " << names[i] << std::endl;
    auto iter = std::find(names.begin(), names.end(), collision_link_names_[i]);
    if (iter == names.end())
      throw std::invalid_argument("The names does not contain link " +
                                  collision_link_names_[i]);
    size_t link_i = iter - names.begin();
    collision_link_user_indices_.push_back(link_i);
  }
}

template <typename S>
void FCLModelTpl<S>::printCollisionPairs() const {
  for (const auto &cp : collision_pairs_) {
    auto i = cp.first, j = cp.second;
    std::cout << collision_link_names_[i] << " " << collision_link_names_[j]
              << std::endl;
  }
}

template <typename S>
void FCLModelTpl<S>::removeCollisionPairsFromSRDF(const std::string &srdf_filename) {
  const std::string extension =
      srdf_filename.substr(srdf_filename.find_last_of('.') + 1);
  if (srdf_filename == "") {
    std::cout << "No SRDF file provided!" << std::endl;
    return;
  }

  ASSERT(extension == "srdf", srdf_filename + " does not have the right extension.");

  std::ifstream srdf_stream(srdf_filename.c_str());

  ASSERT(srdf_stream.is_open(), "Cannot open " + srdf_filename);

  std::stringstream buffer;
  buffer << srdf_stream.rdbuf();
  removeCollisionPairsFromSRDFString(buffer.str());
}

template <typename S>
void FCLModelTpl<S>::removeCollisionPairsFromSRDFString(const std::string &srdf_string) {
  std::istringstream srdf_stream(srdf_string);

  boost::property_tree::ptree pt;
  boost::property_tree::xml_parser::read_xml(srdf_stream, pt);

  for (const auto &node : pt.get_child("robot"))
    if (node.first == "disable_collisions") {
      const std::string link1 = node.second.get<std::string>("<xmlattr>.link1");
      const std::string link2 = node.second.get<std::string>("<xmlattr>.link2");
      if (verbose_)
        std::cout << "Try to Remove collision parts:" << link1 << " " << link2
                  << std::endl;
      for (auto iter = collision_pairs_.begin(); iter != collision_pairs_.end();)
        if ((collision_link_names_[iter->first] == link1 &&
             collision_link_names_[iter->second] == link2) ||
            (collision_link_names_[iter->first] == link2 &&
             collision_link_names_[iter->second] == link1))
          iter = collision_pairs_.erase(iter);
        else
          iter++;
    }
}

template <typename S>
void FCLModelTpl<S>::updateCollisionObjects(
    const std::vector<Transform3<S>> &link_pose) const {
  for (size_t i = 0; i < collision_objects_.size(); i++) {
    auto link_i = collision_link_user_indices_[i];
    Transform3<S> t_i = link_pose[link_i] * collision_origin2link_poses_[i];
    collision_objects_[i].get()->setTransform(t_i);
    // auto tmp1 = collision_objects[i].get()->getTranslation();
    // std::cout << collision_objects[i].get()->getTranslation() << std::endl;
  }
}

template <typename S>
void FCLModelTpl<S>::updateCollisionObjects(
    const std::vector<Vector7<S>> &link_pose) const {
  for (size_t i = 0; i < collision_objects_.size(); i++) {
    auto link_i = collision_link_user_indices_[i];
    Transform3<S> tt_i;
    tt_i.linear() = Quaternion<S>(link_pose[link_i][3], link_pose[link_i][4],
                                  link_pose[link_i][5], link_pose[link_i][6])
                        .matrix();
    tt_i.translation() = link_pose[link_i].head(3);
    Transform3<S> t_i = tt_i * collision_origin2link_poses_[i];
    collision_objects_[i].get()->setTransform(t_i);
    // auto tmp1 = collision_objects[i].get()->getTranslation();
    // auto tmp2 = collision_objects[i].get()->getRotation();
    // Transform3 tmp = collision_objects[i]->getTransform();
    // std::cout << collision_objects[i].get()->getTranslation() << std::endl;
  }
}

template <typename S>
bool FCLModelTpl<S>::collide(const CollisionRequest<S> &request) const {
  // result will be returned via the collision result structure
  CollisionResult<S> result;
  for (const auto &col_pair : collision_pairs_) {
    ::fcl::collide(collision_objects_[col_pair.first].get(),
                   collision_objects_[col_pair.second].get(), request, result);
    if (result.isCollision()) return true;
  }
  return false;
}

template <typename S>
std::vector<CollisionResult<S>> FCLModelTpl<S>::collideFull(
    const CollisionRequest<S> &request) const {
  std::vector<CollisionResult<S>> ret;
  for (const auto &col_pair : collision_pairs_) {
    CollisionResult<S> result;
    result.clear();
    ::fcl::collide(collision_objects_[col_pair.first].get(),
                   collision_objects_[col_pair.second].get(), request, result);
    ret.push_back(result);
  }
  return ret;
}

template <typename S>
void FCLModelTpl<S>::dfs_parse_tree(const urdf::LinkConstSharedPtr &link,
                                    const std::string &parent_link_name) {
  // const urdf::JointConstSharedPtr joint =
  // urdf::const_pointer_cast<urdf::Joint>(link->parent_joint); const Transform3
  // joint_placement =
  // pose_to_transform<S>(joint->parent_to_joint_origin_transform);

  if (link->collision) {
    for (const auto &col_obj : link->collision_array) {
      const auto &geom = col_obj->geometry;
      CollisionGeometryPtr<S> collision_geometry = nullptr;
      auto pose = Transform3<S>::Identity();
      if (geom->type == urdf::Geometry::MESH) {
        const urdf::MeshConstSharedPtr urdf_mesh =
            urdf::dynamic_pointer_cast<const urdf::Mesh>(geom);
        std::string file_name = urdf_mesh->filename;
        if (use_convex_ && file_name.find(".convex.stl") == std::string::npos)
          file_name = file_name += ".convex.stl";
        auto mesh_path = (boost::filesystem::path(package_dir_) / file_name).string();
        if (mesh_path == "") {
          std::stringstream ss;
          ss << "Mesh " << file_name << " could not be found.";
          throw std::invalid_argument(ss.str());
        }
        if (verbose_) std::cout << "File name " << file_name << std::endl;
        Vector3<S> scale = {static_cast<S>(urdf_mesh->scale.x),
                            static_cast<S>(urdf_mesh->scale.y),
                            static_cast<S>(urdf_mesh->scale.z)};
        if (use_convex_)
          collision_geometry = load_mesh_as_Convex(mesh_path, scale);
        else
          collision_geometry = load_mesh_as_BVH(mesh_path, scale);
        if (verbose_) std::cout << scale << " " << collision_geometry << std::endl;
      } else if (geom->type == urdf::Geometry::CYLINDER) {
        const urdf::CylinderConstSharedPtr cylinder =
            urdf::dynamic_pointer_cast<const urdf::Cylinder>(geom);
        collision_geometry = std::make_shared<Cylinder<S>>(
            static_cast<S>(cylinder->radius), static_cast<S>(cylinder->length));
      } else if (geom->type == urdf::Geometry::BOX) {
        const urdf::BoxConstSharedPtr box =
            urdf::dynamic_pointer_cast<const urdf::Box>(geom);
        collision_geometry = std::make_shared<Box<S>>(static_cast<S>(box->dim.x),
                                                      static_cast<S>(box->dim.y),
                                                      static_cast<S>(box->dim.z));
      } else if (geom->type == ::urdf::Geometry::SPHERE) {
        const urdf::SphereConstSharedPtr sphere =
            urdf::dynamic_pointer_cast<const urdf::Sphere>(geom);
        collision_geometry =
            std::make_shared<Sphere<S>>(static_cast<S>(sphere->radius));
      } else
        throw std::invalid_argument("Unknown geometry type :");

      if (!collision_geometry)
        throw std::invalid_argument("The polyhedron retrived is empty");
      auto obj {std::make_shared<CollisionObject<S>>(collision_geometry, pose)};

      collision_objects_.push_back(obj);
      // collision_link_index.push_back(frame_id);
      collision_link_names_.push_back(link->name);
      parent_link_names_.push_back(parent_link_name);
      // collision_joint_index.push_back(model.frames[frame_id].parent);
      /// body_placement * convert_data((*i)->origin);
      collision_origin2link_poses_.push_back(pose_to_transform<S>(col_obj->origin));
      // collision_origin2joint_pose.push_back(
      //         model.frames[frame_id].placement *
      //         convertFromUrdf<S>(geom->origin));
    }
  }
  for (const auto &child : link->child_links) dfs_parse_tree(child, link->name);
}

template <typename S>
void FCLModelTpl<S>::init(const urdf::ModelInterfaceSharedPtr &urdfTree,
                          const std::string &package_dir) {
  package_dir_ = package_dir;
  urdf_model_ = urdfTree;
  if (not urdf_model_)
    throw std::invalid_argument("The XML stream does not contain a valid URDF model.");
  urdf::LinkConstSharedPtr root_link = urdf_model_->getRoot();
  dfs_parse_tree(root_link, "root's parent");
  // setLinkOrder with unique collision link names as user_link_names
  auto user_link_names = collision_link_names_;
  auto last = std::unique(user_link_names.begin(), user_link_names.end());
  user_link_names.erase(last, user_link_names.end());
  setLinkOrder(user_link_names);

  for (size_t i = 0; i < collision_link_names_.size(); i++)
    for (size_t j = 0; j < i; j++)
      if (collision_link_names_[i] != collision_link_names_[j] &&
          parent_link_names_[i] != collision_link_names_[j] &&
          parent_link_names_[j] != collision_link_names_[i]) {
        // We assume that the collisions between objects append to the same
        // joint can be ignored.
        collision_pairs_.push_back(std::make_pair(j, i));
        /*if (verbose)
            std::cout << collision_link_name[j] << " " << collision_link_name[i]
           << std::endl;*/
      }
}

}  // namespace mplib::fcl
