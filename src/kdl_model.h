#pragma once

#include <kdl/tree.hpp>

#include "macros_utils.h"
#include "types.h"

namespace mplib {

// KDLModelTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(KDLModelTpl);

template <typename S>
class KDLModelTpl {
 public:
  KDLModelTpl(std::string const &urdf_filename,
              std::vector<std::string> const &joint_names,
              std::vector<std::string> const &link_names, bool verbose);

  const std::string &getTreeRootName() const { return tree_root_name_; }

  std::tuple<VectorX<S>, int> chainIKLMA(size_t const &index,
                                         VectorX<S> const &q0,
                                         Vector7<S> const &pose) const;

  std::tuple<VectorX<S>, int> chainIKNR(size_t const &index,
                                        VectorX<S> const &q0,
                                        Vector7<S> const &pose) const;

  std::tuple<VectorX<S>, int> chainIKNRJL(size_t const &index,
                                          VectorX<S> const &q0,
                                          Vector7<S> const &pose,
                                          VectorX<S> const &q_min,
                                          VectorX<S> const &q_max) const;

  std::tuple<VectorX<S>, int> TreeIKNRJL(
      const std::vector<std::string> endpoints, VectorX<S> const &q0,
      std::vector<Vector7<S>> const &poses, VectorX<S> const &q_min,
      VectorX<S> const &q_max) const;

 private:
  KDL::Tree tree_;
  std::string tree_root_name_;
  std::vector<std::string> user_link_names_;
  std::vector<std::string> user_joint_names_;
  std::vector<int> joint_mapping_kdl_2_user_;
  std::map<std::string, int> user_joint_idx_mapping_;
  bool verbose_;
};

// Common Type Alias ==========================================================
using KDLModelf = KDLModelTpl<float>;
using KDLModeld = KDLModelTpl<double>;
using KDLModelfPtr = KDLModelTplPtr<float>;
using KDLModeldPtr = KDLModelTplPtr<double>;

// Explicit Template Instantiation Declaration ================================
#define DECLARE_TEMPLATE_KDL_MODEL(S) extern template class KDLModelTpl<S>

DECLARE_TEMPLATE_KDL_MODEL(float);
DECLARE_TEMPLATE_KDL_MODEL(double);

}  // namespace mplib
