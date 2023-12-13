#pragma once
#include <urdf_parser/urdf_parser.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/joint.hpp>
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/utilities/svd_eigen_HH.hpp>

#include "macros_utils.h"
#include "urdf_utils.h"

template <typename DATATYPE>
class KDLModelTpl {
 private:
  DEFINE_TEMPLATE_EIGEN(DATATYPE)
  KDL::Tree tree_;
  std::string tree_root_name_;
  bool verbose_;
  std::vector<std::string> user_link_names_;
  std::vector<std::string> user_joint_names_;
  std::vector<int> joint_mapping_kdl_2_user_;
  std::map<std::string, int> user_joint_idx_mapping_;

 public:
  KDLModelTpl(std::string const &urdf_filename,
              std::vector<std::string> const &joint_names,
              std::vector<std::string> const &link_names, bool const &verbose);
  std::string getTreeRootName() { return tree_root_name_; }
  std::tuple<VectorX, int> chainIKLMA(size_t const &index, VectorX const &q0,
                                      Vector7 const &pose);
  std::tuple<VectorX, int> chainIKNR(size_t const &index, VectorX const &q0,
                                     Vector7 const &pose);
  std::tuple<VectorX, int> chainIKNRJL(size_t const &index, VectorX const &q0,
                                       Vector7 const &pose,
                                       VectorX const &q_min,
                                       VectorX const &q_max);
  std::tuple<VectorX, int> TreeIKNRJL(const std::vector<std::string> endpoints,
                                      VectorX const &q0,
                                      std::vector<Vector7> const &poses,
                                      VectorX const &q_min,
                                      VectorX const &q_max);
};

template <typename T>
using KDLModelTplPtr = std::shared_ptr<KDLModelTpl<T>>;

using KDLModeld = KDLModelTpl<double>;
using KDLModelf = KDLModelTpl<float>;
using KDLModeldPtr = KDLModelTplPtr<double>;
using KDLModelfPtr = KDLModelTplPtr<float>;
