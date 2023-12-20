#include "attached_body.h"

namespace mplib {

// Explicit Template Instantiation Definition =================================
#define DEFINE_TEMPLATE_ATTACHED_BODY(S) template class AttachedBodyTpl<S>

DEFINE_TEMPLATE_ATTACHED_BODY(float);
DEFINE_TEMPLATE_ATTACHED_BODY(double);

template <typename S>
AttachedBodyTpl<S>::AttachedBodyTpl(
    std::string const &name, CollisionObjectPtr const &object,
    ArticulatedModelPtr const &attached_articulation, int attached_link_id,
    Transform3<S> const &pose)
    : name_(name),
      object_(object),
      attached_articulation_(attached_articulation),
      pinocchio_model_(attached_articulation->getPinocchioModel()),
      attached_link_id_(attached_link_id),
      pose_(pose) {
  updatePose();  // updates global pose using link_pose and attached_pose
}

}  // namespace mplib
