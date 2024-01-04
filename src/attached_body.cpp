#include "attached_body.h"

namespace mplib {

// Explicit Template Instantiation Definition =================================
#define DEFINE_TEMPLATE_ATTACHED_BODY(S) template class AttachedBodyTpl<S>

DEFINE_TEMPLATE_ATTACHED_BODY(float);
DEFINE_TEMPLATE_ATTACHED_BODY(double);

template <typename S>
AttachedBodyTpl<S>::AttachedBodyTpl(const std::string &name,
                                    const CollisionObjectPtr &object,
                                    const ArticulatedModelPtr &attached_articulation,
                                    int attached_link_id, const Transform3<S> &pose,
                                    const std::vector<std::string> &touch_links)
    : name_(name),
      object_(object),
      attached_articulation_(attached_articulation),
      pinocchio_model_(attached_articulation->getPinocchioModel()),
      attached_link_id_(attached_link_id),
      pose_(pose),
      touch_links_(touch_links) {
  updatePose();  // updates global pose using link_pose and attached_pose
}

}  // namespace mplib
