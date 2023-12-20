#include "math_utils.h"

namespace mplib {

// Explicit Template Instantiation Definition =================================
#define DEFINE_TEMPLATE_MATH_UTILS(S) \
  template Transform3<S> posevec_to_transform(const Vector7<S> &vec)

DEFINE_TEMPLATE_MATH_UTILS(float);
DEFINE_TEMPLATE_MATH_UTILS(double);

template <typename S>
Transform3<S> posevec_to_transform(const Vector7<S> &vec) {
  Transform3<S> pose;
  pose.linear() = Quaternion<S>(vec[3], vec[4], vec[5], vec[6]).matrix();
  pose.translation() = vec.head(3);
  return pose;
}

}  // namespace mplib
