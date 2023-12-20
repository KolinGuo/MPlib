#pragma once

#include "types.h"

namespace mplib {

template <typename S>
Transform3<S> posevec_to_transform(const Vector7<S> &vec);

// Explicit Template Instantiation Declaration ================================
#define DECLARE_TEMPLATE_MATH_UTILS(S) \
  extern template Transform3<S> posevec_to_transform(const Vector7<S> &vec)

DECLARE_TEMPLATE_MATH_UTILS(float);
DECLARE_TEMPLATE_MATH_UTILS(double);

}  // namespace mplib
