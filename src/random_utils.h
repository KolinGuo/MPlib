#pragma once

#include <cstdlib>

#include <fcl/math/rng.h>
#include <ompl/util/RandomNumbers.h>

namespace mplib {

template <typename S>
inline void setGlobalSeed(unsigned seed) {
  std::srand(seed);
  ::ompl::RNG::setSeed(seed);
  ::fcl::RNG<S>::setSeed(seed);
}

}  // namespace mplib
