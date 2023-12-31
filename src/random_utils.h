#pragma once

#include <ompl/util/RandomNumbers.h>
#include <fcl/math/rng.h>

#include <cstdlib>

namespace mplib {

template <typename S>
inline void setGlobalSeed(unsigned seed) {
  std::srand(seed);
  ::ompl::RNG::setSeed(seed);
  ::fcl::RNG<S>::setSeed(seed);
}

}  // namespace mplib
