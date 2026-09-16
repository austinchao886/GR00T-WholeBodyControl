#pragma once
#include <algorithm>
#include <stdexcept>
#include "planner_seam_diagnostics.hpp"

namespace sonic_reference {
struct Weight { double value; double rate; };
struct Joint { double q; double dq; };
inline Weight QuinticWeight(double u, double duration_s) {
  if (!sonic_diagnostics::PlannerSeam::Finite(u) ||
      !sonic_diagnostics::PlannerSeam::Finite(duration_s) || duration_s<=0.)
    throw std::invalid_argument("invalid planner blend interval");
  if (u<=0.) return {0.,0.};
  if (u>=1.) return {1.,0.};
  const double value=u*u*u*(10.+u*(-15.+6.*u));
  return {value,30.*u*u*(1.-u)*(1.-u)/duration_s};
}
inline Joint BlendJoint(double old_q, double old_dq, double new_q,
                        double new_dq, Weight w) {
  return {(1.-w.value)*old_q+w.value*new_q,
          (1.-w.value)*old_dq+w.value*new_dq+w.rate*(new_q-old_q)};
}
}
