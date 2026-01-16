#include "kinematics_interface_ikfast/kinematics_interface_ikfast.hpp"

#ifdef IKFAST_NO_MAIN
#undef IKFAST_NO_MAIN
#endif
#define IKFAST_NO_MAIN

#ifdef IKFAST_HAS_LIBRARY
#undef IKFAST_HAS_LIBRARY
#endif
#define IKFAST_HAS_LIBRARY

// Include the ikfast file for the dummy robot
#include "ikfast_dummy.cpp"

namespace dummy_ikfast
{
class DummyKinematics : public kinematics_interface_ikfast::KinematicsInterfaceIKFast
{
public:
  int get_num_joints_internal() override {
    return GetNumJoints();
}

  void do_compute_fk(const double* j, double* etrans, double* erot) override {
    ComputeFk(j, etrans, erot);
  }

  void do_compute_ik(const double* etrans, const double* erot, const double* free, void* solutions) override {
    auto* sol_list = static_cast<ikfast::IkSolutionListBase<double>*>(solutions);
    ComputeIk(etrans, erot, free, *sol_list);
  }
};
} // namespace dummy_ikfast

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dummy_ikfast::DummyKinematics, kinematics_interface::KinematicsInterface)