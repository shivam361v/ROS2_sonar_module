#ifndef BASE_INTERFACE_HPP
#define BASE_INTERFACE_HPP

namespace nps_uw_multibeam_sonar_converted {

class BaseInterface {
public:
  virtual void initialize() = 0;
  virtual void execute() = 0;
  virtual ~BaseInterface() = default;
};

} // namespace nps_uw_multibeam_sonar_converted

#endif