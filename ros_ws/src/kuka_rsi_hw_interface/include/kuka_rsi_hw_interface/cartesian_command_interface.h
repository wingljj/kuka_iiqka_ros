#pragma once

#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

#include <string>

namespace kuka_rsi {

// Read-only handle to the KUKA Cartesian TCP pose (RIst). Units: mm / deg
// (KUKA A/B/C = Z-Y-X Euler). Registered under a resource name, by
// convention "kuka_tcp".
class CartesianStateHandle {
 public:
  CartesianStateHandle() = default;
  CartesianStateHandle(const std::string& name, const double* x,
                       const double* y, const double* z, const double* a,
                       const double* b, const double* c)
      : name_(name), x_(x), y_(y), z_(z), a_(a), b_(b), c_(c) {
    if (!x || !y || !z || !a || !b || !c) {
      throw hardware_interface::HardwareInterfaceException(
          "Cannot create CartesianStateHandle '" + name +
          "': a data pointer is null.");
    }
  }

  std::string getName() const { return name_; }
  double getX() const { return *x_; }
  double getY() const { return *y_; }
  double getZ() const { return *z_; }
  double getA() const { return *a_; }
  double getB() const { return *b_; }
  double getC() const { return *c_; }

 private:
  std::string name_;
  const double* x_{nullptr};
  const double* y_{nullptr};
  const double* z_{nullptr};
  const double* a_{nullptr};
  const double* b_{nullptr};
  const double* c_{nullptr};
};

// Read-write handle: state plus the per-cycle RKorr command (mm/deg per
// 4 ms cycle). Claimed exclusively by one controller at a time (spec 5.3).
class CartesianCorrectionHandle : public CartesianStateHandle {
 public:
  CartesianCorrectionHandle() = default;
  CartesianCorrectionHandle(const CartesianStateHandle& state, double* cx,
                            double* cy, double* cz, double* ca, double* cb,
                            double* cc)
      : CartesianStateHandle(state),
        cx_(cx), cy_(cy), cz_(cz), ca_(ca), cb_(cb), cc_(cc) {
    if (!cx || !cy || !cz || !ca || !cb || !cc) {
      throw hardware_interface::HardwareInterfaceException(
          "Cannot create CartesianCorrectionHandle '" + state.getName() +
          "': a command pointer is null.");
    }
  }

  void setCommand(double x, double y, double z, double a, double b,
                  double c) {
    *cx_ = x;
    *cy_ = y;
    *cz_ = z;
    *ca_ = a;
    *cb_ = b;
    *cc_ = c;
  }
  double getCommandX() const { return *cx_; }
  double getCommandY() const { return *cy_; }
  double getCommandZ() const { return *cz_; }
  double getCommandA() const { return *ca_; }
  double getCommandB() const { return *cb_; }
  double getCommandC() const { return *cc_; }

 private:
  double* cx_{nullptr};
  double* cy_{nullptr};
  double* cz_{nullptr};
  double* ca_{nullptr};
  double* cb_{nullptr};
  double* cc_{nullptr};
};

// Read-only interface: never claims resources.
class CartesianStateInterface
    : public hardware_interface::HardwareResourceManager<
          CartesianStateHandle> {
 public:
  // The base HardwareInterface::claim() unconditionally records a claim;
  // override it as a no-op so a read-only interface never appears in the
  // claim list, regardless of how claim() is invoked.
  void claim(std::string /*resource*/) override {}
};

// Command interface: exclusive claim, enforced by controller_manager.
class CartesianCorrectionCommandInterface
    : public hardware_interface::HardwareResourceManager<
          CartesianCorrectionHandle, hardware_interface::ClaimResources> {};

}  // namespace kuka_rsi
