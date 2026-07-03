#pragma once

#include <cstddef>
#include <vector>

#include "soft_force_control_core/payload_estimator.h"
#include "soft_force_control_core/types.h"

namespace sfm {

struct CalPose {
  double a{0}, b{0}, c{0};  // deg (KUKA A/B/C = Z-Y-X Euler)
};

struct CalibrationConfig {
  std::vector<CalPose> poses;   // calibration.yaml sequence (legacy 8-set)
  CalPose return_pose;          // spec 9 step 5; default A=B=C=0
  double settle_time_s{1.0};
  int samples_per_pose{100};
};

enum class CalPhase { IDLE, MOVING, SETTLING, SAMPLING, RETURNING, DONE,
                      FAILED };
enum class CalFailure { NONE, MOVE_FAILED, SOLVE_FAILED, STREAM_LOST,
                        CANCELLED };

// tick() output: at most one goal emission per pose (decision 5: the
// estimator uses the goal pose, not a robot readback).
struct CalAction {
  bool send_goal{false};
  CalPose target;
};

struct CalStatus {
  CalPhase phase{CalPhase::IDLE};
  CalFailure failure{CalFailure::NONE};
  std::size_t pose_index{0};
  std::size_t pose_count{0};
  int samples_collected{0};
  bool return_move_ok{true};
};

// Spec-9 calibration workflow as a pure event-driven sequencer: the
// runtime feeds motion results, wrench samples, and stream-loss events;
// tick() advances settle timing and emits goals. A failure never emits
// further goals (decision 6: no return motion on abort); a failed return
// move does NOT discard the fit (the estimate is already solved). No ROS,
// no threads; the runtime serializes access.
class CalibrationSequencer {
 public:
  void configure(const CalibrationConfig& cfg) { cfg_ = cfg; }
  bool start(double now_s);
  void cancel() { fail(CalFailure::CANCELLED); }
  void onMotionResult(bool success, double now_s);
  void onWrench(const sfc::Wrench& w);
  void onStreamLost() { fail(CalFailure::STREAM_LOST); }
  CalAction tick(double now_s);
  CalStatus status() const;
  const sfc::PayloadFitResult& result() const { return fit_; }

 private:
  bool active() const;
  void fail(CalFailure f);
  CalibrationConfig cfg_;
  CalPhase phase_{CalPhase::IDLE};
  CalFailure failure_{CalFailure::NONE};
  std::size_t idx_{0};
  bool goal_pending_{false};
  double settle_start_s_{0};
  int sample_count_{0};
  sfc::Wrench accum_;
  bool return_ok_{true};
  sfc::PayloadEstimator estimator_;
  sfc::PayloadFitResult fit_;
};

}  // namespace sfm
