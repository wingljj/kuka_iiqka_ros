#include "soft_force_control_manager/calibration_sequencer.h"

namespace sfm {

bool CalibrationSequencer::active() const {
  return phase_ == CalPhase::MOVING || phase_ == CalPhase::SETTLING ||
         phase_ == CalPhase::SAMPLING || phase_ == CalPhase::RETURNING;
}

void CalibrationSequencer::fail(CalFailure f) {
  if (!active()) return;
  phase_ = CalPhase::FAILED;
  failure_ = f;
  goal_pending_ = false;
}

bool CalibrationSequencer::start(double /*now_s*/) {
  if (active()) return false;
  if (cfg_.poses.empty() || cfg_.samples_per_pose <= 0) return false;
  estimator_.clear();
  fit_ = sfc::PayloadFitResult{};
  failure_ = CalFailure::NONE;
  idx_ = 0;
  sample_count_ = 0;
  accum_ = sfc::Wrench{};
  return_ok_ = true;
  phase_ = CalPhase::MOVING;
  goal_pending_ = true;
  return true;
}

CalAction CalibrationSequencer::tick(double now_s) {
  CalAction act;
  if (phase_ == CalPhase::SETTLING &&
      now_s - settle_start_s_ >= cfg_.settle_time_s) {
    phase_ = CalPhase::SAMPLING;
    sample_count_ = 0;
    accum_ = sfc::Wrench{};
  }
  if (goal_pending_) {
    act.send_goal = true;
    act.target = phase_ == CalPhase::RETURNING ? cfg_.return_pose
                                               : cfg_.poses[idx_];
    goal_pending_ = false;
  }
  return act;
}

void CalibrationSequencer::onMotionResult(bool success, double now_s) {
  if (phase_ == CalPhase::MOVING) {
    if (!success) {
      fail(CalFailure::MOVE_FAILED);
      return;
    }
    phase_ = CalPhase::SETTLING;
    settle_start_s_ = now_s;
  } else if (phase_ == CalPhase::RETURNING) {
    return_ok_ = success;   // a failed return move keeps the solved fit
    phase_ = CalPhase::DONE;
  }
}

void CalibrationSequencer::onWrench(const sfc::Wrench& w) {
  if (phase_ != CalPhase::SAMPLING) return;
  accum_.fx += w.fx;
  accum_.fy += w.fy;
  accum_.fz += w.fz;
  accum_.tx += w.tx;
  accum_.ty += w.ty;
  accum_.tz += w.tz;
  if (++sample_count_ < cfg_.samples_per_pose) return;

  const double n = static_cast<double>(cfg_.samples_per_pose);
  sfc::Wrench mean;
  mean.fx = accum_.fx / n;
  mean.fy = accum_.fy / n;
  mean.fz = accum_.fz / n;
  mean.tx = accum_.tx / n;
  mean.ty = accum_.ty / n;
  mean.tz = accum_.tz / n;
  const CalPose& p = cfg_.poses[idx_];
  estimator_.addSample(p.a, p.b, p.c, mean);

  if (idx_ + 1 < cfg_.poses.size()) {
    ++idx_;
    phase_ = CalPhase::MOVING;
    goal_pending_ = true;
    return;
  }
  fit_ = estimator_.solve();
  if (!fit_.ok) {
    fail(CalFailure::SOLVE_FAILED);
    return;
  }
  phase_ = CalPhase::RETURNING;
  goal_pending_ = true;
}

CalStatus CalibrationSequencer::status() const {
  CalStatus s;
  s.phase = phase_;
  s.failure = failure_;
  s.pose_index = idx_;
  s.pose_count = cfg_.poses.size();
  s.samples_collected = sample_count_;
  s.return_move_ok = return_ok_;
  return s;
}

}  // namespace sfm
