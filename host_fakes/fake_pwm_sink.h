// FakePwmSink：记录每次 Apply（Iteration 006，header-only，仅 host）。
// 用于断言"PWM 只经仲裁写入"（Apply 次数 == Tick 次数）与逐周期方向/占空比轨迹。

#ifndef ROBOTCAR01_HOST_FAKES_FAKE_PWM_SINK_H_
#define ROBOTCAR01_HOST_FAKES_FAKE_PWM_SINK_H_

#include <cstdint>
#include <vector>

#include "app/control/motor_output.h"

namespace robotcar01::host_fakes {

class FakePwmSink : public control::PwmSink {
 public:
  struct Record {
    control::MotorOutputRequest left;
    control::MotorOutputRequest right;
  };

  control::PwmApplyStatus Apply(const control::MotorOutputRequest& left,
                                const control::MotorOutputRequest& right) override {
    records_.push_back(Record{left, right});
    control::PwmApplyStatus status{};
    status.ok = !fail_next_;
    status.platform_error = fail_next_ ? 1u : 0u;
    fail_next_ = false;
    return status;
  }

  void FailNext() { fail_next_ = true; }
  const std::vector<Record>& records() const { return records_; }
  void Clear() { records_.clear(); }

 private:
  std::vector<Record> records_;
  bool fail_next_ = false;
};

}  // namespace robotcar01::host_fakes

#endif  // ROBOTCAR01_HOST_FAKES_FAKE_PWM_SINK_H_
