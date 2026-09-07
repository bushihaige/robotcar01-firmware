// OutputSink：记录仲裁后输出请求的测试替身（Iteration 001；仅 host）。
//
// 001 无真实 PWM（PWM/方向域 MotorOutputRequest 归 Iteration 006），
// 故 sink 记录速度域 WheelSpeedRequest 的写入序列供断言。
// header-only；调用方负责 include <vector>。

#ifndef ROBOTCAR01_HOST_FAKES_FAKE_OUTPUT_H_
#define ROBOTCAR01_HOST_FAKES_FAKE_OUTPUT_H_

#include <vector>

#include "app/chassis/chassis_types.h"

namespace robotcar01::host_fakes {

class OutputSink {
 public:
  void Write(const chassis::WheelSpeedRequest& request) {
    writes_.push_back(request);
  }

  const std::vector<chassis::WheelSpeedRequest>& writes() const {
    return writes_;
  }

  void Clear() { writes_.clear(); }

 private:
  std::vector<chassis::WheelSpeedRequest> writes_;
};

}  // namespace robotcar01::host_fakes

#endif  // ROBOTCAR01_HOST_FAKES_FAKE_OUTPUT_H_
