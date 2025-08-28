#include "nlink_protocol.h"

#include <assert.h>

#include <numeric>
#include <string>
#include <chrono>
#include <iostream>

void NLinkProtocol::HandleData(const uint8_t *data) {
  UnpackFrameData(data);
  assert(HandleDataCallback_);
  auto now = std::chrono::system_clock::now(); // 获取当前时间点
  auto duration = now.time_since_epoch(); // 获取时间点与 UNIX 时间原点之间的时间间隔
  const unsigned long int timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(); // 转换为毫秒级的时间戳
  HandleDataCallback_(&timestamp);
}

bool NLinkProtocol::Verify(const uint8_t *data) {
  uint8_t sum = 0;
  return data[length() - 1] ==
         std::accumulate(data, data + length() - sizeof(sum), sum);
}

bool NLinkProtocolVLength::UpdateLength(const uint8_t *data,
                                        size_t available_bytes) {
  if (available_bytes < 4)
    return false;
  return set_length(static_cast<size_t>(data[2] | data[3] << 8));
}
