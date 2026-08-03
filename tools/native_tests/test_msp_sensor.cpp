#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "msp_sensor.h"

static int g_failures = 0;

static void runCase(const std::string &name, const std::function<void()> &body) {
  try {
    body();
    std::cout << "[PASS] " << name << '\n';
  } catch (const std::exception &error) {
    std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
    g_failures++;
  }
}

#define CHECK(expr)                                                          \
  do {                                                                       \
    if (!(expr)) throw std::runtime_error(#expr);                            \
  } while (0)

#define CHECK_EQ(a, b)                                                       \
  do {                                                                       \
    const auto lhs_ = (a);                                                   \
    const auto rhs_ = (b);                                                   \
    if (!(lhs_ == rhs_))                                                     \
      throw std::runtime_error(std::string(#a " == " #b) + " (got " +        \
                               std::to_string(lhs_) + ")");                  \
  } while (0)

// 테스트가 펌웨어 파서와 **독립적으로** 프레임을 만든다. 인코더와 디코더가
// 같은 실수를 하면 통과해버리므로, CRC 도 여기서 다시 구현한다.
static uint8_t refCrc(uint8_t crc, uint8_t v) {
  crc ^= v;
  for (int i = 0; i < 8; i++)
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  return crc;
}

static std::vector<uint8_t> buildFrame(uint16_t cmd,
                                       const std::vector<uint8_t> &payload,
                                       char dir = '<', uint8_t flags = 0,
                                       int crc_offset = 0) {
  std::vector<uint8_t> f{'$', 'X', (uint8_t)dir};
  const uint16_t size = (uint16_t)payload.size();
  const uint8_t head[5] = {flags, (uint8_t)(cmd & 0xFF), (uint8_t)(cmd >> 8),
                           (uint8_t)(size & 0xFF), (uint8_t)(size >> 8)};
  uint8_t crc = 0;
  for (uint8_t b : head) { f.push_back(b); crc = refCrc(crc, b); }
  for (uint8_t b : payload) { f.push_back(b); crc = refCrc(crc, b); }
  f.push_back((uint8_t)(crc + crc_offset));
  return f;
}

static std::vector<uint8_t> i32le(int32_t v) {
  const uint32_t u = (uint32_t)v;
  return {(uint8_t)(u & 0xFF), (uint8_t)((u >> 8) & 0xFF),
          (uint8_t)((u >> 16) & 0xFF), (uint8_t)((u >> 24) & 0xFF)};
}

static std::vector<uint8_t> rangePayload(uint8_t q, int32_t mm) {
  std::vector<uint8_t> p{q};
  for (uint8_t b : i32le(mm)) p.push_back(b);
  return p;
}

static std::vector<uint8_t> flowPayload(uint8_t q, int32_t x, int32_t y) {
  std::vector<uint8_t> p{q};
  for (uint8_t b : i32le(x)) p.push_back(b);
  for (uint8_t b : i32le(y)) p.push_back(b);
  return p;
}

// 바이트를 하나씩 먹이고 마지막으로 나온 결과를 돌려준다.
static MspParseResult feed(MspSensorParser &p,
                           const std::vector<uint8_t> &bytes) {
  MspParseResult last = MSP_PARSE_NONE;
  for (uint8_t b : bytes) {
    const MspParseResult r = mspSensorParseByte(p, b);
    if (r != MSP_PARSE_NONE) last = r;
  }
  return last;
}

int main() {
  runCase("거리계 프레임을 해석한다", [] {
    MspSensorParser p;
    const auto f = buildFrame(MSP2_SENSOR_RANGEFINDER, rangePayload(180, 1234));
    CHECK(feed(p, f) == MSP_PARSE_RANGEFINDER);
    CHECK_EQ((int)p.range.quality, 180);
    CHECK_EQ(p.range.distance_mm, 1234);
    CHECK_EQ(p.frames_ok, 1u);
    CHECK_EQ(p.crc_errors, 0u);
  });

  runCase("범위 밖 음수 거리를 부호 그대로 보존한다", [] {
    // 이 모듈은 8cm~200cm 밖이면 음수를 보낸다. 너무 가까울 때도 음수라
    // 부호만으로 위/아래를 구분할 수 없다 — 값을 훼손하지 않는 것이 중요하다.
    MspSensorParser p;
    CHECK(feed(p, buildFrame(MSP2_SENSOR_RANGEFINDER, rangePayload(0, -1))) ==
          MSP_PARSE_RANGEFINDER);
    CHECK_EQ(p.range.distance_mm, -1);
    CHECK(feed(p, buildFrame(MSP2_SENSOR_RANGEFINDER,
                             rangePayload(3, -2147483647))) ==
          MSP_PARSE_RANGEFINDER);
    CHECK_EQ(p.range.distance_mm, -2147483647);
  });

  runCase("광류 프레임의 두 축을 각각 해석한다", [] {
    MspSensorParser p;
    CHECK(feed(p, buildFrame(MSP2_SENSOR_OPTIC_FLOW,
                             flowPayload(77, -30000, 45678))) ==
          MSP_PARSE_OPTIC_FLOW);
    CHECK_EQ((int)p.flow.quality, 77);
    CHECK_EQ(p.flow.motion_x, -30000);
    CHECK_EQ(p.flow.motion_y, 45678);
  });

  runCase("CRC가 틀리면 거부하고 카운트한다", [] {
    MspSensorParser p;
    const auto f =
        buildFrame(MSP2_SENSOR_RANGEFINDER, rangePayload(9, 500), '<', 0, 1);
    CHECK(feed(p, f) == MSP_PARSE_CRC_ERROR);
    CHECK_EQ(p.crc_errors, 1u);
    CHECK_EQ(p.frames_ok, 0u);
    CHECK_EQ(p.range.distance_mm, 0);   // 값이 오염되지 않아야 한다
  });

  runCase("모르는 function은 건너뛰고 마지막 ID를 남긴다", [] {
    // function 상수를 잘못 넣었을 때 "값이 안 나온다"가 아니라 이 카운터가
    // 올라가는 것으로 드러나야 한다.
    MspSensorParser p;
    CHECK(feed(p, buildFrame(0x1F05, {1, 2, 3})) == MSP_PARSE_UNKNOWN);
    CHECK_EQ(p.unknown_frames, 1u);
    CHECK_EQ((int)p.last_unknown_cmd, 0x1F05);
  });

  runCase("아는 function이라도 길이가 다르면 unknown으로 센다", [] {
    MspSensorParser p;
    CHECK(feed(p, buildFrame(MSP2_SENSOR_RANGEFINDER, {1, 2, 3})) ==
          MSP_PARSE_UNKNOWN);
    CHECK_EQ(p.unknown_frames, 1u);
    CHECK_EQ(p.range.distance_mm, 0);   // 잘못 해석하지 않는다
  });

  runCase("버퍼보다 큰 페이로드는 소비만 하고 동기를 유지한다", [] {
    MspSensorParser p;
    std::vector<uint8_t> big(MSP_SENSOR_MAX_PAYLOAD + 8, 0xAB);
    CHECK(feed(p, buildFrame(0x2000, big)) == MSP_PARSE_UNKNOWN);
    CHECK_EQ(p.oversize_frames, 1u);
    // 바로 다음 정상 프레임이 읽혀야 동기가 유지된 것이다.
    CHECK(feed(p, buildFrame(MSP2_SENSOR_RANGEFINDER, rangePayload(5, 77))) ==
          MSP_PARSE_RANGEFINDER);
    CHECK_EQ(p.range.distance_mm, 77);
  });

  runCase("앞에 쓰레기가 있어도 재동기화한다", [] {
    MspSensorParser p;
    std::vector<uint8_t> s{0x00, 0xFF, 'X', '$', 'Z', 0x11, '$', 'X'};
    const auto f = buildFrame(MSP2_SENSOR_RANGEFINDER, rangePayload(20, 850));
    s.insert(s.end(), f.begin(), f.end());
    CHECK(feed(p, s) == MSP_PARSE_RANGEFINDER);
    CHECK_EQ(p.range.distance_mm, 850);
  });

  runCase("헤더 도중의 '$'는 새 프레임 시작으로 받아들인다", [] {
    // 'X' 가 와야 할 자리에 '$' 가 오면 그 바이트를 버리면 안 된다.
    MspSensorParser p;
    std::vector<uint8_t> s{'$', '$'};
    const auto f = buildFrame(MSP2_SENSOR_RANGEFINDER, rangePayload(1, 99));
    s.insert(s.end(), f.begin() + 1, f.end());   // 이미 '$' 를 준 셈
    CHECK(feed(p, s) == MSP_PARSE_RANGEFINDER);
    CHECK_EQ(p.range.distance_mm, 99);
  });

  runCase("프레임이 여러 read 에 걸쳐 나뉘어도 재조립한다", [] {
    MspSensorParser p;
    const auto f = buildFrame(MSP2_SENSOR_OPTIC_FLOW, flowPayload(11, 7, -8));
    for (std::size_t i = 0; i + 1 < f.size(); i++) {
      CHECK(mspSensorParseByte(p, f[i]) == MSP_PARSE_NONE);
    }
    CHECK(mspSensorParseByte(p, f.back()) == MSP_PARSE_OPTIC_FLOW);
    CHECK_EQ(p.flow.motion_x, 7);
    CHECK_EQ(p.flow.motion_y, -8);
  });

  runCase("방향 바이트 세 가지를 모두 받는다", [] {
    for (char dir : {'<', '>', '!'}) {
      MspSensorParser p;
      CHECK(feed(p, buildFrame(MSP2_SENSOR_RANGEFINDER, rangePayload(2, 640),
                               dir)) == MSP_PARSE_RANGEFINDER);
      CHECK_EQ(p.range.distance_mm, 640);
    }
  });

  runCase("CRC 는 방향 바이트를 포함하지 않는다", [] {
    // 방향만 다르고 나머지가 같은 두 프레임은 같은 CRC 를 가져야 한다.
    const auto a = buildFrame(MSP2_SENSOR_RANGEFINDER, rangePayload(2, 640), '<');
    const auto b = buildFrame(MSP2_SENSOR_RANGEFINDER, rangePayload(2, 640), '>');
    CHECK_EQ(a.back(), b.back());
  });

  if (g_failures != 0) {
    std::cerr << g_failures << " msp-sensor case(s) failed\n";
    return 1;
  }
  std::cout << "all msp-sensor cases passed\n";
  return 0;
}
