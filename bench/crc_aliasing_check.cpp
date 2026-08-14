// crc_aliasing_check.cpp — gate for the C++ rebuild blocker (2026-08-14)
//
// WHY THIS EXISTS
// ---------------
// humanoid_control could not be rebuilt from shipped source: 3 rebuilds, 3
// failure modes (-O0 slow loop; -O2 "CRC-dead via strict aliasing"; -O2
// -fno-strict-aliasing "misbehaved") — HANDOFF-archive-2026-08-11 trap #2.
// The image therefore still runs the stock Jul-16 binary, which means every
// C++ fix (A-gate, POLICY torque limiting, NaN rejection) is undeployable.
//
// Root cause is in src/src/common/motor_crc_hg.cpp:
//
//     raw.crc = crc32_core((uint32_t *)&raw, (sizeof(LowCmd) >> 2) - 1);
//
// Casting LowCmd* -> uint32_t* and dereferencing violates strict aliasing.
// At -O2 the compiler may assume the struct-field stores and the uint32_t
// loads do not alias, and reorder/elide them: the CRC is computed over a
// partially-written struct, firmware rejects every LowCmd, and the robot
// never leaves ZERO_TORQUE.
//
// This program compares the stock (cast) path against the patched (memcpy)
// path over an identical, fully-populated struct.
//
// BUILD + RUN (both levels — the -O2 run is the one that matters):
//
//     g++ -O0 -Wall -o /tmp/crc_o0 bench/crc_aliasing_check.cpp && /tmp/crc_o0
//     g++ -O2 -Wall -o /tmp/crc_o2 bench/crc_aliasing_check.cpp && /tmp/crc_o2
//
// PASS = "MATCH" at BOTH levels. The patched path is the reference: it is
// well-defined at every optimisation level, so its value at -O0 and at -O2
// must be identical. If the stock path diverges at -O2 on your toolchain,
// that is the bug reproducing.
//
// NOTE ON SCOPE: this uses a stand-in struct with the same shape as LowCmd
// (35 motor commands + reserve + trailing crc word). It demonstrates the
// hazard and validates the fix pattern. It does NOT substitute for the real
// gate, which is: rebuild in the container, then confirm the robot actually
// leaves ZERO_TORQUE on Start with the tether on and NO policy running.

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

struct MotorCmd {
  uint8_t mode;
  float q;
  float dq;
  float tau;
  float Kp;
  float Kd;
  uint32_t reserve;
};

struct LowCmdLike {
  uint8_t modePr;
  uint8_t modeMachine;
  MotorCmd motorCmd[35];
  uint8_t reserve[4];
  uint32_t crc;
};

static_assert(sizeof(LowCmdLike) % sizeof(uint32_t) == 0,
              "stand-in struct must be a whole number of 32-bit words");

uint32_t crc32_core(const uint32_t *ptr, uint32_t len) {
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    uint32_t xbit = 1u << 31;
    uint32_t data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else {
        CRC32 <<= 1;
      }
      if (data & xbit) CRC32 ^= dwPolynomial;
      xbit >>= 1;
    }
  }
  return CRC32;
}

// Fill exactly the way get_crc() does: zero-init, then field-by-field stores.
// The stores are what -O2 is entitled to move across an aliasing-violating read.
void populate(LowCmdLike &raw) {
  raw = LowCmdLike{};
  raw.modePr = 1;
  raw.modeMachine = 5;
  for (int i = 0; i < 35; i++) {
    raw.motorCmd[i].mode = 1;
    raw.motorCmd[i].q = 0.1f * static_cast<float>(i);
    raw.motorCmd[i].dq = 0.0f;
    raw.motorCmd[i].tau = 0.0f;
    raw.motorCmd[i].Kp = 300.0f;
    raw.motorCmd[i].Kd = 5.0f;
    raw.motorCmd[i].reserve = 0;
  }
  const uint8_t res[4] = {0, 0, 0, 0};
  std::memcpy(&raw.reserve[0], &res[0], 4);
}

// STOCK: undefined behaviour — reads the struct through a uint32_t lvalue.
uint32_t crc_stock_aliasing() {
  LowCmdLike raw;
  populate(raw);
  return crc32_core(reinterpret_cast<const uint32_t *>(&raw),
                    (sizeof(LowCmdLike) >> 2) - 1);
}

// PATCHED: well-defined — copies the object representation, then reads words.
uint32_t crc_patched_memcpy() {
  LowCmdLike raw;
  populate(raw);
  constexpr size_t kWordCount = sizeof(LowCmdLike) / sizeof(uint32_t);
  std::array<uint32_t, kWordCount> words{};
  std::memcpy(words.data(), &raw, sizeof(LowCmdLike));
  return crc32_core(words.data(), kWordCount - 1);
}

}  // namespace

int main() {
  const uint32_t stock = crc_stock_aliasing();
  const uint32_t patched = crc_patched_memcpy();

  std::printf("sizeof(LowCmdLike) = %zu bytes (%zu words)\n",
              sizeof(LowCmdLike), sizeof(LowCmdLike) / sizeof(uint32_t));
  std::printf("stock   (uint32_t* cast) : 0x%08X\n", stock);
  std::printf("patched (memcpy)         : 0x%08X\n", patched);

  if (stock == patched) {
    std::printf("RESULT: MATCH — patched CRC is byte-identical to stock here.\n");
    return 0;
  }
  std::printf("RESULT: MISMATCH — stock path miscomputed the CRC.\n");
  std::printf("        This is the rebuild blocker reproducing. The patched\n");
  std::printf("        value is the correct one (it is the defined path).\n");
  return 1;
}
