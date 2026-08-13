#include "motor_crc_hg.h"

#include <array>   // LOCAL PATCH (2026-08-14)
#include <cstring> // LOCAL PATCH (2026-08-14)

void get_crc(unitree_hg::msg::LowCmd &msg)
{
    LowCmd raw{};

    raw.modePr = msg.mode_pr;
    raw.modeMachine = msg.mode_machine;

    for (int i = 0; i < 35; i++)
    {
        raw.motorCmd[i].mode = msg.motor_cmd[i].mode;
        raw.motorCmd[i].q = msg.motor_cmd[i].q;
        raw.motorCmd[i].dq = msg.motor_cmd[i].dq;
        raw.motorCmd[i].tau = msg.motor_cmd[i].tau;
        raw.motorCmd[i].Kp = msg.motor_cmd[i].kp;
        raw.motorCmd[i].Kd = msg.motor_cmd[i].kd;

        raw.motorCmd[i].reserve = msg.motor_cmd[i].reserve;
    }

    memcpy(&raw.reserve[0], &msg.reserve[0], 4);

    // LOCAL PATCH (2026-08-14): THIS IS THE C++ REBUILD BLOCKER.
    //
    // Stock did:  crc32_core((uint32_t *)&raw, (sizeof(LowCmd) >> 2) - 1);
    //
    // Casting `LowCmd*` to `uint32_t*` and dereferencing it violates strict
    // aliasing. At -O2 the compiler is entitled to assume the `raw.motorCmd[i]`
    // stores above cannot alias the `ptr[i]` loads inside crc32_core, so it may
    // reorder or elide them -- the CRC is then computed over a partially
    // written struct. Firmware rejects every LowCmd and the robot never leaves
    // ZERO_TORQUE. That is exactly the 3-for-3 rebuild failure recorded in
    // HANDOFF-archive-2026-08-11 (-O0 = slow loop; -O2 = "CRC-dead via strict
    // aliasing"; -O2 -fno-strict-aliasing = "misbehaved"), and why the image
    // still runs the stock Jul-16 binary (5,605,520 B) while the Aug-11
    // rebuild (1,147,496 B) is known-bad.
    //
    // memcpy into a real uint32_t array is well-defined at every -O level, so
    // the package can be rebuilt normally -- no -fno-strict-aliasing needed.
    // Byte-for-byte identical CRC input: same length, same trailing word
    // (the crc field itself) excluded by the -1.
    constexpr size_t kWordCount = sizeof(LowCmd) / sizeof(uint32_t);
    static_assert(sizeof(LowCmd) % sizeof(uint32_t) == 0,
                  "LowCmd must be a whole number of 32-bit words");

    std::array<uint32_t, kWordCount> words{};
    std::memcpy(words.data(), &raw, sizeof(LowCmd));

    raw.crc = crc32_core(words.data(), kWordCount - 1);
    msg.crc = raw.crc;
}

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}