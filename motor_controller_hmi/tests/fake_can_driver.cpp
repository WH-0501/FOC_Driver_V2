// Test-only DLL. No hardware access; exported ABI matches the legacy ctypes API.
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <windows.h>
#define API extern "C" __declspec(dllexport)
namespace {
constexpr uint64_t device = 0x1122334455667788ull, channel = 0x8877665544332211ull;
std::mutex mutex;
std::array<unsigned char, 76> lastTx{};
std::deque<std::array<unsigned char, 80>> frames;
std::atomic<unsigned> txCount{0}, bad{0};
std::atomic<bool> holdRx{false}, enteredRx{false};
} // namespace
API unsigned __stdcall TestBadCalls() {
    return bad;
}
API unsigned __stdcall TestTxCount() {
    return txCount;
}
API void __stdcall TestLastTx(void *out) {
    std::lock_guard lock(mutex);
    std::memcpy(out, lastTx.data(), 76);
}
API void __stdcall TestHoldRx(unsigned hold) {
    holdRx = hold != 0;
    if (hold)
        enteredRx = false;
}
API unsigned __stdcall TestEnteredRx() {
    return enteredRx;
}
API uint64_t __stdcall ZCAN_OpenDevice(unsigned type, unsigned index, unsigned reserved) {
    txCount = bad = 0;
    enteredRx = holdRx = false;
    if (type != 41 || index != 2 || reserved)
        ++bad;
    std::lock_guard lock(mutex);
    frames.clear();
    return device;
}
API unsigned __stdcall ZCAN_CloseDevice(uint64_t h) {
    if (h != device)
        ++bad;
    return 1;
}
API unsigned __stdcall ZCAN_ResetCAN(uint64_t h) {
    if (h != channel)
        ++bad;
    return 1;
}
API unsigned __stdcall ZCAN_StartCAN(uint64_t h) {
    if (h != channel)
        ++bad;
    return 1;
}
API unsigned __stdcall ZCAN_SetAbitBaud(uint64_t h, unsigned ch, unsigned baud) {
    if (h != device || ch > 1 || baud != 1000000)
        ++bad;
    return 1;
}
API unsigned __stdcall ZCAN_SetDbitBaud(uint64_t h, unsigned ch, unsigned baud) {
    if (h != device || ch > 1 || baud != 5000000)
        ++bad;
    return 1;
}
API unsigned __stdcall ZCAN_SetCANFDStandard(uint64_t h, unsigned ch, unsigned standard) {
    if (h != device || ch > 1 || standard != 0)
        ++bad;
    return 1;
}
API uint64_t __stdcall ZCAN_InitCAN(uint64_t h, unsigned ch, const unsigned char *cfg) {
    const std::array<unsigned char, 32> expected{1, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255};
    if (h != device || ch != 1 || std::memcmp(cfg, expected.data(), 32))
        ++bad;
    return channel;
}
API unsigned __stdcall ZCAN_ClearFilter(uint64_t h) {
    if (h != channel)
        ++bad;
    return 1;
}
API unsigned __stdcall ZCAN_SetFilterMode(uint64_t h, unsigned mode) {
    if (h != channel || mode)
        ++bad;
    return 1;
}
API unsigned __stdcall ZCAN_SetFilterStartID(uint64_t h, unsigned id) {
    if (h != channel || id)
        ++bad;
    return 1;
}
API unsigned __stdcall ZCAN_SetFilterEndID(uint64_t h, unsigned id) {
    if (h != channel || id != 0x7ff)
        ++bad;
    return 1;
}
API unsigned __stdcall ZCAN_AckFilter(uint64_t h) {
    if (h != channel)
        ++bad;
    return 1;
}
API unsigned __stdcall ZCAN_TransmitFD(uint64_t h, const unsigned char *bytes, unsigned count) {
    if (h != channel || count != 1)
        ++bad;
    std::lock_guard lock(mutex);
    std::memcpy(lastTx.data(), bytes, 76);
    ++txCount;
    // An independent ABI producer: ID at 0, length at 4, data at 8, uint64 us at 72.
    std::array<unsigned char, 80> response{};
    response[0] = 1;
    response[4] = 4;
    response[5] = 1;
    response[8] = 7;
    response[9] = 0x7e;
    response[10] = bytes[8];
    response[11] = bytes[9];
    uint64_t us = bytes[8] == 0xee ? 18446743947967255416ull : 10000000000000000ull + txCount * 1000ull;
    std::memcpy(response.data() + 72, &us, 8);
    frames.push_back(response);
    return 1;
}
API unsigned __stdcall ZCAN_GetReceiveNum(uint64_t h, unsigned type) {
    if (h != channel || type > 1)
        ++bad;
    if (type == 0)
        return 0;
    if (holdRx)
        return 1;
    std::lock_guard lock(mutex);
    return unsigned(frames.size());
}
API unsigned __stdcall ZCAN_ReceiveFD(uint64_t h, unsigned char *out, unsigned count, int wait) {
    if (h != channel || !count || count > 256 || wait)
        ++bad;
    enteredRx = true;
    while (holdRx)
        Sleep(1);
    std::lock_guard lock(mutex);
    unsigned n = 0;
    while (n < count && !frames.empty()) {
        std::memcpy(out + n * 80, frames.front().data(), 80);
        frames.pop_front();
        ++n;
    }
    return n;
}
API unsigned __stdcall ZCAN_Receive(uint64_t, void *, unsigned, int) {
    ++bad;
    return 0;
}
