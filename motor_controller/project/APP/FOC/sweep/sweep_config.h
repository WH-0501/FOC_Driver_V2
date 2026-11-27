#ifndef FOC_SWEEP_CONFIG_H
#define FOC_SWEEP_CONFIG_H

/* Device-owned Chirp timing, advertised via AE/2 before acquisition.
 * Current configuration: 4 s x (1 settling + 3 measurement periods) = 4 s. */
#define SWEEP_CHIRP_PERIOD_MS             1000u ///< 单周期时长 1 s，由 AE/2 上报
#define SWEEP_CHIRP_SETTLE_PERIODS        1u ///< 舍弃的稳定周期数
#define SWEEP_CHIRP_MEASURE_PERIODS       3u ///< 参与 H1 的测量周期数
#define SWEEP_CHIRP_TOTAL_PERIODS \
    (SWEEP_CHIRP_SETTLE_PERIODS + SWEEP_CHIRP_MEASURE_PERIODS) ///< 稳定 + 测量
#define SWEEP_CHIRP_EDGE_MS               5u ///< chirp 扫频 edge 周期 5ms
#define SWEEP_CHIRP_PEAK_HEADROOM         1.001f ///< chirp 扫频 peak 1.001

/** 逐点正弦扫频设置 */
#define SWEEP_STEPPED_SETTLE_MS           20u ///< stepped 扫频 settling 周期 20ms
#define SWEEP_STEPPED_MIN_SETTLE_CYCLES   4u ///< stepped 扫频 min settle cycles 4
#define SWEEP_STEPPED_BLOCKS              4u ///< stepped 扫频 blocks 4
#define SWEEP_STEPPED_CYCLES_PER_BLOCK    4u ///< stepped 扫频 cycles per block 4
#define SWEEP_STEPPED_MEASURE_CYCLES \
    (SWEEP_STEPPED_BLOCKS * SWEEP_STEPPED_CYCLES_PER_BLOCK) ///< stepped 扫频 采样周期 16 个周期

/* Admission limits; the explicit host-planned range is used without anchors.
 * Full/local measurement coverage and baseline selection belong to the host;
 * do not silently extend this range or duplicate its baseline configuration.
 * These describe the existing
 * wire/host contract, not substitutes for motor current/motion protections. */
#define SWEEP_MAX_POINTS                 16u ///< 扫频点数最大值 16
#define SWEEP_MIN_POINTS                 3u ///< 扫频点数最小值 3
#define SWEEP_MIN_HZ                     20u ///< 扫频频率最小值 20Hz
#define SWEEP_MAX_HZ                     2000u ///< 扫频频率最大值 2000Hz
#define SWEEP_MIN_SAMPLE_HZ              10000u
#define SWEEP_MAX_SAMPLE_HZ              40000u ///< 扫频采样率最大值 40000Hz
#define SWEEP_MIN_SAMPLES_PER_CYCLE      10u
#define SWEEP_MIN_PEAK_A                 0.005f ///< 扫频峰值最小值 0.005A
#define SWEEP_MAX_PEAK_A                 0.5f ///< 扫频峰值最大值 0.5A

/* AE/2 status tail reports Chirp timing. Compatible host required; the
 * stepped window is still fixed. Raw sample frames and protection are unchanged.
 * Whole-second periods preserve the existing integer-Hz analysis grid.
 * Keep the entire capture within the host's existing 1,200,000-sample bound. */
#if SWEEP_STEPPED_SETTLE_MS != 20u || SWEEP_STEPPED_MIN_SETTLE_CYCLES != 4u || \
    SWEEP_STEPPED_BLOCKS != 4u || SWEEP_STEPPED_CYCLES_PER_BLOCK != 4u
#error Stepped timing differs from the host window
#endif
#if SWEEP_CHIRP_PERIOD_MS < 1000u || SWEEP_CHIRP_PERIOD_MS > 60000u || \
    (SWEEP_CHIRP_PERIOD_MS % 1000u) != 0u
#error Chirp period must be whole seconds from 1000 to 60000 ms
#endif
#if (1ULL * SWEEP_MAX_SAMPLE_HZ * SWEEP_CHIRP_PERIOD_MS / 1000u * SWEEP_CHIRP_TOTAL_PERIODS) > 1200000u
#error Chirp capture exceeds host sample capacity at maximum sample rate
#endif
#if SWEEP_CHIRP_SETTLE_PERIODS < 1u
#error Chirp requires at least one settling period
#endif
#if SWEEP_CHIRP_EDGE_MS == 0u || 2u * SWEEP_CHIRP_EDGE_MS >= SWEEP_CHIRP_PERIOD_MS
#error Chirp edges must be positive and shorter than half a period
#endif
#if SWEEP_CHIRP_MEASURE_PERIODS < 2u || SWEEP_CHIRP_TOTAL_PERIODS > 255u
#error Chirp needs multiple measurement periods and an 8-bit period count
#endif
#if SWEEP_STEPPED_BLOCKS < 2u || SWEEP_STEPPED_CYCLES_PER_BLOCK == 0u || \
    SWEEP_STEPPED_MIN_SETTLE_CYCLES == 0u
#error Invalid stepped acquisition layout
#endif
#if ((SWEEP_MAX_HZ * SWEEP_STEPPED_SETTLE_MS + 999u) / 1000u + 1u + \
     SWEEP_STEPPED_MEASURE_CYCLES) > 255u || \
    (SWEEP_STEPPED_MIN_SETTLE_CYCLES + SWEEP_STEPPED_MEASURE_CYCLES) > 255u
#error Stepped settling and measurement exceed the 8-bit cycle counter
#endif

/* Commissioning protections, at the MOTOR SHAFT. Values are unchanged.
 * A configuration refactor is not authorization to relax these limits. */
#ifndef FOC_SWEEP_ENABLED
#define FOC_SWEEP_ENABLED                 1
#endif
/* Sweep-only displacement abort, independent of the speed guard.
 * 0 skips only displacement from the sweep origin; finite-value checks,
 * starting-speed admission and all other protections are unchanged. */
#ifndef FOC_SWEEP_TRAVEL_GUARD_ENABLED
#define FOC_SWEEP_TRAVEL_GUARD_ENABLED    0
#endif
#if FOC_SWEEP_TRAVEL_GUARD_ENABLED != 0 && FOC_SWEEP_TRAVEL_GUARD_ENABLED != 1
#error FOC_SWEEP_TRAVEL_GUARD_ENABLED must be 0 or 1
#endif
#ifndef FOC_SWEEP_MAX_TRAVEL_RAD
#define FOC_SWEEP_MAX_TRAVEL_RAD          0.5f
#endif
/* Sweep-only running-speed abort. 0 disables only this comparison:
 * finite checks, starting-speed admission, independent travel guard, motor speed limits,
 * current/temperature/timing and recording protections remain active. */
#ifndef FOC_SWEEP_SPEED_GUARD_ENABLED
#define FOC_SWEEP_SPEED_GUARD_ENABLED     0
#endif
#if FOC_SWEEP_SPEED_GUARD_ENABLED != 0 && FOC_SWEEP_SPEED_GUARD_ENABLED != 1
#error FOC_SWEEP_SPEED_GUARD_ENABLED must be 0 or 1
#endif
#ifndef FOC_SWEEP_MAX_SPEED_RAD_S
#define FOC_SWEEP_MAX_SPEED_RAD_S         15.0f
#endif
#ifndef FOC_SWEEP_MAX_TEMP_C
#define FOC_SWEEP_MAX_TEMP_C              60.0f
#endif
#define FOC_SWEEP_START_SPEED_RAD_S      1.0f
#define FOC_SWEEP_CURRENT_LIMIT_FRACTION 0.8f
#define FOC_SWEEP_INTERVAL_MIN_RATIO     0.8f
#define FOC_SWEEP_INTERVAL_MAX_RATIO     1.2f
#define FOC_SWEEP_EXECUTION_MAX_RATIO    0.9f

/* Queue capacity only; wire v1 still packs six binary32 pairs per frame. */
#ifndef SWEEP_RECORD_QUEUE_FRAMES
#define SWEEP_RECORD_QUEUE_FRAMES        16u
#endif
#if SWEEP_RECORD_QUEUE_FRAMES < 2u || SWEEP_RECORD_QUEUE_FRAMES > 128u || \
    (SWEEP_RECORD_QUEUE_FRAMES & (SWEEP_RECORD_QUEUE_FRAMES - 1u))
#error Recording queue must be a power of two from 2 to 128 frames
#endif
#endif
