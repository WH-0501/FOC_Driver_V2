#pragma once
namespace mt::sweepPolicy {
// Current-loop acquisition policy, owned by the host. The device executes
// the explicit range and period list over the existing AE/2 and AD/2 protocol.
// A future velocity/position loop must supply its own baseline and limits.
inline constexpr double minHz=20, maxHz=2000;
inline constexpr double baselineStartHz=20;
inline constexpr unsigned minPoints=3, maxPoints=16;
// Shared by the acquisition planner and the measured-platform validator.
// Planning coverage alone does not qualify a measured baseline.
inline constexpr unsigned baselineMinPoints=3;
inline constexpr double baselineMaxRatio=3, baselineMinRatio=1.5;
inline constexpr double maxAdjacentRatio=2;
inline constexpr double flatDb=.5, phaseLimitDeg=15, unityToleranceDb=1;
static_assert(baselineStartHz>=minHz && baselineStartHz<maxHz);
}
