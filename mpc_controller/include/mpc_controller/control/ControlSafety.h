// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <Eigen/Core>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <type_traits>
#include <vector>

namespace zephyr::mpc {

/** Validate once when MRT swaps a policy, before indexing or interpolating it. */
template<class Times, class States, class Inputs>
bool validPolicy(const Times& times, const States& states, const Inputs& inputs,
                 Eigen::Index state_dim, Eigen::Index input_dim) {
    if (times.size() < 2 || states.size() != times.size() || inputs.size() != times.size() ||
        state_dim <= 0 || input_dim <= 0 || !(times.back() > times.front())) return false;
    for (size_t i = 0; i < times.size(); ++i) {
        if (!std::isfinite(times[i]) || (i && times[i] < times[i - 1]) ||
            states[i].size() != state_dim || inputs[i].size() != input_dim ||
            !states[i].allFinite() || !inputs[i].allFinite()) return false;
    }
    return true;
}

/** Both ROS time and monotonic receipt age must be valid; never extrapolate. */
inline bool usablePolicy(double now, double sample, double begin, double end,
                         double wall_age, double max_age) {
    return std::isfinite(now) && std::isfinite(sample) && std::isfinite(begin) &&
        std::isfinite(end) && std::isfinite(wall_age) && std::isfinite(max_age) &&
        max_age > 0 && end > begin && now >= begin && sample >= now && sample <= end &&
        wall_age >= 0 && wall_age <= max_age && now - begin <= max_age;
}

/** Gather only named arm DoFs, never the leading mobile-base generalized forces. */
inline bool selectJointTorques(const Eigen::VectorXd& full, const std::vector<int>& indices,
                              Eigen::VectorXd& arm) {
    if (!full.allFinite() || arm.size() != static_cast<Eigen::Index>(indices.size())) return false;
    for (int index : indices) if (index < 0 || index >= full.size()) return false;
    for (size_t i = 0; i < indices.size(); ++i) arm(i) = full(indices[i]);
    return true;
}

/** Fixed-size RT -> non-RT mailbox. A busy consumer never makes the RT writer wait.
 * T must be a bounded value without owning dynamically allocated storage.
 */
template<class T> class TryWriteMailbox {
public:
    static_assert(std::is_trivially_copyable_v<T>);
    bool tryWrite(const T& value) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) return false;
        value_ = value;
        return true;
    }
    T read() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return value_;
    }
private:
    mutable std::mutex mutex_;
    T value_{};
};

/** Lifecycle/update are the sole writer. Old callbacks may not cross a pause
 * and resume, even if they were waiting for TF or the non-RT reference mutex.
 */
class TargetSession {
public:
    void setAccepting(bool accept) {
        const auto current = generation_.load(std::memory_order_relaxed);
        if (static_cast<bool>(current & 1) != accept)
            generation_.store(current + 1, std::memory_order_release);
    }
    uint64_t token() const {
        const auto current = generation_.load(std::memory_order_acquire);
        return current & 1 ? current : 0;
    }
    bool accepts(uint64_t token) const {
        return token != 0 && token == generation_.load(std::memory_order_acquire);
    }
private:
    std::atomic<uint64_t> generation_{0};
};

enum class CollisionStatus { Pending, Safe, Collision, Invalid, Stale };

/** One atomic word keeps status and observation timestamp coherent. Timestamps
 * are monotonic microseconds, not completion times: rechecking an old observation
 * must never refresh the watchdog. The worker is joined before reset().
 */
class CollisionWatchdog {
public:
    void reset() { result_.store(0, std::memory_order_release); }
    void record(bool valid, bool collision, int64_t observation_us) {
        const uint64_t status = !valid ? 3 : collision ? 2 : 1;
        result_.store((static_cast<uint64_t>(observation_us) << 2) | status,
                      std::memory_order_release);
    }
    CollisionStatus status(int64_t now_us, int64_t started_us, int64_t timeout_us) const {
        const auto value = result_.load(std::memory_order_acquire);
        if (!value) return now_us >= started_us && now_us - started_us <= timeout_us ?
            CollisionStatus::Pending : CollisionStatus::Stale;
        const auto code = value & 3;
        if (code == 3) return CollisionStatus::Invalid;
        if (code == 2) return CollisionStatus::Collision;
        const auto observed = static_cast<int64_t>(value >> 2);
        if (now_us < observed || now_us - observed > timeout_us) return CollisionStatus::Stale;
        return CollisionStatus::Safe;
    }
private:
    static_assert(std::atomic<uint64_t>::is_always_lock_free);
    std::atomic<uint64_t> result_{0};
};
} // namespace zephyr::mpc
