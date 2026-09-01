#pragma once

#include <cstddef>
#include <functional>
#include <optional>

namespace simulator {

// A minimal parallel-for used to run the (independent) simulation runs.
//
// `requested_threads` is the num_threads CLI value: the number of WORKER threads
// requested IN ADDITION to the main thread. Semantics from the assignment:
//   - nullopt or <= 1  -> everything runs on the calling thread (1 thread total)
//   - >= 2             -> that many workers, capped at the task count so no
//                         worker sits idle; main waits (joins)
// The total thread count is never exactly 2: a lone worker (which would give
// 1 worker + main = 2) is demoted to inline execution.
class ParallelExecutor {
public:
    // Invokes task(i) exactly once for each i in [0, count). task must be safe to
    // call concurrently for distinct i (each run writes to its own result slot).
    static void run(std::size_t count, std::optional<int> requested_threads,
                    const std::function<void(std::size_t)>& task);

    // The number of worker threads run() will spawn for these inputs (0 == inline).
    [[nodiscard]] static std::size_t workerCount(std::size_t count,
                                                 std::optional<int> requested_threads);
};

} // namespace simulator
