#include <Simulator/ParallelExecutor.h>

#include <algorithm>
#include <atomic>
#include <thread>
#include <vector>

namespace simulator {

std::size_t ParallelExecutor::workerCount(std::size_t count, std::optional<int> requested_threads) {
    if (count == 0) {
        return 0;
    }
    if (!requested_threads || *requested_threads <= 1) {
        return 0; // run inline on the calling thread
    }
    // Cap at the task count so no worker is idle.
    std::size_t workers = std::min(static_cast<std::size_t>(*requested_threads), count);
    // A single worker plus main would be 2 threads total, which is disallowed;
    // fall back to inline execution instead.
    if (workers <= 1) {
        return 0;
    }
    return workers;
}

void ParallelExecutor::run(std::size_t count, std::optional<int> requested_threads,
                           const std::function<void(std::size_t)>& task) {
    const std::size_t workers = workerCount(count, requested_threads);

    if (workers == 0) {
        for (std::size_t i = 0; i < count; ++i) {
            task(i);
        }
        return;
    }

    std::atomic<std::size_t> next{0};
    std::vector<std::thread> pool;
    pool.reserve(workers);
    for (std::size_t t = 0; t < workers; ++t) {
        pool.emplace_back([&] {
            for (std::size_t i = next.fetch_add(1); i < count; i = next.fetch_add(1)) {
                task(i);
            }
        });
    }
    for (std::thread& worker : pool) {
        worker.join();
    }
}

} // namespace simulator
