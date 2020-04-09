#pragma once

#include <vector>
#include <thread>

namespace lbvh {

//! \brief Emulates a task scheduler.
//! The only real difference this
//! has from a full implementation is
//! that this class creates and joins
//! threads instead of creating them
//! once and using synchronization mechanisms.
class fake_task_scheduler final {
  //! The maximum number of threads to run.
  size_type max_thread_count;
public:
  //! Constructs a new fake task scheduler.
  //! \param max_threads The maximum number of threads to run.
  fake_task_scheduler(size_type max_threads_)
    : max_thread_count(max_threads_ ? max_threads_ : 1) { }
  //! Schedules a new task to be completed.
  //! \tparam task_type The type of the task functor.
  //! \tparam arg_types The arguments to pass to the thread.
  template <typename task_type, typename... arg_types>
  void operator () (task_type task, arg_types... args) {

    std::vector<std::thread> threads;

    for (size_type i = 0; i < max_thread_count - 1; i++) {

      work_division div { i, max_thread_count };

      threads.emplace_back(task, div, args...);
    }

    task(work_division { max_thread_count - 1, max_thread_count }, args...);

    for (auto& th : threads) {
      th.join();
    }
  }
  //! Indicates to the library the maximum number of threads
  //! that may be invoked at a time. This is useful for determining
  //! how much memory to allocate for some functions.
  //!
  //! \return The max number of threads that may be invoked at a time.
  inline size_type max_threads() const noexcept {
    return max_thread_count;
  }
};

} // namespace lbvh
