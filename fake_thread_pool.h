#pragma once

namespace lbvh {

//! \brief Emulates a task scheduler.
//! The only real difference this
//! has from a full implementation is
//! that this class creates and joins
//! threads instead of creating them
//! once and using synchronization mechanisms.
class fake_task_scheduler final {
  //! These run the tasks sent to the scheduler.
  std::vector<std::thread> threads;
  //! The maximum number of threads to run.
  size_type max_threads;
public:
  //! Constructs a new fake task scheduler.
  //! \param max_threads The maximum number of threads to run.
  fake_task_scheduler(size_type max_threads_)
    : max_threads(max_threads_) { }
  //! Schedules a new task to be completed.
  //! \tparam task_type The type of the task functor.
  //! \tparam arg_types The arguments to pass to the thread.
  template <typename task_type, typename... arg_types>
  void operator () (task_type task, arg_types... args) {

    for (size_type i = 0; i < max_threads; i++) {

      work_division div { i, max_threads };

      threads.emplace_back(task, div, args...);
    }

    for (auto& th : threads) {
      th.join();
    }

    threads.clear();
  }
};

} // namespace lbvh
