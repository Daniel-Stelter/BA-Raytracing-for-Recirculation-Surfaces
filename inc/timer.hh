#pragma once

#include <chrono>
#include <iostream>
#include <omp.h>
#include <vector>

//--------------------------------------------------------------------------//
using namespace std::chrono::_V2;
typedef system_clock::time_point time_point_t;
typedef std::chrono::duration<double> duration_t;

//--------------------------------------------------------------------------//
namespace RS
{
    // ----------------------------------------------------------- //
    class Timer
    {
    private:
        // ----------------------------------------------------------- //
        bool is_running;
        time_point_t start_time;
        duration_t from_previous_runs;
        // ----------------------------------------------------------- //
    public:
        // ----------------------------------------------------------- //
        Timer();
        // ----------------------------------------------------------- //
        bool start();
        bool stop();
        // ----------------------------------------------------------- //
        duration_t getCurrentTime() const;
        duration_t getTotalTime() const;
        // ----------------------------------------------------------- //
        void printStartTime() const;
        void printTotalTime() const;
        // ----------------------------------------------------------- //
    };
    // ----------------------------------------------------------- //
    /// Class with two singletons for measuring total execution and specific
    /// integration time. Provides options for creating timers and delete
    /// them. When deleted, the duration will be summed up. In the end,
    /// one can look up the ratio between both.
    class TimerHandler
    {
    private:
        // ----------------------------------------------------------- //
        std::vector<Timer *> timers;
        duration_t total_time;
        omp_lock_t lck;
        // ----------------------------------------------------------- //
        static TimerHandler _instance_overall;
        static TimerHandler _instance_integration;
        // ----------------------------------------------------------- //
        TimerHandler();
        ~TimerHandler();
        // ----------------------------------------------------------- //
    public:
        // ----------------------------------------------------------- //
        /// Returns the instance of the overall timer
        static TimerHandler &overall_timer()
        {
            return _instance_overall;
        }
        //--------------------------------------------------------------------------//
        /// Returns the instance of the integration timer
        static TimerHandler &integration_timer()
        {
            return _instance_integration;
        }
        // ----------------------------------------------------------- //
        /// Creates a timer. The returned number is an index for deleting
        /// the timer.
        size_t createTimer();
        //--------------------------------------------------------------------------//
        /// Deletes the timer with the associated index.
        bool deleteTimer(size_t index);
        // ----------------------------------------------------------- //
        duration_t getTotalTime();
        // ----------------------------------------------------------- //
        void printTotalTime();
        // ----------------------------------------------------------- //
        /// Prints ratio of integration to overall time to the console
        static void printRatio()
        {
            double r = TimerHandler::integration_timer().getTotalTime().count() /
                       TimerHandler::overall_timer().getTotalTime().count();
            if (r != r) // Test for not a number
                r = 0.0;
            std::cout << "Portion of integration: " << r * 100.0 << "%" << std::endl;
        }
        // ----------------------------------------------------------- //
        /// Resets both timer instances (for integration and overall)
        static void reset()
        {
            _instance_integration = TimerHandler();
            _instance_overall = TimerHandler();
        }
        // ----------------------------------------------------------- //
    };
    // ----------------------------------------------------------- //
}
//--------------------------------------------------------------------------//