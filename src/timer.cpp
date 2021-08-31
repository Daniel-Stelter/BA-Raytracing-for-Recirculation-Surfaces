#include "timer.hh"

//-------------------------------------------------------------------------//
namespace RS
{
    //-------------------------------------------------------------------------//
    Timer::Timer() : is_running{true},
                     start_time{std::chrono::system_clock::now()},
                     from_previous_runs{} {}

    //-------------------------------------------------------------------------//
    bool Timer::start()
    {
        if (!is_running)
        {
            start_time = std::chrono::system_clock::now();
            return (is_running = true);
        }
        return false;
    }

    //-------------------------------------------------------------------------//
    bool Timer::stop()
    {
        if (is_running)
        {
            from_previous_runs += getCurrentTime();
            is_running = false;
            return true;
        }
        return false;
    }

    //-------------------------------------------------------------------------//
    duration_t Timer::getCurrentTime() const
    {
        if (!is_running)
            return duration_t{};
        auto end_time = std::chrono::system_clock::now();
        return end_time - start_time;
    }

    //-------------------------------------------------------------------------//
    duration_t Timer::getTotalTime() const
    {
        return from_previous_runs + getCurrentTime();
    }

    //-------------------------------------------------------------------------//
    void Timer::printStartTime() const
    {
        time_t time_start = std::chrono::system_clock::to_time_t(start_time);
        std::cout << "Starting time: " << ctime(&time_start) << std::flush;
    }

    //-------------------------------------------------------------------------//
    void Timer::printTotalTime() const
    {
        double total_seconds = getTotalTime().count();
        int hours = total_seconds / 3600;
        int minutes = (int)(total_seconds / 60.0) % 60;
        int seconds = (int)total_seconds % 60;
        int millis = (int)(total_seconds * 1000) % 1000;
        std::cout << "Time needed: " << hours << "h " << minutes << "min " << seconds << "s " << millis << "ms" << std::endl;
    }

    //-------------------------------------------------------------------------//
    //-------------------------------------------------------------------------//

    TimerHandler TimerHandler::_instance_integration = TimerHandler();
    TimerHandler TimerHandler::_instance_overall = TimerHandler();

    //-------------------------------------------------------------------------//
    TimerHandler::TimerHandler() : timers{},
                                   total_time{},
                                   lck{}
    {
        omp_init_lock(&lck);
    }

    //-------------------------------------------------------------------------//
    TimerHandler::~TimerHandler()
    {
        for (auto &p : timers)
        {
            if (p)
            {
                delete p;
                p = nullptr;
            }
        }
    }

    //-------------------------------------------------------------------------//
    size_t TimerHandler::createTimer()
    {
        omp_set_lock(&lck);
        size_t index = 0;
        while (index < timers.size() && timers[index] != nullptr)
            ++index;
        if (index == timers.size())
            timers.push_back(new Timer());
        else
            timers[index] = new Timer();
        omp_unset_lock(&lck);
        return index;
    }

    //-------------------------------------------------------------------------//
    bool TimerHandler::deleteTimer(size_t index)
    {
        omp_set_lock(&lck);
        bool exists = index < timers.size() && timers[index];
        if (exists)
        {
            total_time += timers[index]->getCurrentTime();
            delete timers[index];
            timers[index] = nullptr;
            while (!timers.empty() && timers.back() == nullptr)
                timers.pop_back();
        }
        omp_unset_lock(&lck);
        return exists;
    }

    //-------------------------------------------------------------------------//
    duration_t TimerHandler::getTotalTime()
    {
        duration_t result = total_time;
        omp_set_lock(&lck);
        for (auto &p : timers)
            if (p)
                result += p->getCurrentTime();
        omp_unset_lock(&lck);
        return result;
    }

    //-------------------------------------------------------------------------//
    void TimerHandler::printTotalTime()
    {
        duration_t total_seconds = getTotalTime();
        int hours = total_seconds.count() / 3600.0;
        int minutes = (int)(total_seconds.count() / 60.0) % 60;
        int seconds = (int)(total_seconds.count()) % 60;
        std::cout << "Time needed: " << hours << "h " << minutes << "min " << seconds << "s" << std::endl;
    }
    //-------------------------------------------------------------------------//
}
//-------------------------------------------------------------------------//