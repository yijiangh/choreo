// ros output
#include <ros/console.h>

#include "framefab_task_sequence_planner/utils/Timer.h"

Timer::Timer()
{
	Reset();
}


Timer::~Timer()
{
}


void Timer::Start()
{
	start_time_ = std::chrono::system_clock::now();
}


void Timer::Stop()
{
	end_time_ = std::chrono::system_clock::now();
	sum_time_ += std::chrono::duration_cast<std::chrono::duration<double>>
		(end_time_ - start_time_).count();
	count_++;
}


void Timer::Reset()
{
	sum_time_ = 0;
	count_ = 0;
}


void Timer::Print(char *item)
{
//	printf("%s total-time:%3.4lf   count:%4d   avg-time:%3.4lf\n",
//		item, sum_time_, count_, sum_time_ / count_);

  ROS_INFO("%s total-time:%3.4lf   count:%4d   avg-time:%3.4lf\n",
           item, sum_time_, count_, sum_time_ / count_);
}


std::string Timer::ToString() const
{
	std::string s = std::to_string(sum_time_);
	s.append(" s");
	return s;
}


std::ostream & operator << (std::ostream& os, const Timer& t)
{
	os << t.ToString();
	return os;
}