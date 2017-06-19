/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<FrameFab: Robotic Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
----------------------------------------------------------------------------
*		class:	Timer
*
*		Description:  Timing tool.
*
*		Version:  1.0
*		Created:  Mar/23/2016
*		Updated:  Aug/24/2016
*
*		Author:  Xin Hu, Yijiang Huang, Guoxian Song
*		Company:  GCL@USTC
----------------------------------------------------------------------------
*		Copyright (C) 2016  Yijiang Huang, Xin Hu, Guoxian Song, Juyong Zhang
*		and Ligang Liu.
*
*		FrameFab is free software: you can redistribute it and/or modify
*		it under the terms of the GNU General Public License as published by
*		the Free Software Foundation, either version 3 of the License, or
*		(at your option) any later version.
*
*		FrameFab is distributed in the hope that it will be useful,
*		but WITHOUT ANY WARRANTY; without even the implied warranty of
*		MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*		GNU General Public License for more details.
*
*		You should have received a copy of the GNU General Public License
*		along with FrameFab.  If not, see <http://www.gnu.org/licenses/>.
* ==========================================================================
*/

#pragma once

#ifndef FIBER_PRINT_TIMER_H
#define FIBER_PRINT_TIMER_H

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>

class Timer
{
	typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;
	typedef std::chrono::duration<double> Duration;
public:
	Timer();
	~Timer();

public:
	void	Start();
	void	Stop();
	void	Reset();
	void	Print(char *item);
	std::string ToString() const;

private:
	TimePoint	start_time_;
	TimePoint	end_time_;
	double		sum_time_;
	int			count_;
};

std::ostream& operator << (std::ostream & os, const Timer& t);


#endif //Timer.h