/*
* ==========================================================================
*		This file is part of the implementation of
*
*		<Sustainable Fabrication of Frame Shapes>
*		Yijiang Huang, Juyong Zhang, Xin Hu, Guoxian Song, Zhongyuan Liu, Lei Yu, Ligang Liu
*		In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2016)
*
*		Description:
*
*		Version:  2.0
*		Created:  Oct/20/2015
*		Updated: Aug/24/2016
*
*		Author: Yijiang Huang
*		Company:  GCL@USTC
* ==========================================================================
*/
#include <cstddef>
#ifndef GLOBALFUNCTIONS_H
#define GLOBALFUNCTIONS_H

namespace framefab
{
template<class object>
void safeDelete(object *ptr)
{
    if (ptr != NULL)
    {
        delete ptr;
        ptr = NULL;
    }
}

template<class object>
void safeDeletes(object *ptr)
{
    if (ptr != NULL)
    {
        delete[] ptr;
        ptr = NULL;
    }
}
}

#endif // GLOBALFUNCTIONS_H
