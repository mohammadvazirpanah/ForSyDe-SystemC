#ifndef FUNC_HPP
#define FUNC_HPP

#include <forsyde.hpp>

using namespace ForSyDe;


void func(int& out1, const int& inp1)
{
#pragma ForSyDe begin func
    out1 = inp1 + 1;
#pragma ForSyDe end
}

#endif