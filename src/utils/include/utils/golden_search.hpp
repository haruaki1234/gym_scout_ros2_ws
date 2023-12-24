#pragma once

#include <functional>

namespace tlab
{

template<typename T>
static T golden_search(std::function<double(T)> f, const T low, const T high, int loop_num = 5)
{
    const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    T left = low;
    T right = high;
    T p0 = (phi * left + right) / (1.0 + phi);
    double f0 = f(p0);
    T p1 = (left + phi * right) / (1.0 + phi);
    double f1 = f(p1);

    for (int i = 0; i < loop_num; i++) {
        if (f0 < f1) {
            right = p1;
            p1 = p0;
            f1 = f0;
            p0 = (phi * left + right) / (1.0 + phi);
            f0 = f(p0);
        }
        else {
            left = p0;
            p0 = p1;
            f0 = f1;
            p1 = (left + phi * right) / (1.0 + phi);
            f1 = f(p1);
        }
        if (left == right) {
            return left;
        }
    }

    return 0.5 * (left + right);
}

}