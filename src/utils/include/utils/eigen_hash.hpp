#pragma once

#include <Eigen/Core>

namespace std
{

template<typename Scalar, int Rows, int Cols>
struct hash<Eigen::Matrix<Scalar, Rows, Cols>> {
    size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols>& matrix) const
    {
        size_t seed = 0;
        for (size_t i = 0; i < static_cast<size_t>(matrix.size()); ++i) {
            Scalar elem = *(matrix.data() + i);
            seed ^= std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

} // namespace std