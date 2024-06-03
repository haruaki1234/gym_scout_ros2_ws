/**
 * @file eigen_hash.hpp
 * @author Takuma Nakao
 * @brief Eigenの型に対するハッシュ関数
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <Eigen/Core>

namespace std
{

/**
 * @brief Eigen Matrixのハッシュ関数
 *
 * @tparam Scalar Matrixの要素型
 * @tparam Rows 行数
 * @tparam Cols 列数
 */
template<typename Scalar, int Rows, int Cols>
struct hash<Eigen::Matrix<Scalar, Rows, Cols>> {
    /**
     * @brief ハッシュ関数
     *
     * @param matrix Matrix
     * @return size_t ハッシュ値
     */
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