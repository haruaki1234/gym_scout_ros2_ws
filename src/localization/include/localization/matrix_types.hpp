/**
 * @file matrix_types.hpp
 * @author Takuma Nakao
 * @brief Eigenの型定義
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <Eigen/Core>

//! 6次元ベクトルの型エイリアス
using Vector6d = Eigen::Matrix<double, 6, 1>;
//! 6x6行列の型エイリアス
using Matrix6d = Eigen::Matrix<double, 6, 6>;