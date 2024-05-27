/**
 * @file state_index.hpp
 * @author Takuma Nakao
 * @brief 状態ベクトルのインデックス
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

namespace tlab
{

/**
 * @enum IDX
 * @brief 状態ベクトルのインデックス
 *
 */
// clang-format off
enum IDX {
    X = 0, //!< 位置X
    Y = 1, //!< 位置Y
    YAW = 2, //!< 姿勢YAW
    VX = 3, //!< 速度VX
    VY = 4, //!< 速度VY
    WZ = 5, //!< 角速度WZ
};
// clang-format on

} // namespace tlab