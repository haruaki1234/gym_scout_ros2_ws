/**
 * @file kd_tree.hpp
 * @author Takuma Nakao
 * @brief KD木
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <array>
#include <vector>
#include <limits>
#include <algorithm>
#include <memory>

#include <Eigen/Dense>

#include "./math_util.hpp"

namespace tlab
{

namespace kd_tree
{

/**
 * @brief D次元のベクトルをKD木で管理するクラス
 *
 * @tparam D ベクトルの次元
 */
template<size_t D>
class EigenVectorTree {
public:
    /**
     * @brief KD木を構築
     *
     * @param p_v ベクトルのリスト
     * @return EigenVectorTree KD木
     */
    static EigenVectorTree build(std::vector<Eigen::Matrix<double, D, 1>> p_v) { return EigenVectorTree(p_v); }
    /**
     * @brief Construct a new Eigen Vector Tree object
     *
     * @param p_v ベクトルのリスト
     * @param depth 深さ
     */
    EigenVectorTree(std::vector<Eigen::Matrix<double, D, 1>>& p_v, size_t depth = 1) : axis_(depth % D)
    {
        const size_t mid = (p_v.size() - 1) / 2;
        std::nth_element(p_v.begin(), p_v.begin() + mid, p_v.end(), [&](const auto& lhs, const auto& rhs) { return lhs(axis_) < rhs(axis_); });
        point_ = p_v[mid];
        {
            std::vector<Eigen::Matrix<double, D, 1>> v(p_v.begin(), p_v.begin() + mid);
            if (!v.empty()) {
                next_node_[0] = std::make_unique<EigenVectorTree>(v, depth + 1);
            }
        }
        {
            std::vector<Eigen::Matrix<double, D, 1>> v(p_v.begin() + mid + 1, p_v.end());
            if (!v.empty()) {
                next_node_[1] = std::make_unique<EigenVectorTree>(v, depth + 1);
            }
        }
    }
    /**
     * @brief クエリに最も近いベクトルを探索
     *
     * @param query クエリベクトル
     * @param guess 現状の推定値
     * @param min_sq_dist 最小二乗距離
     * @return Eigen::Matrix<double, D, 1> 結果ベクトル
     */
    Eigen::Matrix<double, D, 1> nn_serch(const Eigen::Matrix<double, D, 1>& query, Eigen::Matrix<double, D, 1> guess = {}, std::shared_ptr<double> min_sq_dist = nullptr) const
    {
        if (!min_sq_dist) {
            min_sq_dist = std::make_shared<double>(std::numeric_limits<double>::max());
        }
        const Eigen::Matrix<double, D, 1> e = query - point_;
        const double sq_dist = e.dot(e);
        if (sq_dist < *min_sq_dist) {
            guess = point_;
            *min_sq_dist = sq_dist;
        }
        const size_t dir = query(axis_) < point_(axis_) ? 0 : 1;
        if (next_node_[dir] != nullptr) {
            guess = next_node_[dir]->nn_serch(query, guess, min_sq_dist);
        }
        if (next_node_[!dir] != nullptr && square(query(axis_) - point_(axis_)) < *min_sq_dist) {
            guess = next_node_[!dir]->nn_serch(query, guess, min_sq_dist);
        }
        return guess;
    }

private:
    //! 子ノード
    std::array<std::unique_ptr<EigenVectorTree>, 2> next_node_ = {nullptr, nullptr};
    //! 要素ベクトル
    Eigen::Matrix<double, D, 1> point_;
    //! 分割軸
    size_t axis_;
};

/**
 * @brief D次元のベクトルとデータをKD木で管理するクラス
 *
 * @tparam T データ型
 * @tparam D ベクトルの次元
 */
template<class T, size_t D>
class EigenVectorAndDataTree {
public:
    /**
     * @brief KD木を構築
     *
     * @param map ベクトルとデータのマップ
     * @return EigenVectorAndDataTree KD木
     */
    static EigenVectorAndDataTree build(std::unordered_map<Eigen::Matrix<double, D, 1>, T> map)
    {
        std::vector<std::pair<Eigen::Matrix<double, D, 1>, T>> p_v;
        p_v.reserve(map.size());
        for (const auto& [key, value] : map) {
            p_v.push_back({key, value});
        }
        return EigenVectorAndDataTree(p_v);
    }
    /**
     * @brief KD木を構築
     *
     * @param p_v ベクトルとデータのリスト
     * @return EigenVectorAndDataTree KD木
     */
    static EigenVectorAndDataTree build(std::vector<std::pair<Eigen::Matrix<double, D, 1>, T>> p_v) { return EigenVectorAndDataTree(p_v); }
    /**
     * @brief Construct a new Eigen Vector And Data Tree object
     *
     * @param p_v ベクトルとデータのリスト
     * @param depth 深さ
     */
    EigenVectorAndDataTree(std::vector<std::pair<Eigen::Matrix<double, D, 1>, T>>& p_v, size_t depth = 1) : axis_(depth % D)
    {
        const size_t mid = (p_v.size() - 1) / 2;
        std::nth_element(p_v.begin(), p_v.begin() + mid, p_v.end(), [&](const auto& lhs, const auto& rhs) { return lhs.first(axis_) < rhs.first(axis_); });
        point_ = p_v[mid].first;
        data_ = p_v[mid].second;
        {
            std::vector<std::pair<Eigen::Matrix<double, D, 1>, T>> v(p_v.begin(), p_v.begin() + mid);
            if (!v.empty()) {
                next_node_[0] = std::make_unique<EigenVectorAndDataTree>(v, depth + 1);
            }
        }
        {
            std::vector<std::pair<Eigen::Matrix<double, D, 1>, T>> v(p_v.begin() + mid + 1, p_v.end());
            if (!v.empty()) {
                next_node_[1] = std::make_unique<EigenVectorAndDataTree>(v, depth + 1);
            }
        }
    }
    /**
     * @brief クエリに最も近いベクトルとデータを探索
     *
     * @param query クエリベクトル
     * @param guess 現状の推定値
     * @param min_sq_dist 最小二乗距離
     * @return std::pair<Eigen::Matrix<double, D, 1>, T> 結果ベクトルとデータ
     */
    std::pair<Eigen::Matrix<double, D, 1>, T> nn_serch(const Eigen::Matrix<double, D, 1>& query, std::pair<Eigen::Matrix<double, D, 1>, T> guess = {}, std::shared_ptr<double> min_sq_dist = nullptr) const
    {
        if (!min_sq_dist) {
            min_sq_dist = std::make_shared<double>(std::numeric_limits<double>::max());
        }
        const Eigen::Matrix<double, D, 1> e = query - point_;
        const double sq_dist = e.dot(e);
        if (sq_dist < *min_sq_dist) {
            guess = {point_, data_};
            *min_sq_dist = sq_dist;
        }
        const size_t dir = query(axis_) < point_(axis_) ? 0 : 1;
        if (next_node_[dir] != nullptr) {
            guess = next_node_[dir]->nn_serch(query, guess, min_sq_dist);
        }
        if (next_node_[!dir] != nullptr && square(query(axis_) - point_(axis_)) < *min_sq_dist) {
            guess = next_node_[!dir]->nn_serch(query, guess, min_sq_dist);
        }
        return guess;
    }

private:
    //! 子ノード
    std::array<std::unique_ptr<EigenVectorAndDataTree>, 2> next_node_ = {nullptr, nullptr};
    //! 要素ベクトル
    Eigen::Matrix<double, D, 1> point_;
    //! データ
    T data_;
    //! 分割軸
    size_t axis_;
};

} // namespace kd_tree

} // namespace tlab