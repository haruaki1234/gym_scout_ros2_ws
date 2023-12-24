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

template<size_t D>
class EigenVectorTree {
public:
    static EigenVectorTree build(std::vector<Eigen::Matrix<double, D, 1>> p_v) { return EigenVectorTree(p_v); }
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
    std::array<std::unique_ptr<EigenVectorTree>, 2> next_node_ = {nullptr, nullptr};
    Eigen::Matrix<double, D, 1> point_;
    size_t axis_;
};

template<class T, size_t D>
class EigenVectorAndDataTree {
public:
    static EigenVectorAndDataTree build(std::unordered_map<Eigen::Matrix<double, D, 1>, T> map)
    {
        std::vector<std::pair<Eigen::Matrix<double, D, 1>, T>> p_v;
        p_v.reserve(map.size());
        for (const auto& [key, value] : map) {
            p_v.push_back({key, value});
        }
        return EigenVectorAndDataTree(p_v);
    }
    static EigenVectorAndDataTree build(std::vector<std::pair<Eigen::Matrix<double, D, 1>, T>> p_v) { return EigenVectorAndDataTree(p_v); }
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
    std::array<std::unique_ptr<EigenVectorAndDataTree>, 2> next_node_ = {nullptr, nullptr};
    Eigen::Matrix<double, D, 1> point_;
    T data_;
    size_t axis_;
};

} // namespace kd_tree

} // namespace tlab