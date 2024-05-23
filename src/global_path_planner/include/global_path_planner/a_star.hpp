/**
 * @file a_star.hpp
 * @author Takuma Nakao
 * @brief A*アルゴリズム
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <vector>
#include <memory>
#include <algorithm>
#include <unordered_set>

#include <Eigen/Dense>

#include "utils/eigen_hash.hpp"

namespace tlab
{

class AStar {
private:
    class Node {
    public:
        using SharedPtr = std::shared_ptr<Node>;
        Node() : pos(Eigen::Vector2i::Zero()), g(0), h(0), parent(nullptr) {}
        Node(Eigen::Vector2i _pos, double _g, double _h, Node::SharedPtr _parent) : pos(_pos), g(_g), h(_h), parent(_parent) {}
        Eigen::Vector2i pos;
        double g;
        double h;
        Node::SharedPtr parent;
        double cost() const { return g + h; }
        friend bool operator<(const Node& lhs, const Node& rhs) { return lhs.cost() < rhs.cost(); }
        friend bool operator>(const Node& lhs, const Node& rhs) { return rhs < lhs; }
        friend bool operator<=(const Node& lhs, const Node& rhs) { return !(lhs > rhs); }
        friend bool operator>=(const Node& lhs, const Node& rhs) { return !(lhs < rhs); }
    };

    inline static const std::vector<Eigen::Vector2i> directions_4_ = {Eigen::Vector2i(1, 0), Eigen::Vector2i(0, 1), Eigen::Vector2i(-1, 0), Eigen::Vector2i(0, -1)};
    inline static const std::vector<Eigen::Vector2i> directions_8_ = {Eigen::Vector2i(1, 0), Eigen::Vector2i(1, 1), Eigen::Vector2i(0, 1), Eigen::Vector2i(-1, 1), Eigen::Vector2i(-1, 0), Eigen::Vector2i(-1, -1), Eigen::Vector2i(0, -1), Eigen::Vector2i(1, -1)};

    bool is_diagonal_movement_ = false;
    int wall_search_range_ = 3;
    double wall_cost_waight_ = 1.0;

public:
    void set_diagonal_movement(bool is_diagonal_movement) { is_diagonal_movement_ = is_diagonal_movement; }
    void set_wall_search_range(int wall_search_range) { wall_search_range_ = wall_search_range; }
    void set_wall_cost_waight(double wall_cost_waight) { wall_cost_waight_ = wall_cost_waight; }
    std::vector<Eigen::Vector2i> find_path(const Eigen::Vector2i& start, const Eigen::Vector2i goal, const std::unordered_set<Eigen::Vector2i>& map) const
    {
        std::vector<Node::SharedPtr> open;
        std::vector<Node::SharedPtr> close;
        std::vector<Eigen::Vector2i> directions = is_diagonal_movement_ ? directions_8_ : directions_4_;
        open.push_back(std::make_shared<Node>(start, 0, (start - goal).norm(), nullptr));
        while (!open.empty()) {
            std::sort(open.begin(), open.end(), [](const Node::SharedPtr& lhs, const Node::SharedPtr& rhs) { return lhs->cost() > rhs->cost(); });
            auto current = open.back();
            if (current->pos == goal) {
                std::vector<Eigen::Vector2i> path;
                while (current) {
                    path.push_back(current->pos);
                    current = current->parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            open.pop_back();
            close.push_back(current);
            for (const auto& d : directions) {
                auto next_pos = current->pos + d;
                if (map.find(next_pos) != map.end()) {
                    continue;
                }
                double g = current->g + d.norm();
                if (current->parent) {
                    auto d1 = current->pos - current->parent->pos;
                    auto d2 = next_pos - current->pos;
                    if (d1.x() != d2.x()) {
                        g += 1;
                    }
                    if (d1.y() != d2.y()) {
                        g += 1;
                    }
                }
                for (int i = -wall_search_range_; i <= wall_search_range_; i++) {
                    for (int j = -wall_search_range_; j <= wall_search_range_; j++) {
                        if (i == 0 && j == 0) {
                            continue;
                        }
                        auto diff = Eigen::Vector2i(i, j);
                        auto wall_pos = next_pos + diff;
                        if (map.find(wall_pos) != map.end()) {
                            g += wall_cost_waight_ * diff.norm();
                        }
                    }
                }
                auto next = std::make_shared<Node>(next_pos, g, (next_pos - goal).norm(), current);
                if (auto itr = std::find_if(close.begin(), close.end(), [&](const Node::SharedPtr& node) { return node->pos == next->pos; }); itr != close.end()) {
                    if ((*itr)->cost() > next->cost()) {
                        close.erase(itr);
                        open.push_back(next);
                    }
                }
                else if (auto itr = std::find_if(open.begin(), open.end(), [&](const Node::SharedPtr& node) { return node->pos == next->pos; }); itr != open.end()) {
                    if ((*itr)->cost() > next->cost()) {
                        *itr = next;
                    }
                }
                else {
                    open.push_back(next);
                }
            }
        }
        return {};
    }
};

} // namespace tlab