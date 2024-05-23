/**
 * @file aged_object_queue.hpp
 * @author Takuma Nakao
 * @brief 寿命付きオブジェクトキュー
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <queue>

namespace tlab
{

template<typename Object>
class AgedObjectQueue {
public:
    explicit AgedObjectQueue(const int max_age) : max_age_(max_age) {}

    bool empty() const { return this->size() == 0; }

    size_t size() const { return objects_.size(); }

    Object back() const { return objects_.back(); }

    void push(const Object& object)
    {
        objects_.push(object);
        ages_.push(0);
    }

    Object pop_increment_age()
    {
        const Object object = objects_.front();
        const int age = ages_.front() + 1;
        objects_.pop();
        ages_.pop();

        if (age < max_age_) {
            objects_.push(object);
            ages_.push(age);
        }

        return object;
    }

    void clear()
    {
        objects_ = std::queue<Object>();
        ages_ = std::queue<int>();
    }

private:
    const int max_age_;
    std::queue<Object> objects_;
    std::queue<int> ages_;
};

} // namespace tlab