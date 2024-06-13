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

/**
 * @brief 寿命付きオブジェクトキュークラス
 * @details キューを取得する際に年齢を増加させ，最大年齢に達した場合はキューから削除する
 *
 * @tparam Object オブジェクト型
 */
template<typename Object>
class AgedObjectQueue {
public:
    /**
     * @brief Construct a new Aged Object Queue object
     *
     * @param max_age 最大年齢
     */
    explicit AgedObjectQueue(const int max_age) : max_age_(max_age) {}

    /**
     * @brief キューが空かどうか
     *
     * @retval true 空
     * @retval false 空でない
     */
    bool empty() const { return this->size() == 0; }

    /**
     * @brief キューのサイズを取得
     *
     * @return size_t キューのサイズ
     */
    size_t size() const { return objects_.size(); }

    /**
     * @brief キューの末尾要素を取得
     *
     * @return Object 末尾要素
     */
    Object back() const { return objects_.back(); }

    /**
     * @brief キューを追加
     *
     * @param object 追加するオブジェクト
     */
    void push(const Object& object)
    {
        objects_.push(object);
        ages_.push(0);
    }

    /**
     * @brief キューの先頭要素を年齢を増加させて取得
     *
     * @return Object 先頭要素
     */
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

    /**
     * @brief キューを削除
     *
     */
    void clear()
    {
        objects_ = std::queue<Object>();
        ages_ = std::queue<int>();
    }

private:
    //! 最大年齢
    const int max_age_;
    //! オブジェクトキュー
    std::queue<Object> objects_;
    //! 年齢キュー
    std::queue<int> ages_;
};

} // namespace tlab