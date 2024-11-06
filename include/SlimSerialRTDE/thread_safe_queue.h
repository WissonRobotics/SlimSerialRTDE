#pragma once

#include <algorithm>
#include <concepts>
#include <deque>
#include <mutex>
#include <optional>
#include <condition_variable>
namespace dp {
    /**
     * @brief Simple concept for the Lockable and Basic Lockable types as defined by the C++
     * standard.
     * @details See https://en.cppreference.com/w/cpp/named_req/Lockable and
     * https://en.cppreference.com/w/cpp/named_req/BasicLockable for details.
     */
    template <typename Lock>
    concept is_lockable = requires(Lock&& lock) {
        lock.lock();
        lock.unlock();
        { lock.try_lock() } -> std::convertible_to<bool>;
    };

    template <typename T, typename Lock = std::mutex>
        requires is_lockable<Lock>
    class thread_safe_queue {
      public:
        using value_type = T;
        using size_type = typename std::deque<T>::size_type;

        thread_safe_queue() = default;
 
 
        void push_back(T&& value) {
            {
                // std::scoped_lock lock(mutex_);
                std::unique_lock<std::mutex> lock(_sync);
                data_.push_back(std::forward<T>(value));
            }
            _cvCanPop.notify_one();
        }

        void push_front(T&& value) {
            {
            // std::scoped_lock lock(mutex_);
            std::unique_lock<std::mutex> lock(_sync);
            data_.push_front(std::forward<T>(value));
            }
            _cvCanPop.notify_one();
        }

        [[nodiscard]] bool empty() const {
            // std::scoped_lock lock(mutex_);
            std::unique_lock<std::mutex> lock(_sync);
            return data_.empty();
        }
 
        [[nodiscard]] T& at(size_t n=0) {
            
            std::unique_lock<std::mutex> lock(_sync);
  
            try{
                return data_.at(n);
            }
            catch(...){
                printf("[ERROR!!!!!!!!!] Bad access to queue data.at(%d)",n);
                return Tbad;
            }  
        }
 

        size_t size() {

            return data_.size();
        }

        [[nodiscard]] std::optional<T> pop_front(int timeoutMs=0) {
            // std::scoped_lock lock(mutex_);
            std::unique_lock<std::mutex> lock(_sync);
            
            if(timeoutMs>0){
                auto timeExp = std::chrono::system_clock::now() + std::chrono::milliseconds(timeoutMs);
                while(data_.empty()) {
                    if(_cvCanPop.wait_until(lock,timeExp)==std::cv_status::timeout){
                        break;
                    }
                }
            }

            if (data_.empty()) return std::nullopt;

            auto front = std::move(data_.front());
            data_.pop_front();
            return front;
        }

        [[nodiscard]] std::optional<T> pop_back(int timeoutMs=0) {
            // std::scoped_lock lock(mutex_);
            std::unique_lock<std::mutex> lock(_sync);

            if(timeoutMs>0){
                auto timeExp = std::chrono::system_clock::now() + std::chrono::milliseconds(timeoutMs);
                while(data_.empty()) {
                    if(_cvCanPop.wait_until(lock,timeExp)==std::cv_status::timeout){
                        break;
                    }
                }
            }

            if (data_.empty()) return std::nullopt;

            auto back = std::move(data_.back());
            data_.pop_back();
            return back;
        }

        [[nodiscard]] std::optional<T> steal() {
            // std::scoped_lock lock(mutex_);
            std::unique_lock<std::mutex> lock(_sync);
            if (data_.empty()) return std::nullopt;

            auto back = std::move(data_.back());
            data_.pop_back();
            return back;
        }

        void rotate_to_front(const T& item) {
            // std::scoped_lock lock(mutex_);
            std::unique_lock<std::mutex> lock(_sync);
            auto iter = std::find(data_.begin(), data_.end(), item);

            if (iter != data_.end()) {
                std::ignore = data_.erase(iter);
            }

            data_.push_front(item);
        }

        [[nodiscard]] std::optional<T> copy_front_and_rotate_to_back() {
            // std::scoped_lock lock(mutex_);
            std::unique_lock<std::mutex> lock(_sync);

            if (data_.empty()) return std::nullopt;

            auto front = data_.front();
            data_.pop_front();

            data_.push_back(front);

            return front;
        }

      private:
        std::deque<T> data_{};
        // mutable Lock mutex_{};
        std::mutex _sync;
        std::condition_variable _cvCanPop;
        T Tbad;
    };
}  // namespace dp
