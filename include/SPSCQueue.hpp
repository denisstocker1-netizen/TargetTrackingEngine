#pragma once 

#include <array>
#include <atomic>
#include <optional>
#include <new>

#if defined(__cpp_lib_hardware_interference_size)
    using std::hardware_destructive_interference_size;
#else
    // Fallback für x86_64 und gängige CPU-Architekturen (64 Bytes)
    constexpr std::size_t hardware_destructive_interference_size = 64;
#endif

template<typename T, std::size_t Capacity>
class SPSCQueue {
private:
    std::array<T, Capacity> buffer;

    alignas(hardware_destructive_interference_size) std::atomic<std::size_t> head{0};
    alignas(hardware_destructive_interference_size) std::atomic<std::size_t> tail{0};
public:

    [[nodiscard]] auto isFull() const noexcept  -> bool{
        const auto current_head = head.load(std::memory_order_relaxed);
        const auto next_head = (current_head + 1) % Capacity;
        return next_head == tail.load(std::memory_order_relaxed);
    }

    [[nodiscard]] auto isEmpty() const noexcept  -> bool{
        return head.load(std::memory_order_relaxed) == tail.load(std::memory_order_relaxed);
    }

    [[nodiscard]] auto push(const T& data) -> bool {
        auto current_head = head.load(std::memory_order_relaxed);
        auto next_head = (current_head + 1) % Capacity;

        if (next_head == tail.load(std::memory_order_acquire)){
            return false;
        }else {
            buffer[current_head] = data;

            head.store(next_head, std::memory_order_release);
            return true;
        }
    }

    [[nodiscard]] auto pop() -> std::optional<T> {
        auto current_tail = tail.load(std::memory_order_relaxed);
        auto current_head = head.load(std::memory_order_acquire);

        if (current_tail == current_head) {
            return std::nullopt;
        }else{
            T data = buffer[current_tail];
            tail.store((current_tail + 1) % Capacity, std::memory_order_release);

            return data;
        }
    }
};