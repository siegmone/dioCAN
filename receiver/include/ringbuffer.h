#include <Arduino.h>

#include <array>
#include <vector>

template <typename T, std::size_t Capacity>
class RingBuffer {
   private:
    // Use one extra slot to distinguish full from empty.
    std::array<T, Capacity + 1> buffer{};
    std::size_t start = 0;
    std::size_t end = 0;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    // Helper to get the actual size of the underlying array.
    constexpr std::size_t internal_size() const {
        return Capacity + 1;
    }

   public:
    // Returns true if the buffer is full.
    bool full() const {
        return (end + 1) % internal_size() == start;
    }

    // Returns true if the buffer is empty.
    bool empty() const {
        return start == end;
    }

    // Adds an element to the buffer.
    // Returns false if the buffer is full.
    bool push(const T& value) {
        taskENTER_CRITICAL(&mux);
        if (full()) {
            taskEXIT_CRITICAL(&mux);
            return false;
        }
        buffer[end] = value;
        end = (end + 1) % internal_size();
        taskEXIT_CRITICAL(&mux);
        return true;
    }

    // Removes an element from the buffer.
    // Returns false if the buffer is empty.
    bool pop(T& value) {
        taskENTER_CRITICAL(&mux);
        if (empty()) {
            taskEXIT_CRITICAL(&mux);
            return false;
        }
        value = buffer[start];
        start = (start + 1) % internal_size();
        taskEXIT_CRITICAL(&mux);
        return true;
    }

    std::vector<T> emit() {
        std::vector<T> result;
        result.reserve(Capacity);
        taskENTER_CRITICAL(&mux);
        while (start != end) {
            result.push_back(buffer[start]);
            start = (start + 1) % internal_size();
        }

        start = 0;
        end = 0;
        taskEXIT_CRITICAL(&mux);

        return result;
    }
};