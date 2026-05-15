#pragma once
//
// Fixed-capacity ring buffer of floats with on-the-fly min/max/sum tracking.
// Header-only; used for the "rolling stats" screen and for smoothing noisy
// instantaneous readings.

#include <math.h>
#include <stdint.h>

namespace bm {

template <uint8_t N>
class RingBuffer
{
public:
    RingBuffer() { reset(); }

    void reset()
    {
        head_ = 0;
        count_ = 0;
        for (uint8_t i = 0; i < N; ++i)
        {
            data_[i] = 0.0f;
        }
    }

    void push(float v)
    {
        data_[head_] = v;
        head_ = static_cast<uint8_t>((head_ + 1) % N);
        if (count_ < N)
        {
            ++count_;
        }
    }

    uint8_t size() const { return count_; }
    bool full() const { return count_ == N; }
    static constexpr uint8_t capacity() { return N; }

    float mean() const
    {
        if (count_ == 0)
        {
            return 0.0f;
        }
        float s = 0.0f;
        for (uint8_t i = 0; i < count_; ++i)
        {
            s += data_[i];
        }
        return s / static_cast<float>(count_);
    }

    float min() const
    {
        if (count_ == 0)
        {
            return 0.0f;
        }
        float m = data_[0];
        for (uint8_t i = 1; i < count_; ++i)
        {
            if (data_[i] < m)
            {
                m = data_[i];
            }
        }
        return m;
    }

    float max() const
    {
        if (count_ == 0)
        {
            return 0.0f;
        }
        float m = data_[0];
        for (uint8_t i = 1; i < count_; ++i)
        {
            if (data_[i] > m)
            {
                m = data_[i];
            }
        }
        return m;
    }

private:
    float   data_[N];
    uint8_t head_;
    uint8_t count_;
};

} // namespace bm
