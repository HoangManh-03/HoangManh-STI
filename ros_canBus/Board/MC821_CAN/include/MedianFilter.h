#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include <Arduino.h>

template <typename T, size_t N>
class MedianFilter {
public:
    MedianFilter() : index(0), count(0) {}

    void addValue(T value) {
        buffer[index] = value;
        index = (index + 1) % N;
        if (count < N) count++;
    }

    T getMedian() {
        if (count == 0) return 0;
        T sorted[N];
        for (size_t i = 0; i < count; i++) {
            sorted[i] = buffer[i];
        }

        // Sort the array
        for (size_t i = 0; i < count - 1; i++) {
            for (size_t j = i + 1; j < count; j++) {
                if (sorted[j] < sorted[i]) {
                    T temp = sorted[i];
                    sorted[i] = sorted[j];
                    sorted[j] = temp;
                }
            }
        }

        if (count % 2 == 1) {
            return sorted[count / 2];
        } else {
            return (sorted[count / 2 - 1] + sorted[count / 2]) / 2;
        }
    }

private:
    T buffer[N];
    size_t index;
    size_t count;
};

#endif
