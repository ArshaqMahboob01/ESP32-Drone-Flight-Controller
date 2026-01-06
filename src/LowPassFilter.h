#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#include <Arduino.h>

// First-order IIR low-pass filter
class LowPassFilter {
public:
    LowPassFilter() : alpha(1.0f), output(0.0f), initialized(false) {}
    
    // Initialize with cutoff frequency and sample rate
    void begin(float cutoffHz, float sampleHz) {
        float rc = 1.0f / (2.0f * PI * cutoffHz);
        float dt = 1.0f / sampleHz;
        alpha = dt / (rc + dt);
        output = 0.0f;
        initialized = false;
    }
    
    // Update filter with new input
    float update(float input) {
        if (!initialized) {
            output = input;
            initialized = true;
            return output;
        }
        output = output + alpha * (input - output);
        return output;
    }
    
    // Get current output without updating
    float getOutput() const { return output; }
    
    // Reset filter state
    void reset() {
        output = 0.0f;
        initialized = false;
    }
    
    // Set alpha directly (for custom tuning)
    void setAlpha(float a) { alpha = a; }

private:
    float alpha;
    float output;
    bool initialized;
};

#endif // LOW_PASS_FILTER_H
