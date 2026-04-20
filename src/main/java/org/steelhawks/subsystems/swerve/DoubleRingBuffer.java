package org.steelhawks.subsystems.swerve;

/**
 * A fixed-capacity, zero-allocation ring buffer for primitive doubles.
 * Replaces Queue&lt;Double&gt; to eliminate autoboxing GC pressure in the odometry thread.
 */
public class DoubleRingBuffer {
    private final double[] buffer;
    private int head = 0;
    private int tail = 0;
    private int size = 0;

    public DoubleRingBuffer(int capacity) {
        buffer = new double[capacity];
    }

    /** Adds a value. Returns false and drops the value if the buffer is full. */
    public boolean offer(double value) {
        if (size == buffer.length) return false;
        buffer[head] = value;
        head = (head + 1) % buffer.length;
        size++;
        return true;
    }

    /** Removes and returns the oldest value. Returns 0.0 if empty. */
    public double poll() {
        if (size == 0) return 0.0;
        double value = buffer[tail];
        tail = (tail + 1) % buffer.length;
        size--;
        return value;
    }

    public int size() {
        return size;
    }

    public boolean isEmpty() {
        return size == 0;
    }

    public void clear() {
        head = tail = size = 0;
    }
}
