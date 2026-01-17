package org.steelhawks.subsystems.led;

public final class Color {
    public final int r, g, b;

    public Color(int r, int g, int b) {
        this.r = Math.max(0, Math.min(255, r));
        this.g = Math.max(0, Math.min(255, g));
        this.b = Math.max(0, Math.min(255, b));
    }

    public static final Color RED = new Color(255, 0, 0);
    public static final Color GREEN = new Color(0, 255, 0);
    public static final Color BLUE = new Color(0, 0, 255);
    public static final Color WHITE = new Color(255, 255, 255);
    public static final Color BLACK = new Color(0, 0, 0);
    public static final Color YELLOW = new Color(255, 255, 0);
    public static final Color CYAN = new Color(0, 255, 255);
    public static final Color MAGENTA = new Color(255, 0, 255);
    public static final Color ORANGE = new Color(255, 165, 0);
    public static final Color PURPLE = new Color(128, 0, 128);
    public static final Color PINK = new Color(255, 192, 203);
    public static final Color OFF = new Color(0, 0, 0);

    public Color blend(Color other, double ratio) {
        ratio = Math.max(0, Math.min(1, ratio));
        return new Color(
            (int)(this.r * (1 - ratio) + other.r * ratio),
            (int)(this.g * (1 - ratio) + other.g * ratio),
            (int)(this.b * (1 - ratio) + other.b * ratio)
        );
    }

    public Color dim(double brightness) {
        brightness = Math.max(0, Math.min(1, brightness));
        return new Color(
            (int)(r * brightness),
            (int)(g * brightness),
            (int)(b * brightness)
        );
    }
}