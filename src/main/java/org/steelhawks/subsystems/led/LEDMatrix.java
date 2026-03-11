package org.steelhawks.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.*;
import org.steelhawks.Constants.LEDConstants;
import org.steelhawks.subsystems.led.anim.FrameAnimation;
import org.steelhawks.subsystems.led.anim.PacMan;

/**
 * LED Matrix subsystem for controlling 2D LED arrays with animations
 * Optimized for 32x8 matrix (256 LEDs) in serpentine/zigzag layout
 */
public class LEDMatrix extends SubsystemBase {

    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final int width;
    private final int height;
    private final MatrixLayout layout;

    private Animation currentAnimation = null;
    private Animation overlayAnimation = null;
    private int animationFrame = 0;
    private double brightness = 1.0;

    private String trackedText = null;

    public void updateText(String text) {
        if (trackedText == null || !trackedText.equals(text)) {
            trackedText = text;
            if (currentAnimation instanceof ScrollingText scrollingT) {
                scrollingT.setText(text);
            }
            if (currentAnimation instanceof StaticText staticT) {
                staticT.setText(text);
            }
        }
    }

    public void setBrightness(double brightness) {
        this.brightness = Math.max(0.0, Math.min(1.0, brightness));
    }

    public enum MatrixLayout {
        SERPENTINE,  // Zigzag pattern (our LEDs use this)
        PROGRESSIVE  // All rows go same direction
    }

    /**
     * Base class for all animations
     */
    public abstract static class Animation {
        protected int frameCount = 0;

        /**
         * Render the animation to the matrix
         * @param matrix The LED matrix to render to
         */
        public abstract void render(LEDMatrix matrix);

        /**
         * Reset animation state
         */
        public void reset() {
            frameCount = 0;
        }

        /**
         * Called each frame
         */
        protected void tick() {
            frameCount++;
        }
    }

    /**
     * Constructor for LED Matrix
     * @param port PWM port for LED control
     * @param width Matrix width in pixels
     * @param height Matrix height in pixels
     * @param layout Wiring layout (SERPENTINE recommended)
     */
    public LEDMatrix(int port, int width, int height, MatrixLayout layout) {
        this.width = width;
        this.height = height;
        this.layout = layout;

        ledStrip = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(width * height);

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    public LEDMatrix(int width, int height) {
        this(LEDConstants.PORT, width, height, MatrixLayout.SERPENTINE);
    }

    public LEDMatrix() {
        this(16, 16);
    }

    /**
     * Convert 2D coordinates to 1D buffer index
     * Based on Row Major, Upper Left start, Serpentine wiring
     */
    private int getPixelIndex(int x, int y) {
        if (x < 0 || x >= width || y < 0 || y >= height) {
            return -1;
        }

        if (layout == MatrixLayout.SERPENTINE) {
            if (x % 2 == 0) {
                return (x * height) + y;
            } else {
                return (x * height) + (height - 1 - y);
            }
        } else {
            return y * width + x;
        }
    }

    /**
     * Set a single pixel color
     */
    public void setPixel(int x, int y, Color color) {
        int index = getPixelIndex(x, y);
        if (index >= 0) {
            ledBuffer.setRGB(index,
                (int)(color.r * brightness),
                (int)(color.g * brightness),
                (int)(color.b * brightness));
        }
    }

    /**
     * Get pixel color at coordinates
     */
    public Color getPixel(int x, int y) {
        int index = getPixelIndex(x, y);
        if (index >= 0) {
            return new Color(
                ledBuffer.getRed(index),
                ledBuffer.getGreen(index),
                ledBuffer.getBlue(index)
            );
        }
        return Color.BLACK;
    }

    /**
     * Fill entire matrix with a color
     */
    public void fill(Color color) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, color.r, color.g, color.b);
        }
    }

    /**
     * Clear the matrix (set all to black)
     */
    public void clear() {
        fill(Color.BLACK);
    }

    /**
     * Draw a horizontal line
     */
    public void drawHLine(int y, int x1, int x2, Color color) {
        for (int x = Math.min(x1, x2); x <= Math.max(x1, x2); x++) {
            setPixel(x, y, color);
        }
    }

    /**
     * Draw a vertical line
     */
    public void drawVLine(int x, int y1, int y2, Color color) {
        for (int y = Math.min(y1, y2); y <= Math.max(y1, y2); y++) {
            setPixel(x, y, color);
        }
    }

    /**
     * Draw a rectangle outline
     */
    public void drawRect(int x, int y, int w, int h, Color color) {
        drawHLine(y, x, x + w - 1, color);
        drawHLine(y + h - 1, x, x + w - 1, color);
        drawVLine(x, y, y + h - 1, color);
        drawVLine(x + w - 1, y, y + h - 1, color);
    }

    /**
     * Draw a filled rectangle
     */
    public void fillRect(int x, int y, int w, int h, Color color) {
        for (int dy = 0; dy < h; dy++) {
            for (int dx = 0; dx < w; dx++) {
                setPixel(x + dx, y + dy, color);
            }
        }
    }

    /**
     * Update the physical LEDs with buffer data
     */
    public void show() {
        ledStrip.setData(ledBuffer);
    }

    /**
     * Play an animation
     */
    public void playAnimation(Animation animation) {
        this.currentAnimation = animation;
        animation.reset();
    }



    public void playOverlay(Animation animation) {
        this.overlayAnimation = animation;
        animation.reset();
    }

    public void clearOverlay() {
        this.overlayAnimation = null;
    }

    public Animation getOverlayAnimation() { return overlayAnimation; }

    @Override
    public void periodic() {
        if (currentAnimation != null) {
            currentAnimation.render(this);
            currentAnimation.tick();
        }
        if (overlayAnimation != null) {
            overlayAnimation.render(this);
            overlayAnimation.tick();
        }
        show();
    }

    private static boolean[][] getCharPattern(char c) {
        // 5 rows x 4 columns for each character
        boolean[][] pattern = new boolean[5][4];

        switch (c) {
            case 'A':
                pattern[0] = new boolean[]{false, true, true, false};
                pattern[1] = new boolean[]{true, false, false, true};
                pattern[2] = new boolean[]{true, true, true, true};
                pattern[3] = new boolean[]{true, false, false, true};
                pattern[4] = new boolean[]{true, false, false, true};
                break;
            case 'B':
                pattern[0] = new boolean[]{true, true, true, false};
                pattern[1] = new boolean[]{true, false, false, true};
                pattern[2] = new boolean[]{true, true, true, false};
                pattern[3] = new boolean[]{true, false, false, true};
                pattern[4] = new boolean[]{true, true, true, false};
                break;
            case 'C':
                pattern[0] = new boolean[]{false, true, true, true};
                pattern[1] = new boolean[]{true, false, false, false};
                pattern[2] = new boolean[]{true, false, false, false};
                pattern[3] = new boolean[]{true, false, false, false};
                pattern[4] = new boolean[]{false, true, true, true};
                break;
            case 'D':
                pattern[0] = new boolean[]{true, true, true, false};
                pattern[1] = new boolean[]{true, false, false, true};
                pattern[2] = new boolean[]{true, false, false, true};
                pattern[3] = new boolean[]{true, false, false, true};
                pattern[4] = new boolean[]{true, true, true, false};
                break;
            case 'E':
                pattern[0] = new boolean[]{true, true, true, true};
                pattern[1] = new boolean[]{true, false, false, false};
                pattern[2] = new boolean[]{true, true, true, false};
                pattern[3] = new boolean[]{true, false, false, false};
                pattern[4] = new boolean[]{true, true, true, true};
                break;
            case 'F':
                pattern[0] = new boolean[]{true, true, true, true};
                pattern[1] = new boolean[]{true, false, false, false};
                pattern[2] = new boolean[]{true, true, true, false};
                pattern[3] = new boolean[]{true, false, false, false};
                pattern[4] = new boolean[]{true, false, false, false};
                break;
            case 'G':
                pattern[0] = new boolean[]{false, true, true, true};
                pattern[1] = new boolean[]{true, false, false, false};
                pattern[2] = new boolean[]{true, false, true, true};
                pattern[3] = new boolean[]{true, false, false, true};
                pattern[4] = new boolean[]{false, true, true, true};
                break;
            case 'H':
                pattern[0] = new boolean[]{true, false, false, true};
                pattern[1] = new boolean[]{true, false, false, true};
                pattern[2] = new boolean[]{true, true, true, true};
                pattern[3] = new boolean[]{true, false, false, true};
                pattern[4] = new boolean[]{true, false, false, true};
                break;
            case 'I':
                pattern[0] = new boolean[]{true, true, true, false};
                pattern[1] = new boolean[]{false, true, false, false};
                pattern[2] = new boolean[]{false, true, false, false};
                pattern[3] = new boolean[]{false, true, false, false};
                pattern[4] = new boolean[]{true, true, true, false};
                break;
            case 'J': //jqvyz23456789!
                pattern[0] = new boolean[]{true, true, true, true};
                pattern[1] = new boolean[]{false, false, true, false};
                pattern[2] = new boolean[]{false, false, true, false};
                pattern[3] = new boolean[]{true, false, true, false};
                pattern[4] = new boolean[]{false, true, false, false};
                break;
            case 'K':
                pattern[0] = new boolean[]{true, false, false, true};
                pattern[1] = new boolean[]{true, false, true, false};
                pattern[2] = new boolean[]{true, true, false, false};
                pattern[3] = new boolean[]{true, false, true, false};
                pattern[4] = new boolean[]{true, false, false, true};
                break;
            case 'L':
                pattern[0] = new boolean[]{true, false, false, false};
                pattern[1] = new boolean[]{true, false, false, false};
                pattern[2] = new boolean[]{true, false, false, false};
                pattern[3] = new boolean[]{true, false, false, false};
                pattern[4] = new boolean[]{true, true, true, true};
                break;
            case 'M':
                pattern[0] = new boolean[]{true, false, false, true};
                pattern[1] = new boolean[]{true, true, true, true};
                pattern[2] = new boolean[]{true, false, false, true};
                pattern[3] = new boolean[]{true, false, false, true};
                pattern[4] = new boolean[]{true, false, false, true};
                break;
            case 'N':
                pattern[0] = new boolean[]{true, false, false, true};
                pattern[1] = new boolean[]{true, true, false, true};
                pattern[2] = new boolean[]{true, false, true, true};
                pattern[3] = new boolean[]{true, false, false, true};
                pattern[4] = new boolean[]{true, false, false, true};
                break;
            case 'O':
                pattern[0] = new boolean[]{false, true, true, false};
                pattern[1] = new boolean[]{true, false, false, true};
                pattern[2] = new boolean[]{true, false, false, true};
                pattern[3] = new boolean[]{true, false, false, true};
                pattern[4] = new boolean[]{false, true, true, false};
                break;
            case 'P':
                pattern[0] = new boolean[]{true, true, true, false};
                pattern[1] = new boolean[]{true, false, false, true};
                pattern[2] = new boolean[]{true, true, true, false};
                pattern[3] = new boolean[]{true, false, false, false};
                pattern[4] = new boolean[]{true, false, false, false};
                break;
            case 'Q':
                pattern[0] = new boolean[]{false, true, true, false};
                pattern[1] = new boolean[]{true, false, false, true};
                pattern[2] = new boolean[]{true, true, false, true};
                pattern[3] = new boolean[]{true, false, true, true};
                pattern[4] = new boolean[]{false, true, true, true};
                break;
            case 'R':
                pattern[0] = new boolean[]{true, true, true, false};
                pattern[1] = new boolean[]{true, false, false, true};
                pattern[2] = new boolean[]{true, true, true, false};
                pattern[3] = new boolean[]{true, false, true, false};
                pattern[4] = new boolean[]{true, false, false, true};
                break;
            case 'S':
                pattern[0] = new boolean[]{false, true, true, true};
                pattern[1] = new boolean[]{true, false, false, false};
                pattern[2] = new boolean[]{false, true, true, false};
                pattern[3] = new boolean[]{false, false, false, true};
                pattern[4] = new boolean[]{true, true, true, false};
                break;
            case 'T':
                pattern[0] = new boolean[]{true, true, true, true};
                pattern[1] = new boolean[]{false, true, false, false};
                pattern[2] = new boolean[]{false, true, false, false};
                pattern[3] = new boolean[]{false, true, false, false};
                pattern[4] = new boolean[]{false, true, false, false};
                break;
            case 'U':
                pattern[0] = new boolean[]{true, false, false, true};
                pattern[1] = new boolean[]{true, false, false, true};
                pattern[2] = new boolean[]{true, false, false, true};
                pattern[3] = new boolean[]{true, false, false, true};
                pattern[4] = new boolean[]{false, true, true, false};
                break;
            case 'V':
                pattern[0] = new boolean[]{true, false, true, false};
                pattern[1] = new boolean[]{true, false, true, false};
                pattern[2] = new boolean[]{true, false, true, false};
                pattern[3] = new boolean[]{false, true, false, false};
                pattern[4] = new boolean[]{false, true, false, false};
                break;
            case 'W':
                pattern[0] = new boolean[]{true, false, false, true};
                pattern[1] = new boolean[]{true, false, false, true};
                pattern[2] = new boolean[]{true, false, false, true};
                pattern[3] = new boolean[]{true, true, true, true};
                pattern[4] = new boolean[]{true, false, false, true};
                break;
            case 'X':
                pattern[0] = new boolean[]{true, false, false, true};
                pattern[1] = new boolean[]{false, true, true, false};
                pattern[2] = new boolean[]{false, true, true, false};
                pattern[3] = new boolean[]{false, true, true, false};
                pattern[4] = new boolean[]{true, false, false, true};
                break;
            case 'Y':
                pattern[0] = new boolean[]{true, false, true, false};
                pattern[1] = new boolean[]{true, false, true, false};
                pattern[2] = new boolean[]{false, true, false, false};
                pattern[3] = new boolean[]{false, true, false, false};
                pattern[4] = new boolean[]{false, true, false, false};
                break;
            case 'Z':
                pattern[0] = new boolean[]{true, true, true, true};
                pattern[1] = new boolean[]{false, false, false, true };
                pattern[2] = new boolean[]{false, false, true, false};
                pattern[3] = new boolean[]{false, true, false, false};
                pattern[4] = new boolean[]{true, true, true, true};
                break;
            case '0':
                pattern[0] = new boolean[]{false, true, true, false};
                pattern[1] = new boolean[]{true, false, false, true};
                pattern[2] = new boolean[]{true, false, false, true};
                pattern[3] = new boolean[]{true, false, false, true};
                pattern[4] = new boolean[]{false, true, true, false};
                break;
            case '1':
                pattern[0] = new boolean[]{false, true, false, false};
                pattern[1] = new boolean[]{true, true, false, false};
                pattern[2] = new boolean[]{false, true, false, false};
                pattern[3] = new boolean[]{false, true, false, false};
                pattern[4] = new boolean[]{true, true, true, false};
                break;
            case '2':
                pattern[0] = new boolean[]{false, true, true, false};
                pattern[1] = new boolean[]{true, false, false, true};
                pattern[2] = new boolean[]{false, false, true, false};
                pattern[3] = new boolean[]{false, true, false, false};
                pattern[4] = new boolean[]{true, true, true, true};
                break;
            case '3':
                pattern[0] = new boolean[]{true, true, true, false};
                pattern[1] = new boolean[]{false, false, false, true};
                pattern[2] = new boolean[]{false, false, true, false};
                pattern[3] = new boolean[]{false, false, false, true};
                pattern[4] = new boolean[]{true, true, true, false};
                break;
            case '4':
                pattern[0] = new boolean[]{true, false, true, false};
                pattern[1] = new boolean[]{true, false, true, false};
                pattern[2] = new boolean[]{true, true, true, false};
                pattern[3] = new boolean[]{false, false, true, false};
                pattern[4] = new boolean[]{false, false, true, false};
                break;
            case '5':
                pattern[0] = new boolean[]{true, true, true, true};
                pattern[1] = new boolean[]{true, false, false, false};
                pattern[2] = new boolean[]{true, true, true, true};
                pattern[3] = new boolean[]{false, false, false, true};
                pattern[4] = new boolean[]{true, true, true, false};
                break;
            case '6':
                pattern[0] = new boolean[]{false, true, true, true};
                pattern[1] = new boolean[]{true, false, false, false};
                pattern[2] = new boolean[]{true, true, true, false};
                pattern[3] = new boolean[]{true, false, false, true};
                pattern[4] = new boolean[]{false, true, true, false};
                break;
            case '7':
                pattern[0] = new boolean[]{true, true, true, true};
                pattern[1] = new boolean[]{false, false, true, false};
                pattern[2] = new boolean[]{false, true, false, false};
                pattern[3] = new boolean[]{false, true, false, false};
                pattern[4] = new boolean[]{false, true, false, false};
                break;
            case '8':
                pattern[0] = new boolean[]{false, true, true, false};
                pattern[1] = new boolean[]{true, false, false, true};
                pattern[2] = new boolean[]{false, true, true, false};
                pattern[3] = new boolean[]{true, false, false, true};
                pattern[4] = new boolean[]{false, true, true, false};
                break;
            case '9':
                pattern[0] = new boolean[]{false, true, true, true};
                pattern[1] = new boolean[]{true, false, false, true};
                pattern[2] = new boolean[]{false, true, true, true};
                pattern[3] = new boolean[]{false, false, false, true};
                pattern[4] = new boolean[]{false, false, false, true};
                break;
            case '!':
                pattern[0] = new boolean[]{true, false, false, false};
                pattern[1] = new boolean[]{true, false, false, false};
                pattern[2] = new boolean[]{true, false, false, false};
                pattern[3] = new boolean[]{false, false, false, false};
                pattern[4] = new boolean[]{true, false, false, false};
                break;
            case ' ':
                // blank space
                break;
            case ':':
                pattern[1][1] = true;
                pattern[3][1] = true;
                break;
            default:
                // Unknown character, show a small dot
                pattern[2][1] = true;
                break;
        }

        return pattern;
    }

    // ==================== BUILT-IN ANIMATIONS ====================

    /**
     * Solid color fill
     */
    public static class SolidColor extends Animation {
        private final Color color;

        public SolidColor(Color color) {
            this.color = color;
        }

        @Override
        public void render(LEDMatrix matrix) {
            matrix.fill(color);
        }
    }

    public static class FlashColor extends Animation {
        private final Color color;
        private final double interval;
        private double lastChange = 0;
        private boolean isOn = false;

        public FlashColor(Color color, double interval) {
            this.color = color;
            this.interval = interval;
        }

        @Override
        public void render(LEDMatrix matrix) {
            double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

            if (currentTime - lastChange > interval) {
                lastChange = currentTime;
                isOn = !isOn;
            }

            if (isOn) {
                matrix.fill(color);
            } else {
                matrix.clear();
            }
        }

        @Override
        public void reset() {
            super.reset();
            lastChange = 0;
            isOn = false;
        }
    }

    /**
     * Rainbow wave across the matrix
     */
    public static class RainbowWave extends Animation {
        private final int speed;

        public RainbowWave(int speed) {
            this.speed = speed;
        }

        @Override
        public void render(LEDMatrix matrix) {
            for (int y = 0; y < matrix.height; y++) {
                for (int x = 0; x < matrix.width; x++) {
                    int hue = (x * 180 / matrix.width + frameCount * speed) % 180;
                    matrix.ledBuffer.setHSV(matrix.getPixelIndex(x, y), hue, 255, 128);
                }
            }
        }
    }

    /**
     * Scrolling text (simplified - displays characters as 5x7 bitmaps)
     */
    public static class ScrollingText extends Animation {
        private String text;
        private final Color color;
        private final int speed;
        private int offset = 0;
        private final int startX;
        private final int regionWidth; // -1 = full matrix width
        private static final int CHAR_WIDTH = 4;
        private static final int CHAR_SPACING = 1;

        /** Full-width scrolling text */
        public ScrollingText(String text, Color color, int speed) {
            this.text = text.toUpperCase();
            this.color = color;
            this.speed = speed;
            this.startX = 0;
            this.regionWidth = -1;
        }

        /** Constrained to a region of the matrix */
        public ScrollingText(String text, Color color, int speed, int startX, int regionWidth) {
            this.text = text.toUpperCase();
            this.color = color;
            this.speed = speed;
            this.startX = startX;
            this.regionWidth = regionWidth;
        }

        @Override
        public void render(LEDMatrix matrix) {
            int effWidth = regionWidth > 0 ? regionWidth : matrix.width;
            int effStart = startX;

            // clear our region
            for (int x = effStart; x < effStart + effWidth; x++) {
                for (int y = 0; y < matrix.height; y++) {
                    matrix.setPixel(x, y, Color.BLACK);
                }
            }

            if (frameCount % speed == 0) {
                offset++;
                if (offset > text.length() * (CHAR_WIDTH + CHAR_SPACING) + effWidth) {
                    offset = 0;
                }
            }

            int charY = (matrix.height - 5) / 2;
            for (int i = 0; i < text.length(); i++) {
                int charX = effStart + effWidth - offset + i * (CHAR_WIDTH + CHAR_SPACING);
                if (charX > effStart - CHAR_WIDTH && charX < effStart + effWidth) {
                    drawChar(matrix, text.charAt(i), charX, charY, color);
                }
            }
        }

        public String getText() { return text; }

        public void setText(String newText) {
            this.text = newText.toUpperCase();
            this.offset = 0;
        }

        private void drawChar(LEDMatrix matrix, char c, int x, int y, Color color) {
            boolean[][] pattern = getCharPattern(c);
            for (int dy = 0; dy < 5; dy++) {
                for (int dx = 0; dx < CHAR_WIDTH; dx++) {
                    if (pattern[dy][dx]) {
                        matrix.setPixel(x + dx, y + dy, color);
                    }
                }
            }
        }
    }

    public static class StaticText extends Animation {
        private String text;
        private final Color color;
        private int offset = 0;
        private static final int CHAR_WIDTH = 4;
        private static final int CHAR_SPACING = 1;

        public StaticText(String text, Color color) {
            this.text = text.toUpperCase();
            this.color = color;
        }

        @Override
        public void render(LEDMatrix matrix) {
            matrix.clear();

            for (int i = 0; i < text.length(); i++) {
                int charX = (matrix.width / 2) - ((text.length() * CHAR_WIDTH + (text.length() - 1) * CHAR_SPACING) / 2) + (i * (CHAR_WIDTH + CHAR_SPACING));
                int charY = (matrix.height - 5) / 2;
                drawChar(matrix, text.charAt(i), charX, charY, color);

                if (charX > -CHAR_WIDTH && charX < matrix.width) {
                    drawChar(matrix, text.charAt(i), charX, charY, color);
                }
            }
        }

        public String getText() {
            return text;
        }

        public void setText(String newText) {
            this.text = newText.toUpperCase();
        }

        private void drawChar(LEDMatrix matrix, char c, int x, int y, Color color) {
            boolean[][] pattern = getCharPattern(c);

            for (int dy = 0; dy < 5; dy++) {
                for (int dx = 0; dx < CHAR_WIDTH; dx++) {
                    if (pattern[dy][dx]) {
                        matrix.setPixel(x + dx, y + dy, color);
                    }
                }
            }
        }
    }

    public static class DirectionalArrow extends Animation {
        public enum Direction { LEFT, RIGHT, UP, DOWN, NONE }

        private final Direction direction;
        private double oscillateInterval;
        private double lastToggle = 0;
        private boolean isOn = true;
        private final Color color;
        private final int startX;
        private final int regionWidth;

        public DirectionalArrow(Direction direction, double oscillateInterval, Color color, int startX, int regionWidth) {
            this.direction = direction;
            this.oscillateInterval = oscillateInterval;
            this.color = color;
            this.startX = startX;
            this.regionWidth = regionWidth;
        }

        public void setOscillateInterval(double interval) {
            this.oscillateInterval = interval;
        }

        public Direction getDirection() { return direction; }

        @Override
        public void render(LEDMatrix matrix) {
            double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            if (now - lastToggle > oscillateInterval) {
                lastToggle = now;
                isOn = !isOn;
            }
            for (int x = startX; x < startX + regionWidth; x++) {
                for (int y = 0; y < matrix.getHeight(); y++) {
                    matrix.setPixel(x, y, Color.BLACK);
                }
            }

            if (!isOn || direction == Direction.NONE) return;

            int cx = startX + regionWidth / 2;
            int cy = matrix.getHeight() / 2;

            switch (direction) {
                case RIGHT -> {
                    matrix.setPixel(cx - 1, cy - 3, color);
                    matrix.setPixel(cx, cy - 2, color);
                    matrix.setPixel(cx + 1, cy - 1, color);
                    matrix.setPixel(cx + 2, cy, color);
                    matrix.setPixel(cx + 1, cy + 1, color);
                    matrix.setPixel(cx, cy + 2, color);
                    matrix.setPixel(cx - 1, cy + 3, color);
                }
                case LEFT -> {
                    //
                    matrix.setPixel(cx + 1, cy - 3, color);
                    matrix.setPixel(cx, cy - 2, color);
                    matrix.setPixel(cx - 1, cy - 1, color);
                    matrix.setPixel(cx - 2, cy, color);
                    matrix.setPixel(cx - 1, cy + 1, color);
                    matrix.setPixel(cx, cy + 2, color);
                    matrix.setPixel(cx + 1, cy + 3, color);
                }
                case UP -> {
                    // ^
                    matrix.setPixel(cx, cy - 2, color);
                    matrix.setPixel(cx - 1, cy - 1, color);
                    matrix.setPixel(cx + 1, cy - 1, color);
                    matrix.setPixel(cx - 2, cy, color);
                    matrix.setPixel(cx + 2, cy, color);
                    matrix.setPixel(cx - 3, cy + 1, color);
                    matrix.setPixel(cx + 3, cy + 1, color);
                }
                case DOWN -> {
                    // v
                    matrix.setPixel(cx, cy + 2, color);
                    matrix.setPixel(cx - 1, cy + 1, color);
                    matrix.setPixel(cx + 1, cy + 1, color);
                    matrix.setPixel(cx - 2, cy, color);
                    matrix.setPixel(cx + 2, cy, color);
                    matrix.setPixel(cx - 3, cy - 1, color);
                    matrix.setPixel(cx + 3, cy - 1, color);
                }
            }
        }

        @Override
        public void reset() {
            super.reset();
            lastToggle = 0;
            isOn = true;
        }
    }

    /**
     * Fire/flame effect
     */
    public static class Fire extends Animation {
        private final int cooling;
        private final int sparking;
        private int[][] heat;
        private boolean initialized = false;

        public Fire(int cooling, int sparking) {
            this.cooling = cooling;
            this.sparking = sparking;
        }

        @Override
        public void render(LEDMatrix matrix) {
            // Initialize heat array on first render
            if (!initialized) {
                heat = new int[matrix.width][matrix.height];
                initialized = true;
            }

            // Cool down every cell
            for (int x = 0; x < matrix.width; x++) {
                for (int y = 0; y < matrix.height; y++) {
                    int cooldown = (int)(Math.random() * ((double) (cooling * 10) / matrix.height));
                    heat[x][y] = Math.max(0, heat[x][y] - cooldown);
                }
            }

            // Heat rises - average with cells below
            for (int x = 0; x < matrix.width; x++) {
                for (int y = 0; y < matrix.height - 1; y++) {
                    heat[x][y] = (heat[x][y + 1] + heat[x][Math.min(y + 2, matrix.height - 1)]) / 2;
                }
            }

            // Randomly ignite new sparks at bottom
            for (int x = 0; x < matrix.width; x++) {
                if (Math.random() * 255 < sparking) {
                    int y = matrix.height - 1;
                    heat[x][y] = Math.min(255, heat[x][y] + (int)(Math.random() * 160) + 96);
                }
            }

            // Convert heat to color
            for (int x = 0; x < matrix.width; x++) {
                for (int y = 0; y < matrix.height; y++) {
                    matrix.setPixel(x, y, heatColor(heat[x][y]));
                }
            }
        }

        private Color heatColor(int temperature) {
            // Scale temperature to RGB
            int t = temperature;

            // Black -> Red -> Orange -> Yellow -> White
            if (t < 85) {
                return new Color(t * 3, 0, 0);
            } else if (t < 170) {
                int adjust = t - 85;
                return new Color(255, adjust * 3, 0);
            } else {
                int adjust = t - 170;
                return new Color(255, 255, adjust * 3);
            }
        }

        @Override
        public void reset() {
            super.reset();
            initialized = false;
        }
    }

    /**
     * Plasma effect
     */
    public static class Plasma extends Animation {
        private final double speed;

        public Plasma(double speed) {
            this.speed = speed;
        }

        @Override
        public void render(LEDMatrix matrix) {
            double time = frameCount * speed;

            for (int y = 0; y < matrix.height; y++) {
                for (int x = 0; x < matrix.width; x++) {
                    double v1 = Math.sin(x / 2.0 + time);
                    double v2 = Math.sin((x + y) / 3.0 + time);
                    double v3 = Math.sin(Math.sqrt(x * x + y * y) / 2.0 + time);
                    double v = (v1 + v2 + v3) / 3.0;

                    int hue = (int)((v + 1.0) * 90) % 180;
                    matrix.ledBuffer.setHSV(matrix.getPixelIndex(x, y), hue, 255, 128);
                }
            }
        }
    }

    /**
     * Bouncing ball
     */
    public static class BouncingBall extends Animation {
        private double x, y, vx, vy;
        private final Color color;
        private final int ballSize;
        private boolean initialized = false;

        public BouncingBall(Color color, int ballSize) {
            this.color = color;
            this.ballSize = ballSize;
        }

        @Override
        public void render(LEDMatrix matrix) {
            // Initialize position and velocity based on matrix size
            if (!initialized) {
                x = matrix.width / 2.0;
                y = matrix.height / 2.0;
                // Scale velocity based on matrix dimensions
                vx = matrix.width / 50.0;
                vy = matrix.height / 20.0;
                initialized = true;
            }

            matrix.clear();

            // integrate position
            x += vx;
            y += vy;

            // Bounce off walls
            if (x <= 0 || x >= matrix.width - ballSize) {
                vx = -vx;
                x = Math.max(0, Math.min(matrix.width - ballSize, x));
            }
            if (y <= 0 || y >= matrix.height - ballSize) {
                vy = -vy;
                y = Math.max(0, Math.min(matrix.height - ballSize, y));
            }

            // Draw ball
            matrix.fillRect((int)x, (int)y, ballSize, ballSize, color);
        }

        @Override
        public void reset() {
            super.reset();
            initialized = false;
        }
    }

    // ==================== COMMAND FACTORIES ====================

    public Command solidColorCommand(Color color) {
        return Commands.runOnce(() -> playAnimation(new SolidColor(color)), this)
            .beforeStarting(() -> this.setBrightness(1.0))
            .ignoringDisable(true);
    }

    public Command rainbowWaveCommand(int speed) {
        return Commands.runOnce(() -> playAnimation(new RainbowWave(speed)), this)
            .beforeStarting(() -> this.setBrightness(1.0))
            .ignoringDisable(true);
    }

    public Command seizureCommand() {
        return rainbowWaveCommand(30);
    }

    public Command scrollingTextCommand(String text, Color color, int speed) {
        return Commands.runOnce(() -> playAnimation(new ScrollingText(text, color, speed)), this)
            .ignoringDisable(true);
    }

    public Command staticTextCommand(String text, Color color) {
        return Commands.runOnce(() -> playAnimation(new StaticText(text, color)))
            .ignoringDisable(true);
    }

    /**
     * Constructs a command that flashes the matrix. Most useful for indicators
     *
     * @param color the color to flash
     * @param interval the time between flashes in seconds
     * @param time how long to run this sequence in seconds
     */
    public Command flashCommand(Color color, double interval, double time) {
        return new ParallelDeadlineGroup(
            new WaitCommand(time),
            Commands.runOnce(() -> playAnimation(new FlashColor(color, interval)), this)
        ).ignoringDisable(true);
    }

    /**
     * Constructs a command that flashes the matrix indefinitely
     *
     * @param color the color to flash
     * @param interval the time between flashes in seconds
     */
    public Command flashCommand(Color color, double interval) {
        return Commands.runOnce(() -> playAnimation(new FlashColor(color, interval)), this);
    }

    public Command fireCommand(int cooling, int sparking) {
        return Commands.runOnce(() -> playAnimation(new Fire(cooling, sparking)), this)
            .ignoringDisable(true);
    }

    public Command plasmaCommand(double speed) {
        return Commands.runOnce(() -> playAnimation(new Plasma(speed)), this)
            .ignoringDisable(true);
    }

    public Command bouncingBallCommand(Color color, int size) {
        return Commands.runOnce(() -> playAnimation(new BouncingBall(color, size)), this)
            .ignoringDisable(true);
    }

    public Command pacmanCommand() {
        return Commands.runOnce(() -> playAnimation(new PacMan(width, height)), this)
            .ignoringDisable(true);
    }

    public Command frameAnimationCommand(FrameAnimation animation) {
        return Commands.runOnce(() -> playAnimation(animation), this)
            .ignoringDisable(true);
    }

    public Command clearCommand() {
        return Commands.runOnce(() -> {
            currentAnimation = null;
            overlayAnimation = null;
            clear();
            show();
        }, this).finallyDo(() -> {
            clear();
            show();
        }).ignoringDisable(true);
    }


    public Animation getCurrentAnimation() {
        return currentAnimation;
    }
    public int getWidth() { return width; }
    public int getHeight() { return height; }
}