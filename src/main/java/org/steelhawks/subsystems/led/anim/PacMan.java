package org.steelhawks.subsystems.led.anim;

import org.steelhawks.subsystems.led.Color;
import org.steelhawks.subsystems.led.LEDMatrix;

/**
 * Pac-Man Animation for LED Matrix
 */
public class PacMan extends LEDMatrix.Animation {

    private final int width;
    private final int height;
    private int pacmanX = 0;
    private int ghostX = 0;
    private boolean mouthOpen = true;
    private int mouthCounter = 0;

    public PacMan(int width, int height) {
        this.width = width;
        this.height = height;
        this.pacmanX = -3;
        this.ghostX = -8;
    }

    @Override
    public void render(LEDMatrix matrix) {
        matrix.clear();

        int centerY = height / 2;

        // Toggle mouth every 3 frames
        if (frameCount % 3 == 0) {
            mouthOpen = !mouthOpen;
        }

        // Draw dots for Pac-Man to eat
        for (int x = 0; x < width; x += 4) {
            if (x > pacmanX + 3 && x >= 0 && x < width) { // Only show dots ahead of Pac-Man
                matrix.setPixel(x, centerY, Color.WHITE);
            }
        }

        // Draw Ghost first (so Pac-Man appears on top)
        drawGhost(matrix, ghostX, centerY);

        // Draw Pac-Man
        drawPacMan(matrix, pacmanX, centerY, mouthOpen);

        // Move Pac-Man forward every 2 frames
        if (frameCount % 2 == 0) {
            pacmanX++;
            if (pacmanX > width + 3) {
                pacmanX = -3;
            }
        }

        // Move Ghost slower (every 3 frames), always behind Pac-Man
        if (frameCount % 3 == 0) {
            ghostX++;
            if (ghostX > width + 3) {
                ghostX = -8; // Start further back
            }
        }
    }

    private void drawPacMan(LEDMatrix matrix, int x, int y, boolean mouthOpen) {
        Color yellow = new Color(255, 255, 0);

        // Bounds check helper
        if (x < 0 || x >= width) return;

        if (height >= 16) {
            // Large Pac-Man for 16x16 (3x3 sprite)
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (mouthOpen && dy == 0 && dx == 1) {
                        continue; // Mouth open (skip right-center pixel)
                    }
                    int px = x + dx;
                    int py = y + dy;
                    if (px >= 0 && px < width && py >= 0 && py < height) {
                        matrix.setPixel(px, py, yellow);
                    }
                }
            }
        } else {
            // Small Pac-Man for 32x8 (2x2 sprite)
            int[][] pixels = mouthOpen
                ? new int[][]{{0, 0}, {0, -1}, {-1, 0}}
                : new int[][]{{0, 0}, {0, -1}, {-1, 0}, {1, 0}};

            for (int[] pixel : pixels) {
                int px = x + pixel[0];
                int py = y + pixel[1];
                if (px >= 0 && px < width && py >= 0 && py < height) {
                    matrix.setPixel(px, py, yellow);
                }
            }
        }
    }

    private void drawGhost(LEDMatrix matrix, int x, int y) {
        Color cyan = new Color(0, 255, 255);
        Color white = new Color(255, 255, 255);

        // Bounds check
        if (x < -2 || x >= width + 2) return;

        if (height >= 16) {
            // Large ghost for 16x16 (3x3 sprite)
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int gx = x + dx;
                    int gy = y + dy;
                    if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
                        matrix.setPixel(gx, gy, cyan);
                    }
                }
            }
            // Eyes
            if (x - 1 >= 0 && x - 1 < width && y - 1 >= 0 && y - 1 < height) {
                matrix.setPixel(x - 1, y - 1, white);
            }
            if (x + 1 >= 0 && x + 1 < width && y - 1 >= 0 && y - 1 < height) {
                matrix.setPixel(x + 1, y - 1, white);
            }
        } else {
            // Small ghost for 32x8 (2x2 sprite)
            int[][] pixels = {{0, 0}, {0, -1}, {-1, 0}, {1, 0}};

            for (int[] pixel : pixels) {
                int gx = x + pixel[0];
                int gy = y + pixel[1];
                if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
                    matrix.setPixel(gx, gy, cyan);
                }
            }
            // Eye
            if (x >= 0 && x < width && y - 1 >= 0 && y - 1 < height) {
                matrix.setPixel(x, y - 1, white);
            }
        }
    }

    @Override
    public void reset() {
        super.reset();
        pacmanX = -3;
        ghostX = -8;
        mouthOpen = true;
    }
}