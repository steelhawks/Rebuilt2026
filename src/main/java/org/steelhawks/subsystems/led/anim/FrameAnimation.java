package org.steelhawks.subsystems.led.anim;

import org.steelhawks.subsystems.led.Color;
import org.steelhawks.subsystems.led.LEDMatrix;

/**
 * LIGHTWEIGHT Frame-Based Animation for LED Matrix
 * Pre-bake your animations as frame arrays - no GIF decoding at runtime!
 */
public class FrameAnimation extends LEDMatrix.Animation {

    private final int[][][] frames; // [frame][x][y] = RGB packed int
    private final int[] frameDelays; // delay in periodic cycles
    private int currentFrameIndex = 0;
    private int frameDelayCounter = 0;
    private final boolean loop;
    private boolean finished = false;

    /**
     * Create animation from pre-computed frame data
     * @param frames Array of frames, each frame is [x][y] RGB packed as int (0xRRGGBB)
     * @param frameDelays Delay for each frame in 20ms cycles (1 = 20ms, 2 = 40ms, etc.)
     * @param loop Whether to loop
     */
    public FrameAnimation(int[][][] frames, int[] frameDelays, boolean loop) {
        this.frames = frames;
        this.frameDelays = frameDelays;
        this.loop = loop;
    }

    /**
     * Create animation with uniform frame delay
     */
    public FrameAnimation(int[][][] frames, int frameDelay, boolean loop) {
        this.frames = frames;
        this.frameDelays = new int[frames.length];
        for (int i = 0; i < frames.length; i++) {
            this.frameDelays[i] = frameDelay;
        }
        this.loop = loop;
    }

    @Override
    public void render(LEDMatrix matrix) {
        if (finished || frames.length == 0) return;

        int[][] currentFrame = frames[currentFrameIndex];

        // Draw current frame - optimized
        for (int x = 0; x < Math.min(currentFrame.length, matrix.getWidth()); x++) {
            for (int y = 0; y < Math.min(currentFrame[x].length, matrix.getHeight()); y++) {
                int rgb = currentFrame[x][y];
                int r = (rgb >> 16) & 0xFF;
                int g = (rgb >> 8) & 0xFF;
                int b = rgb & 0xFF;
                matrix.setPixel(x, y, new Color(r, g, b));
            }
        }

        // Handle frame timing
        frameDelayCounter++;
        if (frameDelayCounter >= frameDelays[currentFrameIndex]) {
            frameDelayCounter = 0;
            currentFrameIndex++;

            if (currentFrameIndex >= frames.length) {
                if (loop) {
                    currentFrameIndex = 0;
                } else {
                    finished = true;
                    currentFrameIndex = frames.length - 1;
                }
            }
        }
    }

    @Override
    public void reset() {
        super.reset();
        currentFrameIndex = 0;
        frameDelayCounter = 0;
        finished = false;
    }
}

// ==================== USAGE GUIDE ====================

/*
HOW TO CREATE CUSTOM ANIMATIONS:

Method 1: Hand-code simple animations (shown above)
-------------------------------------------------
public static FrameAnimation myAnimation() {
    int[][][] frames = new int[numFrames][width][height];

    // Fill each frame with RGB values
    frames[0][x][y] = 0xFF0000; // Red pixel at (x,y) in frame 0
    frames[1][x][y] = 0x00FF00; // Green pixel

    return new FrameAnimation(frames, frameDelay, loop);
}


Method 2: Use Python script to convert GIF → Java array
--------------------------------------------------------
Create a Python script to pre-process GIFs into frame arrays:

```python
from PIL import Image
import os

def gif_to_java_frames(gif_path, output_name):
    img = Image.open(gif_path)
    frames = []

    try:
        while True:
            # Resize to target size
            resized = img.resize((32, 8), Image.NEAREST)
            rgb_img = resized.convert('RGB')

            # Extract pixels
            frame_data = []
            for x in range(32):
                col = []
                for y in range(8):
                    r, g, b = rgb_img.getpixel((x, y))
                    rgb_int = (r << 16) | (g << 8) | b
                    col.append(f"0x{rgb_int:06X}")
                frame_data.append(col)
            frames.append(frame_data)

            img.seek(img.tell() + 1)
    except EOFError:
        pass

    # Generate Java code
    java_code = f"public static FrameAnimation {output_name}() {{\n"
    java_code += f"    int[][][] frames = new int[{len(frames)}][32][8];\n\n"

    for i, frame in enumerate(frames):
        java_code += f"    // Frame {i}\n"
        for x, col in enumerate(frame):
            java_code += f"    frames[{i}][{x}] = new int[]{{ {', '.join(col)} }};\n"
        java_code += "\n"

    java_code += "    return new FrameAnimation(frames, 2, true);\n"
    java_code += "}\n"

    return java_code

# Usage:
print(gif_to_java_frames("fire.gif", "fireAnimation"))
```

Then copy-paste the output into AnimationLibrary.java


Method 3: Load from CSV at robot init (one-time load)
------------------------------------------------------
Create CSV files with frame data, load once at startup:

frames.csv format:
frame,x,y,r,g,b
0,0,0,255,0,0
0,0,1,255,0,0
...

Then load in robot init (NOT during match):
```java
public static FrameAnimation loadFromCSV(String path) {
    // Parse CSV and build frame array
    // This only runs ONCE at robot startup
}
```


USAGE IN CODE:
--------------
private void configureBindings() {
    // Pre-made animations (zero overhead)
    driver.a().onTrue(leds.frameAnimationCommand(
        AnimationLibrary.simpleBlinkRed8x8()
    ));

    driver.b().onTrue(leds.frameAnimationCommand(
        AnimationLibrary.scanningLine32x8()
    ));
}


PERFORMANCE:
-----------
✅ Pre-computed frames: Fast
✅ Simple array lookups: Minimal CPU
✅ No file I/O during match: Safe
✅ Memory usage: Reasonable (a 32x8x30-frame animation ≈ 7.6KB)

MEMORY CALCULATION:
------------------
Memory per animation = width × height × frames × 4 bytes
32×8×30 = 7,680 pixels × 4 bytes = ~31KB

Keep total animations under 500KB to be safe on roboRIO.
*/