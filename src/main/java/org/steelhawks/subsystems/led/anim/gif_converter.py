from PIL import Image
import os
import sys

def gif_to_java_frames(gif_path, output_name, target_width=None, target_height=None, frame_delay=2, loop=True):
    """
    Convert GIF to Java frame array code for LED Matrix
    
    Args:
        gif_path: Path to GIF file
        output_name: Name for the Java method
        target_width: Target width (auto-detect if None)
        target_height: Target height (auto-detect if None)
        frame_delay: Delay between frames in 20ms cycles (default 2 = 40ms)
        loop: Whether animation should loop
    
    Returns:
        Java code as string
    """
    if not os.path.exists(gif_path):
        print(f"Error: File not found: {gif_path}")
        return None
    
    img = Image.open(gif_path)
    
    # Auto-detect dimensions from first frame if not specified
    if target_width is None:
        target_width = img.width
    if target_height is None:
        target_height = img.height
    
    print(f"Converting GIF: {gif_path}")
    print(f"Original size: {img.width}x{img.height}")
    print(f"Target size: {target_width}x{target_height}")
    
    frames = []
    frame_delays = []
    
    try:
        frame_num = 0
        while True:
            # Resize to target size
            resized = img.resize((target_width, target_height), Image.NEAREST)
            rgb_img = resized.convert('RGB')
            
            # Extract frame delay from GIF metadata (if available)
            try:
                delay_ms = img.info.get('duration', 40)  # Default 40ms
                delay_cycles = max(1, delay_ms // 20)  # Convert to 20ms cycles
            except:
                delay_cycles = frame_delay
            
            # Extract pixels
            frame_data = []
            for x in range(target_width):
                col = []
                for y in range(target_height):
                    r, g, b = rgb_img.getpixel((x, y))
                    rgb_int = (r << 16) | (g << 8) | b
                    col.append(f"0x{rgb_int:06X}")
                frame_data.append(col)
            
            frames.append(frame_data)
            frame_delays.append(delay_cycles)
            frame_num += 1
            
            img.seek(img.tell() + 1)
    except EOFError:
        pass
    
    print(f"Extracted {len(frames)} frames")
    
    # Generate Java code
    java_code = f"/**\n"
    java_code += f" * Animation: {output_name}\n"
    java_code += f" * Source: {os.path.basename(gif_path)}\n"
    java_code += f" * Size: {target_width}x{target_height}\n"
    java_code += f" * Frames: {len(frames)}\n"
    java_code += f" * Loop: {loop}\n"
    java_code += f" */\n"
    java_code += f"public static FrameAnimation {output_name}() {{\n"
    java_code += f"    int[][][] frames = new int[{len(frames)}][{target_width}][{target_height}];\n"
    
    # Check if all delays are the same
    uniform_delay = all(d == frame_delays[0] for d in frame_delays)
    
    if not uniform_delay:
        java_code += f"    int[] frameDelays = new int[{len(frames)}];\n"
    
    java_code += "\n"
    
    # Generate frame data
    for i, frame in enumerate(frames):
        java_code += f"    // Frame {i}\n"
        for x, col in enumerate(frame):
            java_code += f"    frames[{i}][{x}] = new int[]{{ {', '.join(col)} }};\n"
        if not uniform_delay:
            java_code += f"    frameDelays[{i}] = {frame_delays[i]};\n"
        java_code += "\n"
    
    # Return statement with appropriate constructor
    if uniform_delay:
        java_code += f"    return new FrameAnimation(frames, {frame_delays[0]}, {str(loop).lower()});\n"
    else:
        java_code += f"    return new FrameAnimation(frames, frameDelays, {str(loop).lower()});\n"
    
    java_code += "}\n"
    
    # Calculate memory usage
    memory_bytes = target_width * target_height * len(frames) * 4
    memory_kb = memory_bytes / 1024
    java_code += f"\n// Estimated memory: {memory_kb:.2f} KB\n"
    
    return java_code


def batch_convert(directory, target_width=None, target_height=None, output_file="AnimationLibrary.java"):
    """
    Convert all GIFs in a directory to a single Java file
    
    Args:
        directory: Directory containing GIF files
        target_width: Target width (None = keep original)
        target_height: Target height (None = keep original)
        output_file: Output Java file name
    """
    if not os.path.exists(directory):
        print(f"Error: Directory not found: {directory}")
        return
    
    gif_files = [f for f in os.listdir(directory) if f.lower().endswith('.gif')]
    
    if not gif_files:
        print(f"No GIF files found in {directory}")
        return
    
    print(f"Found {len(gif_files)} GIF files")
    print(f"Generating {output_file}...\n")
    
    # Generate class header
    output = "package org.steelhawks.subsystems.led.anim;\n\n"
    output += "/**\n"
    output += " * Pre-generated animation library\n"
    output += f" * Auto-generated from GIFs in {directory}\n"
    output += " */\n"
    output += "public class AnimationLibrary {\n\n"
    
    # Convert each GIF
    for gif_file in sorted(gif_files):
        gif_path = os.path.join(directory, gif_file)
        method_name = os.path.splitext(gif_file)[0].replace('-', '_').replace(' ', '_')
        
        print(f"Converting {gif_file}...")
        java_method = gif_to_java_frames(
            gif_path, 
            method_name,
            target_width=target_width,
            target_height=target_height
        )
        
        if java_method:
            output += "    " + java_method.replace("\n", "\n    ") + "\n"
    
    output += "}\n"
    
    # Write to file
    with open(output_file, 'w') as f:
        f.write(output)
    
    print(f"\n✓ Generated {output_file}")
    print(f"✓ Contains {len(gif_files)} animations")


def main():
    """
    Command-line interface
    
    Usage:
        # Single file with auto-detect size
        python gif_converter.py my_animation.gif outputName
        
        # Single file with specific size
        python gif_converter.py my_animation.gif outputName 32 8
        
        # Batch convert directory
        python gif_converter.py --batch animations/ 32 8
    """
    if len(sys.argv) < 2:
        print("GIF to LED Matrix Frame Array Converter")
        print("\nUsage:")
        print("  Single file (auto size):  python gif_converter.py input.gif outputName")
        print("  Single file (set size):   python gif_converter.py input.gif outputName 32 8")
        print("  Batch convert:            python gif_converter.py --batch directory/ 32 8")
        print("\nExamples:")
        print("  python gif_converter.py fire.gif fireAnimation")
        print("  python gif_converter.py fire.gif fireAnimation 16 16")
        print("  python gif_converter.py --batch src/main/deploy/anim/ 32 8")
        return
    
    if sys.argv[1] == "--batch":
        # Batch mode
        directory = sys.argv[2] if len(sys.argv) > 2 else "."
        width = int(sys.argv[3]) if len(sys.argv) > 3 else None
        height = int(sys.argv[4]) if len(sys.argv) > 4 else None
        output_file = sys.argv[5] if len(sys.argv) > 5 else "AnimationLibrary.java"
        
        batch_convert(directory, width, height, output_file)
    else:
        # Single file mode
        gif_path = sys.argv[1]
        output_name = sys.argv[2] if len(sys.argv) > 2 else "animation"
        width = int(sys.argv[3]) if len(sys.argv) > 3 else None
        height = int(sys.argv[4]) if len(sys.argv) > 4 else None
        
        java_code = gif_to_java_frames(gif_path, output_name, width, height)
        
        if java_code:
            # Print to console
            print("\n" + "="*80)
            print(java_code)
            print("="*80)
            
            # Also save to file
            output_file = f"{output_name}.java"
            with open(output_file, 'w') as f:
                f.write(java_code)
            print(f"\n✓ Saved to {output_file}")


if __name__ == "__main__":
    main()


# ==================== QUICK USAGE EXAMPLES ====================

# Example 1: Single GIF, auto-detect size
# python gif_converter.py fire.gif fireAnimation

# Example 2: Single GIF, resize to 32x8
# python gif_converter.py explosion.gif explosionAnim 32 8

# Example 3: Convert all GIFs in a folder to 16x16
# python gif_converter.py --batch ../../../../../../deploy/anim 8 32

# Example 4: Use in another Python script
"""
from gif_converter import gif_to_java_frames

code = gif_to_java_frames(
    "my_animation.gif",
    "myAnimation",
    target_width=32,
    target_height=8,
    frame_delay=3,
    loop=True
)

print(code)
"""