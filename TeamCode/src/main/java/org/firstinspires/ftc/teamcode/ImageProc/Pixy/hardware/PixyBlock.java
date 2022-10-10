package org.firstinspires.ftc.teamcode.ImageProc.Pixy.hardware;

import com.qualcomm.robotcore.util.TypeConversion;

/**
 * Block describes the signature, location, and size of a detected block.
 */
public class PixyBlock {
    /**
     * A number from 1 through 7 corresponding to the color trained into the PixyCam,
     * or a sequence of octal digits corresponding to the multiple colors of a color code.
     */
    public final int signature;

    /**
     * The x, y location of the center of the detected block.
     * x is in the range (0, 255)
     * 0 is the left side of the field of view and 255 is the right.
     * y is in the range (0, 199)
     * 0 is the top of the field of view and 199 is the bottom.
     */
    public final int x, y;

    /**
     * The size of the detected block.
     * width or height of zero indicates no block detected.
     * maximum width is 255.
     * maximum height is 199.
     */
    public final int width, height;

    /**
     * The count of blocks detected for the given signature
     * Will be -1 if this is a block from the general query
     */
    public final int blockCount;

    public PixyBlock(int signature, byte blockCount, byte x, byte y, byte width, byte height) {
        this(signature, TypeConversion.unsignedByteToInt(blockCount), x, y, width, height);
    }

    public PixyBlock(int signature, int blockCount, byte x, byte y, byte width, byte height) {
        this.signature = signature;
        this.x = TypeConversion.unsignedByteToInt(x);
        this.y = TypeConversion.unsignedByteToInt(y);
        this.width = TypeConversion.unsignedByteToInt(width);
        this.height = TypeConversion.unsignedByteToInt(height);
        this.blockCount = blockCount;
    }

    public boolean isEmpty()
    {
        return x == 0 && y == 0 && width == 0 && height == 0 && blockCount == -1;
    }

    @Override
    public String toString() {
        return String.format("x: %d, y: %d, w: %d, h: %d cnt: %d", this.x, this.y, this.width, this.height, this.blockCount);
    }
}