/*
 * Copyright (c) 2018 FTC team 4634 FROGbots
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.nio.ByteBuffer;

public class DotstarStrip
{
    private SC18IS602B bridge;
    private DotstarPixel[] pixels;
    private ByteBuffer txBuf;
    private byte[] startFrame = new byte[] {0x00, 0x00, 0x00, 0x00};
    private byte[] endFrame = new byte[] {(byte) 0xFF, (byte)0xFF,(byte) 0xFF, (byte)0xFF};
    private boolean initialized = false;
    private SC18IS602B.I2cMode i2cMode;

    /***
     * Constructs a new DotstarStrip object
     *
     * @param name the name of the I2C to SPI bridge in the hardwareMap
     * @param numPixels the number of pixels on this strip
     * @param hardwareMap the hardwareMap from your OpMode
     * @param i2cMode the I2C mode to use when sending to the bridge
     */
    public DotstarStrip(String name, int numPixels, HardwareMap hardwareMap, SC18IS602B.I2cMode i2cMode)
    {
        this(hardwareMap.get(SC18IS602B.class, name), numPixels, i2cMode);
    }

    public void setI2cMode(SC18IS602B.I2cMode i2cMode)
    {
        bridge.setI2cMode(i2cMode);
    }

    /***
     * Constructs a new DotstarStrip object
     *
     * @param bridge the I2C to SPI bridge the strip is connected to
     * @param numPixels the number of pixels on this strip
     * @param i2cMode the I2C mode to use when sending to the bridge
     */
    public DotstarStrip(SC18IS602B bridge, int numPixels, SC18IS602B.I2cMode i2cMode)
    {
        this.bridge = bridge;
        this.i2cMode = i2cMode;

        pixels = new DotstarPixel[numPixels];

        /*
         * Loop through our pixel array and create a new
         * DotstarPixel object for each index
         */
        for(int i = 0; i < numPixels; i++)
        {
            pixels[i] = new DotstarPixel();
        }

        int payloadLength =
                pixels.length * DotstarPixel.PAYLOAD_SIZE + //Number of bytes needed for all our pixels
                        startFrame.length +                         //Number of bytes needed for the start frame
                        endFrame.length;                            //Number of bytes needed for the end frame

        txBuf = ByteBuffer.allocate(payloadLength);
    }

    /***
     * Initialize the SPI bridge so that apply() can be called
     */
    public void init()
    {
        SC18IS602B.Parameters parameters = new SC18IS602B.Parameters();
        parameters.i2cMode = i2cMode;
        parameters.spiMode = SC18IS602B.SpiMode.SCLK_LOW_WHEN_IDLE_DATA_CLOCKED_ON_RISING_EDGE;
        parameters.byteOrder = SC18IS602B.ByteOrder.MSB_FIRST;
        parameters.spiclkFreq = SC18IS602B.SpiclkFreq.SPICLK_2MHz;

        bridge.initialize(parameters);
        initialized = true;
    }

    /***
     * Query how many pixels this strip is configured for
     *
     * @return the number of pixels this strip is configured for
     */
    public int getNumPixels()
    {
        return pixels.length;
    }

    /***
     * Get the DotstarPixel object at a given position
     * Use this if you want to set a single pixel on the
     * strip only
     *
     * @param pos the position of the pixel on the strip for which to get the DotstarPixel object
     * @return the DotstarPixel object at the position specified by 'pos'
     */
    public DotstarPixel getPixelAt(int pos)
    {
        return pixels[pos];
    }

    /***
     * Get the entire array of all the DotstarPixel objects
     * Use this if you want to set all of the pixels on the
     * strip at the same time
     *
     * @return the entire array of all the DotstarPixel objects
     */
    public DotstarPixel[] getAllPixels()
    {
        return pixels;
    }

    /***
     * Actually send any changes to the pixels out to the strip
     */
    public void apply()
    {
        /*
         * Make sure that we're initialized!
         */
        if(!initialized)
        {
            throw new IllegalStateException("Can't flush to SPI bridge before init()");
        }

        /*
         * Populate our buffer
         */

        int payloadLength =
                pixels.length * DotstarPixel.PAYLOAD_SIZE + //Number of bytes needed for all our pixels
                        startFrame.length +                         //Number of bytes needed for the start frame
                        endFrame.length;                            //Number of bytes needed for the end frame

        ByteBuffer buffer = ByteBuffer.allocate(payloadLength);
        buffer.clear();
        buffer.put(startFrame);
        for(DotstarPixel pixel : pixels)
        {
            buffer.put(pixel.toByteArray());
        }
        buffer.put(endFrame);

        /*
         * Ok, the moment of truth! Write the data
         * out to the SPI bus...
         */
        bridge.writeDataToSpiBus(buffer.array());
    }
}
