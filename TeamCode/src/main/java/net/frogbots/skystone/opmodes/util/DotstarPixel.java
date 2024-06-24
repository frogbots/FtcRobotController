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

import android.graphics.Color;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.nio.ByteBuffer;

public class DotstarPixel
{
    static final int PAYLOAD_SIZE = 4;
    private static final int MAX_HUE_VALUE = 360;
    private static final int MIN_HUE_VALUE = 0;
    private static final int MIN_RGB_COMPONENT_VALUE = 0;
    private static final int MAX_RGB_COMPONENT_VALUE = 255;
    private static final float MAX_SATURATION_VALUE = 1;
    private static final float MIN_SATURATION_VALUE = 0;
    private static final float MAX_VALUE_VALUE = 1;
    private static final float MIN_VALUE_VALUE = 0;
    private static final int HUE_INDEX_IN_HSV_FLOAT_ARRAY = 0;
    private static final int SATURATION_INDEX_IN_HSV_FLOAT_ARRAY = 1;
    private static final int VALUE_INDEX_IN_HSV_FLOAT_ARRAY = 2;
    private byte brightness = (byte) 0xFF;
    private byte r, g, b;

    /*
     * Package-private constructor
     */
    DotstarPixel(){}

    /***
     * Sets the global brightness value for this pixel.
     * Think of this like the brightness control on your
     * monitor, except that it applies only to a single
     * pixel rather than the entire strip.
     *
     * Please note that you will not observe any change on
     * the actual LED strip until you call strip.apply()
     *
     * @param brightness brightness value (0...31)
     */
    public void queueGlobalBrightness(int brightness)
    {
        Range.throwIfRangeIsInvalid(brightness, 0, 31);

        //First 3 bits need to be 1s because... reasons
        this.brightness = (byte) (brightness | 0b11100000);
    }

    /***
     * Sets the color value of this pixel using a 24-bit int
     *
     * Please note that you will not observe any change on
     * the actual LED strip until you call strip.apply()
     *
     * @param color a 24-bit color int
     */
    public void queueColor(int color)
    {
        this.r = (byte) Color.red(color);
        this.g = (byte) Color.green(color);
        this.b = (byte) Color.blue(color);
    }

    /***
     * Sets the color value of this pixel using a color
     * declared in a resource XML file
     *
     * Please note that you will not observe any change on
     * the actual LED strip until you call strip.apply()
     *
     * @param resId the id of the color in the XML file
     */
    public void queueColorFromRes(int resId)
    {
        int color = AppUtil.getInstance().getRootActivity().getResources().getColor(resId);

        this.r = (byte) Color.red(color);
        this.g = (byte) Color.green(color);
        this.b = (byte) Color.blue(color);
    }

    /***
     * Set the color value of the pixel using the standard
     * RGB color space
     *
     * Please note that you will not observe any change on
     * the actual LED strip until you call strip.apply()
     *
     * @param r red value
     * @param g green value
     * @param b blue value
     */
    public void queueRgb(byte r, byte g, byte b)
    {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    /***
     * Set the color value of the pixel using the standard
     * RGB color space
     *
     * Please note that you will not observe any change on
     * the actual LED strip until you call strip.apply()
     *
     * @param r red value (0...255)
     * @param g green value(0...255)
     * @param b blue value (0... 255)
     */
    public void queueRgb(int r, int g, int b)
    {
        Range.throwIfRangeIsInvalid(r, MIN_RGB_COMPONENT_VALUE, MAX_RGB_COMPONENT_VALUE);
        Range.throwIfRangeIsInvalid(g, MIN_RGB_COMPONENT_VALUE, MAX_RGB_COMPONENT_VALUE);
        Range.throwIfRangeIsInvalid(b, MIN_RGB_COMPONENT_VALUE, MAX_RGB_COMPONENT_VALUE);

        this.r = (byte) r;
        this.g = (byte) g;
        this.b = (byte) b;
    }

    /***
     * Set the color value of this pixel using the HSV
     * color space, but uses max saturation and brightness
     *
     * Please note that you will not observe any change on
     * the actual LED strip until you call strip.apply()
     *
     * @param h hue value (0...360)
     */
    public void queueHue(float h)
    {
        queueHsv(h, MAX_SATURATION_VALUE, MAX_VALUE_VALUE);
    }

    /***
     * Set the color value of this pixel using the HSV
     * color space, but uses max saturation
     *
     * Please note that you will not observe any change on
     * the actual LED strip until you call strip.apply()
     *
     * @param h hue value (0...360)
     * @param v brightness value (0...1)
     */
    public void queueHue(float h, float v)
    {
        queueHsv(h, MAX_SATURATION_VALUE, v);
    }

    /***
     * Sets the color value of this pixel using the HSV
     * color space
     *
     * Please note that you will not observe any change on
     * the actual LED strip until you call strip.apply()
     *
     * @param h hue value (0...360)
     * @param s saturation value (0...1)
     * @param v brightness value (0...1)
     */
    public void queueHsv(float h, float s, float v)
    {
        queueHsv(new float[]{h, s, v});
    }

    /***
     * [Internal use only] Sets the color value of this pixel
     * using the HSV color space
     *
     * @param hsvVals [0] = hue, [1] = sat, [2] = val
     */
    private void queueHsv(float[] hsvVals)
    {
        Range.throwIfRangeIsInvalid(hsvVals[HUE_INDEX_IN_HSV_FLOAT_ARRAY       ], MIN_HUE_VALUE,        MAX_HUE_VALUE);
        Range.throwIfRangeIsInvalid(hsvVals[SATURATION_INDEX_IN_HSV_FLOAT_ARRAY], MIN_SATURATION_VALUE, MAX_SATURATION_VALUE);
        Range.throwIfRangeIsInvalid(hsvVals[VALUE_INDEX_IN_HSV_FLOAT_ARRAY     ], MIN_VALUE_VALUE,      MAX_VALUE_VALUE);

        int argb = Color.HSVToColor(hsvVals);

        r = (byte) ((argb>>16) & 0xFF);
        g = (byte) ((argb>>8)  & 0xFF);
        b = (byte) ((argb>>0)  & 0xFF);
    }

    /***
     * Convert this pixel to its byte array representation
     * so it can be sent to the SPI bridge
     *
     * @return the byte array representation of this pixel
     */
    byte[] toByteArray()
    {
        ByteBuffer byteBuffer = ByteBuffer.allocate(PAYLOAD_SIZE);
        byteBuffer.put(brightness);
        byteBuffer.put(b);
        byteBuffer.put(g);
        byteBuffer.put(r);

        return byteBuffer.array();
    }
}
