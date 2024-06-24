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

import com.qualcomm.robotcore.util.Range;

public class ColorWheel
{
    private float startAt;
    private float index;
    private static final float NUM_HUE_DEGREES = 360;

    /***
     * Increment where the next() method will begin looping
     * around the hue wheel, and reset the index counter so
     * that a call to next() will start returning values
     * from the newly incremented start position
     *
     * @param steps the number of steps to increment
     */
    public void incrementStartPositionAndReset(float steps)
    {
        startAt += steps;

        if(startAt >= NUM_HUE_DEGREES)
        {
            startAt = 0;
        }

        index = startAt;
    }

    /***
     * Set the hue value at which the next() method will begin looping
     * around the hue wheel, and reset the index counter so that a call
     * to next() will start returning values from the newly set start position
     *
     * @param startAt start at this hue
     */
    public void setStartPositionAndReset(float startAt)
    {
        Range.throwIfRangeIsInvalid(startAt,0, NUM_HUE_DEGREES);

        this.startAt = startAt;
        index = startAt;
    }

    /***
     * Get the next value from the color wheel
     * This works just like a real color wheel, so as soon as the
     * final value (360) is reached, it loops back around
     *
     * @param numStepsToJump get the next value by going forward this many steps on the color wheel
     * @return the next value from the color wheel, according to 'numStepsToJump'
     */
    public float next(int numStepsToJump)
    {
        if(numStepsToJump < 0)
        {
            throw new IllegalArgumentException();
        }

        index += numStepsToJump;

        if(index >= NUM_HUE_DEGREES)
        {
            index -= NUM_HUE_DEGREES;
        }

        return index;
    }
}
