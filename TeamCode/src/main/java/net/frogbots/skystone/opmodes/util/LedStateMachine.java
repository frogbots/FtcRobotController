package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.util.Range;


public class LedStateMachine
{
    private long startT;
    private DotstarStrip dotstarStrip;
    private boolean flashState = true;
    private long lastFlashT = 0;
    private int lastLeds = -1;

    public LedStateMachine(DotstarStrip dotstarStrip)
    {
        this.dotstarStrip = dotstarStrip;
    }

    enum State
    {
        START,
        BLUE_REPLACE_GREEN,
        YELLOW_REPLACE_BLUE,
        RED_REPLACE_YELLOW,
        SOLID_RED,
        FLASHING_RED,
        PURPLE
    }

    private State state = State.START;

    public void fire()
    {
        switch (state)
        {
            case START:
            {

                startT = System.currentTimeMillis();
                state = State.BLUE_REPLACE_GREEN;
            }

            case BLUE_REPLACE_GREEN:
            {
                long elapsed = System.currentTimeMillis() - startT;
                long remainingT = 90000 - elapsed;

                if(remainingT < 0)
                {
                    lastLeds = -1;
                    state = State.YELLOW_REPLACE_BLUE;
                }

                remainingT = (long) Range.clip(remainingT, 0, 90000);
                int leds = (int) Math.round(Range.scale(remainingT, 0, 90000, 0, 30));

                for(DotstarPixel pixel : dotstarStrip.getAllPixels())
                {
                    pixel.queueRgb(0,0,255);
                }

                for(int i = 0; (i < leds) && (i < dotstarStrip.getNumPixels()); i++)
                {
                    dotstarStrip.getPixelAt(i).queueRgb(0, 255, 0);
                }

                if(lastLeds != leds)
                {
                    lastLeds = leds;
                    dotstarStrip.apply();
                }

                break;
            }

            case YELLOW_REPLACE_BLUE:
            {
                long elapsed = System.currentTimeMillis() - startT;
                long remainingT = 100000 - elapsed;

                if(remainingT < 0)
                {
                    lastLeds = -1;
                    state = State.RED_REPLACE_YELLOW;
                }

                remainingT = (long) Range.clip(remainingT, 0, 10000);
                int leds = (int) Math.round(Range.scale(remainingT, 0, 10000, 0, 30));

                for(DotstarPixel pixel : dotstarStrip.getAllPixels())
                {
                    pixel.queueRgb(255, 140, 0);
                }

                for(int i = 0; (i < leds) && (i < dotstarStrip.getNumPixels()); i++)
                {
                    dotstarStrip.getPixelAt(i).queueRgb(0, 0, 255);
                }

                if(lastLeds != leds)
                {
                    lastLeds = leds;
                    dotstarStrip.apply();
                }

                break;
            }

            case RED_REPLACE_YELLOW:
            {
                long elapsed = System.currentTimeMillis() - startT;
                long remainingT = 110000 - elapsed;

                if(remainingT < 0)
                {
                    lastLeds = -1;
                    state = State.SOLID_RED;
                }

                remainingT = (long) Range.clip(remainingT, 0, 10000);
                int leds = (int) Math.round(Range.scale(remainingT, 0, 10000, 0, 30));

                for(DotstarPixel pixel : dotstarStrip.getAllPixels())
                {
                    pixel.queueRgb(255,0,0);
                }

                for(int i = 0; (i < leds) && (i < dotstarStrip.getNumPixels()); i++)
                {
                    dotstarStrip.getPixelAt(i).queueRgb(255, 140, 0);
                }

                if(lastLeds != leds)
                {
                    lastLeds = leds;
                    dotstarStrip.apply();
                }

                break;
            }

            case SOLID_RED:
            {
                long elapsed = System.currentTimeMillis() - startT;
                long remainingT = 115000 - elapsed;

                if(remainingT < 0)
                {
                    lastLeds = -1;
                    state = State.FLASHING_RED;
                }

                for(DotstarPixel pixel : dotstarStrip.getAllPixels())
                {
                    pixel.queueRgb(255, 0, 0);
                }

                if(lastLeds != 30)
                {
                    lastLeds = 30;
                    dotstarStrip.apply();
                }
                break;
            }

            case FLASHING_RED:
            {
                long elapsed = System.currentTimeMillis() - startT;
                long remainingT = 120000 - elapsed;

                if(remainingT < 0)
                {
                    lastLeds = -1;
                    state = State.PURPLE;
                }

                if(System.currentTimeMillis() - lastFlashT > 250)
                {
                    if(!flashState)
                    {
                        for(DotstarPixel pixel : dotstarStrip.getAllPixels())
                        {
                            pixel.queueRgb(255,0,0);
                        }
                    }
                    else
                    {
                        for(DotstarPixel pixel : dotstarStrip.getAllPixels())
                        {
                            pixel.queueRgb(0,0,0);
                        }
                    }

                    flashState = !flashState;
                    lastFlashT = System.currentTimeMillis();
                    dotstarStrip.apply();
                }

                break;
            }

            case PURPLE:
            {
                for(DotstarPixel pixel : dotstarStrip.getAllPixels())
                {
                    pixel.queueRgb(92, 50, 168);
                }

                if(lastLeds != 30)
                {
                    lastLeds = 30;
                    dotstarStrip.apply();
                }
                break;
            }
        }
    }
}
