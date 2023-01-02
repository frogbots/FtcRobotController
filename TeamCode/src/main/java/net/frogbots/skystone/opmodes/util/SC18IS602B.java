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

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static java.lang.Thread.sleep;

@I2cSensor(name = "Sandbox Electronics SC18IS602B", xmlTag = "SC18IS602B", description = "SC18IS602B I2C<-->SPI bridge from Sandbox Electronics")
public class SC18IS602B extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "SC18IS602B";
    }

    public SC18IS602B(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_I2C_ADDR));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected SC18IS602B(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned)
    {
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    public void setI2cAddress(I2cAddr i2cAddr)
    {
        deviceClient.setI2cAddress(i2cAddr);
    }

    private int MAX_I2C_BYTES_REV = 99;
    private int MAX_I2C_BYTES_MR = 20;
    private byte DEFAULT_I2C_ADDR = 0x28;
    private byte CONFIG_CMD = (byte) 0xF0;
    private Parameters parameters;

    static class Parameters
    {
        public ByteOrder byteOrder;
        public SpiMode spiMode;
        public SpiclkFreq spiclkFreq;
        public I2cMode i2cMode;
    }

    public enum ByteOrder
    {
        MSB_FIRST(0x00),
        LSB_FIRST(0x20);

        ByteOrder(int i)
        {
            this.bVal = (byte) i;
        }

        public final byte bVal;
    }

    public enum SpiMode
    {
        SCLK_LOW_WHEN_IDLE_DATA_CLOCKED_ON_RISING_EDGE(0x00),
        SCLK_LOW_WHEN_IDLE_DATA_CLOCKED_ON_FALLING_EDGE(0x04),
        SCLK_HIGH_WHEN_IDLE_DATA_CLOCKED_ON_RISING_EDGE(0x08),
        SCLK_HIGH_WHEN_IDLE_DATA_CLOCKED_ON_FALLING_EDGE(0x0C);

        SpiMode(int i)
        {
            this.bVal = (byte) i;
        }

        public final byte bVal;
    }

    public enum I2cMode
    {
        MODERN_ROBOTICS,
        REV_STANDARD,
        REV_SPEEDHACK_FOR_400KHz
    }

    public enum SpiclkFreq
    {
        SPICLK_58KHz(0x03),
        SPICLK_115KHz(0x02),
        SPICLK_461KHz(0x01),
        SPICLK_2MHz(0x00);

        SpiclkFreq(int i)
        {
            this.bVal = (byte) i;
        }

        public final byte bVal;
    }

    public void setI2cMode(I2cMode i2cMode)
    {
        parameters.i2cMode = i2cMode;
    }

    public void initialize(Parameters parameters)
    {
        this.parameters = parameters;

        byte config = (byte) (parameters.byteOrder.bVal | parameters.spiMode.bVal | parameters.spiclkFreq.bVal);
        deviceClient.write(CONFIG_CMD, new byte[] {config});
    }

    public void writeDataToSpiBus(byte[] bytes)
    {
        switch (parameters.i2cMode)
        {
            case MODERN_ROBOTICS:
            {
                writeToSpiMr(bytes);
                break;
            }

            case REV_STANDARD:
            {
                writeToSpiRevStd(bytes);
                break;
            }

            case REV_SPEEDHACK_FOR_400KHz:
            {
                writeToSpiRevSpeedHack(bytes);
                break;
            }

            default:
            {
                throw new NullPointerException();
            }
        }
    }

    private void writeToSpiRevStd(byte[] bytes)
    {
        if(bytes.length > MAX_I2C_BYTES_REV)
        {
            for(byte[] chunk : divideArray(bytes, MAX_I2C_BYTES_REV))
            {
                deviceClient.write(0x01, chunk, I2cWaitControl.WRITTEN);
            }
        }
        else
        {
            deviceClient.write(0x01, bytes, I2cWaitControl.WRITTEN);
        }
    }

    private void writeToSpiRevSpeedHack(byte[] bytes)
    {
        if(bytes.length > MAX_I2C_BYTES_REV)
        {
            for(byte[] chunk : divideArray(bytes, MAX_I2C_BYTES_REV))
            {
                deviceClient.write(0x01, chunk, I2cWaitControl.ATOMIC);
                try
                {
                    if(chunk.length > 29) //A magic number! :D
                    {
                        Thread.sleep(5); //Another magic number! :D
                    }
                }
                catch (InterruptedException e)
                {
                    e.printStackTrace();
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
        else
        {
            deviceClient.write(0x01, bytes, I2cWaitControl.ATOMIC);
            try
            {
                if(bytes.length > 29) //A magic number! :D
                {
                    Thread.sleep(5); //Another magic number! :D
                }
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
                Thread.currentThread().interrupt();
            }
        }
    }

    private void writeToSpiMr(byte[] bytes)
    {
        for(byte b : bytes)
        {
            deviceClient.write8(0x01, b, I2cWaitControl.WRITTEN);
        }

        /*if(bytes.length > MAX_I2C_BYTES_MR)
        {
            for(byte[] chunk : divideArray(bytes, MAX_I2C_BYTES_MR))
            {
                deviceClient.write(0x01, chunk, I2cWaitControl.ATOMIC);

                try
                {
                    Thread.sleep(100);
                }
                catch (InterruptedException e)
                {
                    e.printStackTrace();
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
        else
        {
            deviceClient.write(0x01, bytes, I2cWaitControl.WRITTEN);
        }*/
    }

    private static List<byte[]> divideArray(byte[] source, int chunksize)
    {
        List<byte[]> result = new ArrayList<>();
        int start = 0;

        while (start < source.length)
        {
            int end = Math.min(source.length, start + chunksize);
            result.add(Arrays.copyOfRange(source, start, end));
            start += chunksize;
        }

        return result;
    }
}