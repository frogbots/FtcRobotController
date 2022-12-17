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

package net.frogbots.skystone.drivers;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;

@I2cSensor(name = "Adafruit VCNL4010OPENFTC", description = "Light/IR Proximity sensor from Adafruit", xmlTag = "VCNL4010OPENFTC")
public class AdafruitVCNL4010 extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    /*
     * The VCNL4010 has a fixed address. This means that if you want to use for than one:
     *
     *      For MR:
     *              Each sensor would need to be on a different CDIM
     *
     *      For REV:
     *              Each sensor would need to be in a different I2C port
     */
    private static final byte DEVICE_I2C_ADDRESS = (byte) 0x13;

    //----------------------------------------------------------------------------------------------------------------------------------------------
    // Registers / CMD data
    //----------------------------------------------------------------------------------------------------------------------------------------------

    /*
     * The command register controls the overall operation of the sensor
     *
     * Bit 7: [config_lock]
     *              Read only bit. Value = 1
     *
     * Bit 6: [als_data_rdy]
     *              Read only bit. Value = 1 when ambient light measurement data is available in the result registers.
     *              This bit will be reset when one of the corresponding result registers (0x85, 0x86) is read
     *
     * Bit 5: [prox_data_rdy]
     *              Read only bit. Value = 1 when proximity measurement data is available in the result registers.
     *              This bit will be reset when one of the corresponding result registers (0x87, 0x88) is read
     *
     * Bit 4: [als_od]
     *              R/W bit. Starts a single on-demand measurement for ambient light. If averaging is enabled, starts
     *              a sequence of readings and stores the averaged result. Result is available at the end of conversion
     *              for reading in the registers 0x85 (high byte) and 0x86 (low byte)
     *
     * Bit 3: [prox_od]
     *              R/W bit. Starts a single on-demand measurement for proximity. Result is available at the end of
     *              conversion for reading in the registers 0x87 (high byte) and 0x88 (low byte).
     *
     * Bit 2: [als_en]
     *              R/W bit. Enables periodic ambient light measurement, according to the speed set in TODO reg
     *
     * Bit 1: [prox_en]
     *              R/W bit. Enables periodic proximity measurement, according to the speed set in TODO reg
     *
     * Bit 0: selftimed_en]
     *              R/W bit. Enables state machine and LP oscillator for self timed measurements; no measurement is
     *              performed until the corresponding bit is set
     */
    private static final byte REG_COMMAND = (byte) 0x80;

    private enum Cmd
    {
        PERFORM_AMBIENT_LIGHT_MEASUREMENT_NOW           (0x10),
        PERFORM_PROXIMITY_MEASUREMENT_NOW               (0x08),
        ENABLE_AMBIENT_LIGHT_INTERNAL_TIMED_MEASUREMENT (0x04),
        ENABLE_PROXIMITY_INTERNAL_TIMED_MEASUREMENT     (0x02),
        ENABLE_MEASUREMENT_TIMER                        (0x01);

        int bVal;

        Cmd(int i)
        {
            this.bVal = i;
        }

        public int getValue()
        {
            return bVal;
        }
    }

    /*
     * The product ID register contains information about product ID and product revision
     *
     * Bits 7-4:
     *              Product ID
     *
     * Bits 3-0:
     *              Revision ID
     */
    private static final byte REG_PRODUCT_AND_REV_ID = (byte) 0x81;

    /*
     * This register controls how often the sensor internally performs a range reading
     *
     * Bits 7-3:
     *              N/A
     *
     * Bits 2-0:
     *              Rate of Proximity Measurement
     */
    private static final byte REG_PROX_INTERNAL_READ_RATE = (byte) 0x82;

    /*
     * There are 8 possible values for the internal range reading frequency:
     *
     *              0x00 = 1.95 measurements/s (DEFAULT)
     *              0x01 = 3.90625 measurements/s
     *              0x02 = 7.8125 measurements/s
     *              0x03 = 16.625 measurements/s
     *              0x04 = 31.25 measurements/s
     *              0x05 = 62.5 measurements/s
     *              0x06 = 125 measurements/s
     *              0x07 = 250 measurements/s
     */
    enum ProximityInternalSelftimedMeasurementRate
    {
        RATE_2Hz   (0x00),
        RATE_4Hz   (0x01),
        RATE_8Hz   (0x02),
        RATE_17Hz  (0x03),
        RATE_31Hz  (0x04),
        RATE_63Hz  (0x05),
        RATE_125Hz (0x06),
        RATE_250Hz (0x07);

        int bVal;

        ProximityInternalSelftimedMeasurementRate(int i)
        {
            this.bVal = i;
        }

        public int getValue()
        {
            return bVal;
        }
    }

    /*
     * The IR LED current register controls how much current the IR LED is allowed
     * to use when performing a proximity measurement. It is adjustable in steps
     * of 10ma from 0ma to 200ma.
     *
     * CARE SHOULD BE TAKEN WHEN USING THIS SENSOR ON A CDIM AS IT CAN ONLY SUPPLY
     * A MAXIMUM OF 250ma ACROSS THE ENTIRE DEVICE!
     */
    private static final byte REG_IR_LED_CURRENT = (byte) 0x83;

    /*
     * There are 20 possible values for the IR current register.
     * IR LED current = Value (dec.) x 10 mA.
     * Valid Range = 0 to 20d. e.g. 0 = 0 mA, 1 = 10 mA, ...., (2 = 20 mA = DEFAULT)
     *
     * The LED Current is limited to 200 mA for values higher than 20d
     */
    public enum IrLedCurrent
    {
        CURRENT_0mA   (0x00),
        CURRENT_10mA  (0x01),
        CURRENT_20mA  (0x02),
        CURRENT_30mA  (0x03),
        CURRENT_40mA  (0x04),
        CURRENT_50mA  (0x05),
        CURRENT_60mA  (0x06),
        CURRENT_70mA  (0x07),
        CURRENT_80mA  (0x08),
        CURRENT_90mA  (0x09),
        CURRENT_100mA (0x0A),
        CURRENT_110mA (0x0B),
        CURRENT_120mA (0x0C),
        CURRENT_130mA (0x0D),
        CURRENT_140mA (0x0E),
        CURRENT_150mA (0x0F),
        CURRENT_160mA (0x10),
        CURRENT_170mA (0x11),
        CURRENT_180mA (0x12),
        CURRENT_190mA (0x13),
        CURRENT_200mA (0x14);

        int bVal;

        IrLedCurrent(int i)
        {
            this.bVal = i;
        }

        public int getValue()
        {
            return bVal;
        }
    }

    /*
     * The ambient light parameter register controls.. well.. parameters for
     * the light sensor (duh!) :P
     *
     * Bit 7: [Continuous conversion mode]
     *              R/W bit. Continuous conversion mode.
     *              Enable = 1; Disable = 0 (DEFAULT)
     *              This function can be used for performing faster ambient light measurements. Please refer
     *              to the application information chapter 3.3 of the datasheet for details about this function
     *
     * Bits 6-4: [als_rate]
     *              R/W bits. Controls Ambient light measurement rate
     *
     * Bit 3: [Auto Offset Compensation]
     *              R/W bit. Automatic offset compensation.
     *              Enabled = 1 (DEFAULT); Disabled = 0
     *              In order to compensate for a package or temperature related drift of the ambient light values,
     *              there is a built in automatic offset compensation function. With active auto offset compensation,
     *              the offset value is measured before each ambient light measurement and then subtracted automatically
     *              from the actual reading.
     *
     * Bits 2-0: [Averaging function]
     *              R/W bits. Averaging function.
     *              Bit values sets the number of single conversions done during one measurement cycle. Result is the
     *              average value of all conversions. Number of conversions = 2 x decimal value
     *              e.g. 0 = 1 conv., 1 = 2 conv, 2 = 4 conv., ....7 = 128 conv. DEFAULT = 32 conv.
     */
    private static final byte REG_ALS_PARAM = (byte) 0x84;

    enum AlsParameters
    {
        ENABLE_AUTO_OFFSET_COMPENSATION (0x08);

        int bVal;

        AlsParameters(int i)
        {
            this.bVal = i;
        }

        public int getValue()
        {
            return bVal;
        }
    }

    enum AmbientLightInternalSelftimedMeasurementRate
    {
        RATE_1Hz  (0x00),
        RATE_2Hz  (0x01),
        RATE_3Hz  (0x02),
        RATE_4Hz  (0x03),
        RATE_5Hz  (0x04),
        RATE_6Hz  (0x05),
        RATE_8Hz  (0x06),
        RATE_10Hz (0x07);

        int bVal;

        AmbientLightInternalSelftimedMeasurementRate(int i)
        {
            this.bVal = i;
        }

        public int getValue()
        {
            return bVal;
        }
    }

    /*
     * Ambient light result registers
     *
     * Register address = 85h and 86h. These registers are the result registers for ambient light measurement readings.
     * The result is a 16 bit value. The high byte is stored in register 0x85 and the low byte in register 0x86.
     */
    private static final byte REG_ALS_RESULT = (byte) 0x85; //High byte. Low byte = 0x86
    private static final byte NUM_ALS_RESULT_REGISTERS = 2;

    /*
     * Proximity result registers
     *
     * Register address = 87h and 88h. These registers are the result registers for proximity measurement readings.
     * he result is a 16 bit value. The high byte is stored in register 0x87 and the low byte in register 0x88.
     */
    private static final byte REG_PROX_RESULT = (byte) 0x87; //High byte. Low byte = 0x88
    private static final byte NUM_PROX_RESULT_REGISTERS = 2;

    /*
     * The proximity measurement uses a square wave IR signal as the measurement signal.
     * Four different values are possible:
     *
     *              00 = 390.625 kHz (DEFAULT)
     *              01 = 781.25 kHz
     *              10 = 1.5625 MHz
     *              11 = 3.125 MHz
     */
    enum ProxSignalFreq
    {
        FREQ_391KHz(0x00),
        FREQ_781KHz(0x01),
        FREQ_2MHz  (0x02),
        FREQ_3MHz  (0x03);

        int bVal;

        ProxSignalFreq(int i)
        {
            this.bVal = i;
        }

        public int getValue()
        {
            return bVal;
        }
    }

    private boolean proximityInternalTimedMeasurementEnabled = false;
    private boolean ambientLightInternalTimedReadEnabled = false;
    private byte cmdRegisterData = 0x00;
    private byte alsParamRegisterData = 0x00;

    //----------------------------------------------------------------------------------------------------------------------------------------------
    // Constructors
    //----------------------------------------------------------------------------------------------------------------------------------------------

    public AdafruitVCNL4010 (I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEVICE_I2C_ADDRESS));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected AdafruitVCNL4010(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned)
    {
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    //----------------------------------------------------------------------------------------------------------------------------------------------
    // I2CDeviceSync methods
    //----------------------------------------------------------------------------------------------------------------------------------------------

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName()
    {
        return "Adafruit VCNL4010";
    }

    @Override
    protected boolean doInitialize()
    {
        return false;
    }

    //----------------------------------------------------------------------------------------------------------------------------------------------
    // General device info
    //----------------------------------------------------------------------------------------------------------------------------------------------

    /***
     * Get the internal product ID
     *
     * @return the internal product ID
     */
    public synchronized int getProductId()
    {
        byte bVal = deviceClient.read8(REG_PRODUCT_AND_REV_ID); //Read the combined product and rev ID byte

        return TypeConversion.unsignedByteToInt(extractHighNibble(bVal)); //Java bytes are signed so we can't just return it directly
    }

    /***
     * Get the internal revision ID
     *
     * @return the internal revision ID
     */
    public synchronized int getRevision()
    {
        byte bVal = deviceClient.read8(REG_PRODUCT_AND_REV_ID); //Read the combined product and rev ID byte

        return TypeConversion.unsignedByteToInt(extractLowNibble(bVal)); //Java bytes are signed so we can't just return it directly
    }

    //----------------------------------------------------------------------------------------------------------------------------------------------
    // Proximity
    //----------------------------------------------------------------------------------------------------------------------------------------------

    /***
     * Tells the sensor to perform a proximity
     * measurement right now
     */
    public synchronized void performProximityMeasurementNow()
    {
        if(proximityInternalTimedMeasurementEnabled)
        {
            throw new IllegalStateException("Cannot perform an on-demand measurement while automatic internal measurements are enabled!");
        }

        cmdRegisterData |= Cmd.PERFORM_PROXIMITY_MEASUREMENT_NOW.bVal; //Temporarily add the flag to the cmd reg data
        flushCmdReg();
        cmdRegisterData ^= Cmd.PERFORM_PROXIMITY_MEASUREMENT_NOW.bVal; //Ok, now remove it
    }

    /***
     * Toggles the sensor's ability to internally perform
     * timed proximity measurements, so that the user doesn't
     * need to continuously send ranging commands before
     * reading the result registers.
     *
     * Note that when this is enabled, the sensor will
     * ignore any on-demand ranging commands...
     */
    public synchronized void setProximityInternalTimedMeasurementEnabled(boolean en)
    {
        proximityInternalTimedMeasurementEnabled = en;

        if(en)
        {
            cmdRegisterData |= Cmd.ENABLE_PROXIMITY_INTERNAL_TIMED_MEASUREMENT.bVal;
            cmdRegisterData |= Cmd.ENABLE_MEASUREMENT_TIMER.bVal;
        }
        else //disable it
        {
            cmdRegisterData ^= Cmd.ENABLE_PROXIMITY_INTERNAL_TIMED_MEASUREMENT.bVal;

            /*
             * If the ambient light timed measurement is disabled,
             * then there's no need to keep the oscillator running,
             * so go ahead and disable that, too
             */
            if(!ambientLightInternalTimedReadEnabled)
            {
                cmdRegisterData ^= Cmd.ENABLE_MEASUREMENT_TIMER.bVal;
            }
        }

        flushCmdReg();
    }

    /***
     * Returns whether or not the sensor's ability to internally
     * perform timed proximity measurements is enabled
     *
     * @return whether or not the sensor's ability to internally
     *         perform timed proximity measurements is enabled
     */
    public synchronized boolean isProximityInternalTimedMeasurementEnabled()
    {
        return proximityInternalTimedMeasurementEnabled;
    }

    /***
     * Sets the speed at which the sensor internally performs
     * proximity readings. Note that the sensor will ignore
     * this command while internal timed measurement is enabled
     *
     * @param readRate the speed at which the sensor should
     *                 internally perform proximity readings
     */
    public synchronized void setProximityInternalSelftimedReadRate(ProximityInternalSelftimedMeasurementRate readRate)
    {
        if(proximityInternalTimedMeasurementEnabled)
        {
            throw new IllegalStateException("Cannot set measurement rate while timed reading is enabled!");
        }

        deviceClient.write8(REG_PROX_INTERNAL_READ_RATE, readRate.bVal);
    }

    /***
     * Sets the amount of current the IR LED uses
     * when sending out pulses
     *
     * @param current the amount of current the IR LED
     *                should use when sending out a pulse
     */
    public synchronized void setIRLedCurrent(IrLedCurrent current)
    {
        deviceClient.write8(REG_IR_LED_CURRENT, current.bVal);
    }

    /***
     * Queries the sensor's proximity result registers
     * to obtain the latest proximity measurement
     *
     * @return the latest proximity measurement
     */
    public synchronized int getLastProximity()
    {
        byte[] bytes = deviceClient.read(REG_PROX_RESULT, NUM_PROX_RESULT_REGISTERS);

        return TypeConversion.byteArrayToShort(bytes, ByteOrder.BIG_ENDIAN);
    }

    /***
     * Tells the sensor to perform a proximity measurement
     * now, and then queries the proximity result registers
     * to obtain the result of the measurement
     *
     * @return the result of the proximity measurement just commanded
     */
    public synchronized int measureAndGetProximity()
    {
        performProximityMeasurementNow();
        return getLastProximity();
    }

    //----------------------------------------------------------------------------------------------------------------------------------------------
    // Ambient light
    //----------------------------------------------------------------------------------------------------------------------------------------------

    /***
     * Tells the sensor to perform an ambient
     * light measurement right now
     */
    public synchronized void performAmbientLightMeasurementNow()
    {
        if(ambientLightInternalTimedReadEnabled)
        {
            throw new IllegalStateException("Cannot perform an on-demand ambient light measurement while automatic internal measurements for ambient light are enabled!");
        }

        cmdRegisterData |= Cmd.PERFORM_AMBIENT_LIGHT_MEASUREMENT_NOW.bVal; //Temporarily add the flag to the cmd reg data
        flushCmdReg();
        cmdRegisterData ^= Cmd.PERFORM_AMBIENT_LIGHT_MEASUREMENT_NOW.bVal; //Ok, now remove it
    }

    /***
     * Toggles the sensor's ability to internally preform
     * timed ambient light readings, so that the user doesn't
     * need to continuously send ranging commands before
     * reading the result registers.
     *
     * Note that when this is enabled, the sensor will
     * ignore any on-demand measurement commands...
     */
    public synchronized void setAmbientLightInternalTimedMeasurementEnabled(boolean en)
    {
        ambientLightInternalTimedReadEnabled = en;

        if(en)
        {
            cmdRegisterData |= Cmd.ENABLE_AMBIENT_LIGHT_INTERNAL_TIMED_MEASUREMENT.bVal;
            cmdRegisterData |= Cmd.ENABLE_MEASUREMENT_TIMER.bVal;
        }
        else //disable it
        {
            cmdRegisterData ^= Cmd.ENABLE_AMBIENT_LIGHT_INTERNAL_TIMED_MEASUREMENT.bVal;

            /*
             * If the proximity timed measurement is disabled,
             * then there's no need to keep the oscillator running,
             * so go ahead and disable that, too
             */
            if(!proximityInternalTimedMeasurementEnabled)
            {
                cmdRegisterData ^= Cmd.ENABLE_MEASUREMENT_TIMER.bVal;
            }
        }

        flushCmdReg();
    }

    /***
     * Returns whether or not the sensor's ability to internally
     * perform timed ambient light measurements is enabled
     *
     * @return whether or not the sensor's ability to internally
     *         perform timed ambient light measurements is enabled
     */
    public synchronized boolean isAmbientLightInternalTimedMeasurementEnabled()
    {
        return ambientLightInternalTimedReadEnabled;
    }

    /***
     * Sets the speed at which the sensor internally performs
     * ambient light readings. Note that the sensor will ignore
     * this command while internal measurement is enabled
     *
     * @param readRate the speed at which the sensor should
     *                 internally perform ambient light readings
     */
    public synchronized void setAmbientLightInternalSelftimedMeasurementRate(AmbientLightInternalSelftimedMeasurementRate readRate)
    {
        if(ambientLightInternalTimedReadEnabled)
        {
            throw new IllegalStateException("Cannot set ambient light measurement rate while automatic internal measurements for ambient light are enabled!");
        }

        byte clearMask = 0b01110000;
        byte shiftedRate = (byte) (readRate.bVal << 4);

        alsParamRegisterData ^= clearMask; //First we have to clear the 3 bytes
        alsParamRegisterData |= shiftedRate; //Then move in the the value we actually want

        flushAlsParamReg();
    }

    /***
     * Queries the sensor's ambient light result
     * registers to obtain the latest ambient light
     * measurement
     *
     * @return the latest ambient light measurement
     */
    public synchronized int getLastAmbientLight()
    {
        byte[] bytes = deviceClient.read(REG_ALS_RESULT, NUM_ALS_RESULT_REGISTERS);

        return TypeConversion.byteArrayToShort(bytes, ByteOrder.BIG_ENDIAN);
    }

    /***
     * Tells the sensor to perform an ambient light measurement
     * now, and then queries the ambient light result registers
     * to obtain the result of the measurement
     *
     * @return the result of the ambient light measurement just commanded
     */
    public synchronized int measureAndGetAmbientLight()
    {
        performAmbientLightMeasurementNow();
        return getLastAmbientLight();
    }

    /***
     * Enables or disables the AL sensor's auto offset compensation
     * feature. With auto offset compensation, the offset value is
     * measured before each ambient light measurement and subtracted
     * automatically from the actual reading.
     *
     * @param en whether or not to enable auto offset compensation for the ALS
     */
    public synchronized void setAmbientLightAutoOffsetCompensationEnabled(boolean en)
    {

    }

    //----------------------------------------------------------------------------------------------------------------------------------------------
    // Misc
    //----------------------------------------------------------------------------------------------------------------------------------------------

    /***
     * Flushes the current command register configuration
     * to the device
     */
    private void flushCmdReg()
    {
        deviceClient.write8(REG_COMMAND, cmdRegisterData);
    }

    /***
     * Flushes the current ALS param register configuration
     * to the device
     */
    private void flushAlsParamReg()
    {
        deviceClient.write8(REG_ALS_PARAM, alsParamRegisterData);
    }

    /***
     * Extracts the high nibble of a byte and shifts it to
     * the low nibble position
     *
     * @param theByte the byte to extract the high nibble from
     *
     * @return the high nibble of a byte, shifted to the
     *         low nibble position
     */
    private byte extractHighNibble(byte theByte)
    {
        theByte &= 0b11110000; //Mask out the low nibble, we're interested in the high nibble
        theByte >>= 4; //Shift the high nibble into the low nibble position

        return theByte;
    }

    /***
     * Extracts the low nibble of a byte
     *
     * @param theByte the byte to extract the low nibble from
     *
     * @return the low nibble of the byte
     */
    private byte extractLowNibble(byte theByte)
    {
        theByte &= 0b0001111; //Mask out the high nibble, we're interested in the low nibble

        return theByte;
    }
}