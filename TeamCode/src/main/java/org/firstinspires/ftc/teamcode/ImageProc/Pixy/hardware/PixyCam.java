package org.firstinspires.ftc.teamcode.ImageProc.Pixy.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

/**
 * Source code brought in from:
 * https://github.com/Overlake-FTC-7330-2017/ftc_app/blob/TeamCode/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/pixycam/PixyCam.java
 * on 2017-10-19
 */
@I2cSensor(name = "PixyCam", description = "PixyCam", xmlTag = "PixyCam")
public class PixyCam extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, PixyCam.PixyCamParams> {

    static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x54);

    private static final int LEGO_QUERY_BASE = 0x50;
    private static final int LEGO_QUERY_CC = 0x58;
    private static final int EXTENDED_QUERY_BASE = 0x70;
    private static final int EXTENDED_QUERY_CC = 0x78;
    private static final int LEGO_QUERY_COUNT = 6;
    private static final int LEGO_QUERY_SIG_COUNT = 5;
    private static final int LEGO_QUERY_CC_COUNT = 6;
    private static final int EXTENDED_QUERY_COUNT = 26;
    private static final int EXTENDED_QUERY_BLOCK_SIZE = 5;
    private static final int EXTENDED_QUERY_SIG_COUNT = 25;
    private static final int EXTENDED_QUERY_SIG_BLOCK_SIZE = 4;
    private static final int EXTENDED_QUERY_CC_COUNT = 25;
    private static final int EXTENDED_QUERY_CC_BLOCK_SIZE = 6;
    private static final int MIN_SIGNATURE = 1;
    private static final int MAX_SIGNATURE = 7;

    public static class PixyCamParams implements Cloneable {

        I2cAddr i2cAddr = ADDRESS_I2C_DEFAULT;

        public PixyCamParams clone() {
            try {
                return (PixyCamParams) super.clone();
            } catch (CloneNotSupportedException e) {
                throw new RuntimeException("Internal Error: Parameters not cloneable");
            }
        }
    }

    /**
     * The ReadWindow used to do a PixyCam LEGO protocol GeneralQuery
     */
    private I2cDeviceSynch.ReadWindow legoProtocolGeneralQueryReadWindow;
    /**
     * The ReadWindows used to do the PixyCam LEGO protocol SignatureQuery.
     */
    private I2cDeviceSynch.ReadWindow[] legoProtocolSignatureQueryReadWindows;

    /*
     * The ReadWindow used to do the extneded query for the enhanced FTC firmware
     */
    private I2cDeviceSynch.ReadWindow extendedGeneralQueryReadWindow;
    /**
     * The ReadWindows used to do the PixyCam LEGO protocol SignatureQuery.
     */
    private I2cDeviceSynch.ReadWindow[] extnededSignatureQueryReadWindows;

    public PixyCam(I2cDeviceSynch deviceSynch) {
        super(deviceSynch, true, new PixyCamParams());

        this.legoProtocolGeneralQueryReadWindow = new I2cDeviceSynch.ReadWindow(LEGO_QUERY_BASE, LEGO_QUERY_COUNT, I2cDeviceSynch.ReadMode.ONLY_ONCE);
        this.legoProtocolSignatureQueryReadWindows = new I2cDeviceSynch.ReadWindow[MAX_SIGNATURE];
        for (int i = MIN_SIGNATURE; i <= MAX_SIGNATURE; i++) {
            this.legoProtocolSignatureQueryReadWindows[i - 1] = NewLegoProtocolSignatureQueryReadWindow(LEGO_QUERY_BASE, LEGO_QUERY_SIG_COUNT, i);
        }

        this.extendedGeneralQueryReadWindow = new I2cDeviceSynch.ReadWindow(EXTENDED_QUERY_BASE, EXTENDED_QUERY_COUNT, I2cDeviceSynch.ReadMode.ONLY_ONCE);
        this.extnededSignatureQueryReadWindows = new I2cDeviceSynch.ReadWindow[MAX_SIGNATURE];
        for (int i = MIN_SIGNATURE; i <= MAX_SIGNATURE; i++) {
            this.extnededSignatureQueryReadWindows[i - 1] = NewLegoProtocolSignatureQueryReadWindow(EXTENDED_QUERY_BASE, EXTENDED_QUERY_SIG_COUNT, i);
        }

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    private I2cDeviceSynch.ReadWindow NewLegoProtocolSignatureQueryReadWindow(int base, int count, int signature) {
        return new I2cDeviceSynch.ReadWindow(base + signature, count, I2cDeviceSynch.ReadMode.ONLY_ONCE);
    }

    private byte[] ReadEntireWindow(I2cDeviceSynch.ReadWindow readWindow) {
        this.deviceClient.setReadWindow(readWindow);
        return this.deviceClient.read(readWindow.getRegisterFirst(), readWindow.getRegisterCount());
    }

    /***
     *
     * @return a Block object containing details about the location of the largest detected block
     */
    public PixyBlock getBiggestBlock() {
        byte[] buffer = ReadEntireWindow(this.legoProtocolGeneralQueryReadWindow);

        int signature = buffer[1] << 8 | buffer[0];

        return new PixyBlock(signature, -1, buffer[2], buffer[3], buffer[4], buffer[5]);
    }

    /**
     * @param signature is a value between 1 and 7 corresponding to the signature trained into the PixyCam.
     * @return a Block object containing details about the location of the largest detected block for the specified signature.
     */
    public PixyBlock getBiggestBlock(int signature) {
        if (signature < MIN_SIGNATURE || signature > MAX_SIGNATURE) {
            throw new IllegalArgumentException("signature must be between 1 and 7");
        }

        byte[] buffer = ReadEntireWindow(this.legoProtocolSignatureQueryReadWindows[signature - 1]);

        return new PixyBlock(signature, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
    }

    public PixyBlockList getBiggestBlocks() {
        byte[] buffer = ReadEntireWindow(this.extendedGeneralQueryReadWindow);

        PixyBlockList list = new PixyBlockList(buffer[0]);

        for (int i = 1; i < EXTENDED_QUERY_COUNT; i += EXTENDED_QUERY_BLOCK_SIZE) {
            int signature = TypeConversion.unsignedByteToInt(buffer[i]);

            list.add(new PixyBlock(signature, -1, buffer[i + 1], buffer[i + 2], buffer[i + 3], buffer[i + 4]));
        }

        return list;
    }

    public PixyBlockList getBiggestBlocks(int signature)
    {
        if (signature < MIN_SIGNATURE || signature > MAX_SIGNATURE) {
            throw new IllegalArgumentException("signature must be between 1 and 7");
        }

        byte[] buffer = ReadEntireWindow(this.extnededSignatureQueryReadWindows[signature - 1]);

        PixyBlockList list = new PixyBlockList(buffer[0]);

        for (int i = 1; i < EXTENDED_QUERY_SIG_COUNT; i += EXTENDED_QUERY_SIG_BLOCK_SIZE) {
            list.add(new PixyBlock(signature, -1, buffer[i], buffer[i + 1], buffer[i + 2], buffer[i + 3]));
        }

        return list;
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    protected boolean internalInitialize(@NonNull PixyCamParams pixyCamParams) {
        this.parameters = pixyCamParams.clone();
        deviceClient.setI2cAddress(pixyCamParams.i2cAddr);

        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "PixyCam";
    }
}