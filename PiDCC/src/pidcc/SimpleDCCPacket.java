package pidcc;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 *
 * @author Luke
 */
public class SimpleDCCPacket {

    //TODO integrate with PiDCC and TIDY THIS MESS UP
    //MESSAGE_SIZE includes sync bytes
    public final static int SYNC_BYTES = 4;
    //9 is the size of the old message, 4 sync bytes, 1 CRC bytes
    public final static int MESSAGE_SIZE = (9 + 4 + 1);
    //MAX_MESSAGE_DATA_BYTES - 2
    public final static int SHIFT_REG_BYTES_PER_MESSAGE = (8 - 2);
    //this is the AVR's max data bytes + address, which we treat as a bit of data
    public final static int MAX_DCC_DATA_BYTES = 6;
    public final static int COMMAND_PROG_DIRECT_BYTE = 0,
            COMMAND_OPERATIONS_MODE_PACKET = 1,
            COMMAND_PROG_ADDRESS = 2,
            COMMAND_REQUEST_BUFFER_SIZE = 7,
            COMMAND_REQUEST_CURRENT = 8,
            COMMAND_SET_POWER = 9,
            COMMAND_SET_SHIFT_REGISTER_LENGTH = 10,
            COMMAND_SHIFT_REGISTER_DATA = 11,
            COMMAND_OUTPUT_SHIFT_REGISTER = 12,
            COMMAND_READ_CV = 13,
            COMMAND_PROG_DIRECT_BIT = 14;

    public final static int RESPONSE_PACKET_BUFFER_SIZE = 100,
            RESPONSE_COMMS_ERROR = 101,
            RESPONSE_CURRENT = 102,
            RESPONSE_CV = 103;

    public final static int CV_READ_PROG_TRACK = 0,
            CV_WRITE_BYTE_PROG_TRACK = 1,
            CV_WRITE_BIT_PROG_TRACK = 2;

    public final static int MAIN_TRACK = 0, PROG_TRACK = 1;

    public final static int[] syncBytes = {0xff, 0xcc, 0xcc, 0xff};

    private static ByteBuffer createHeader() {
        ByteBuffer bb = ByteBuffer.allocate(MESSAGE_SIZE);
        bb.order(ByteOrder.LITTLE_ENDIAN);
        //sync bytes
        bb.put((byte) 0xff);
        bb.put((byte) 0xcc);
        bb.put((byte) 0xcc);
        bb.put((byte) 0xff);

        return bb;
    }

    private static void addFooter(ByteBuffer bb) {
        //fill in the rest of the message with zeroes (except CRC byte at the end)
        while (bb.remaining() - 1 > 0) {
            //System.out.println("pad");
            bb.put((byte) 0x00);
        }
        //now generate the CRC byte
        byte[] messageArray = bb.array();

        CRC16 crc = new CRC16();

        //only generating from the message, ignoring sync bytes and the still-empty CRC byte
        for (int i = SYNC_BYTES; i < messageArray.length - 1; i++) {
            crc.update(messageArray[i]);
        }
        //now finally add the CRC byte
        bb.put(crc.getCrc());

        bb.flip();
    }

    /**
     * write a CV and provide callbacks
     *
     * @param cv address of CV (10bits)
     * @param newValue new value of CV (byte)
     * @param callback 16bit int to echo back with the response
     * @param subcallback another 16bit int to echo back with the response
     * @return
     */
    public static ByteBuffer writeCVDirectByte(int cv, int newValue, int callback, int subcallback) {
        ByteBuffer bb = createHeader();

        //message type byte
        bb.put((byte) (COMMAND_PROG_DIRECT_BYTE & 0xff));

//        //cv (uint16)
//        //bb.putChar((char) cv);
//        //lowest byte
//        bb.put((byte) (cv & 0xff));
//        //highest byte
//        bb.put((byte) ((cv >> 8) & 0xff));
//        
        //CV uint16 first
        bb.putShort((short) (cv & 0xffff));
        //callback uint16
        bb.putShort((short) (callback & 0xffff));

        //callbacksub  uint16
        bb.putShort((short) (subcallback & 0xffff));

        //new value (byte)
        bb.put((byte) newValue);

        addFooter(bb);

        return bb;
    }

    /**
     * write a single bit in a CV and provide callbacks
     *
     * @param cv address of CV (10bits)
     * @param bit which bit in the cv
     * @param newValue new value of bit in CV (1 or 0)
     * @param callback 16bit int to echo back with the response
     * @param subcallback another 16bit int to echo back with the response
     * @return
     */
    public static ByteBuffer writeCVDirectBit(int cv, int bit, int newValue, int callback, int subcallback) {
        ByteBuffer bb = createHeader();

        //message type byte
        bb.put((byte) (COMMAND_PROG_DIRECT_BIT & 0xff));
        
        //CV uint16 first
        bb.putShort((short) (cv & 0xffff));
        //callback uint16
        bb.putShort((short) (callback & 0xffff));

        //callbacksub  uint16
        bb.putShort((short) (subcallback & 0xffff));

        //new value (1 or 0)
        bb.put((byte) (newValue & 0x01));
        //which bit is being written to
        bb.put((byte) (bit & 0xff));

        addFooter(bb);

        return bb;
    }

    /**
     * read a CV and provide callbacks
     *
     * @param cv address of CV (10bits)
     * @param callback 16bit int to echo back with the response
     * @param subcallback another 16bit int to echo back with the response
     * @return
     */
    public static ByteBuffer readCVDirectByte(int cv, int callback, int subcallback) {
        ByteBuffer bb = createHeader();

        //message type byte
        bb.put((byte) (COMMAND_READ_CV & 0xff));

        //CV uint16 first
        bb.putShort((short) (cv & 0xffff));
        //callback uint16
        bb.putShort((short) (callback & 0xffff));

        //callbacksub  uint16
        bb.putShort((short) (subcallback & 0xffff));

        addFooter(bb);

        return bb;
    }

    /**
     * For programming in address-only mode (need this for my n-gauge bachmann
     * decoders)
     *
     * @param newAddress
     * @return
     */
    public static ByteBuffer createProgrammeAddress(int newAddress) {
        ByteBuffer bb = createHeader();

        //message type byte
        bb.put((byte) (COMMAND_PROG_ADDRESS & 0xff));

        bb.put((byte) (newAddress & 0xff));

        addFooter(bb);

        return bb;
    }

    public static ByteBuffer requestAVRPacketBufferSize() {
        return commandWithNoData(COMMAND_REQUEST_BUFFER_SIZE);
    }

    public static ByteBuffer outputShiftRegister() {
        return commandWithNoData(COMMAND_OUTPUT_SHIFT_REGISTER);
    }

    public static ByteBuffer setShiftRegisterLength(int length) {
        ByteBuffer bb = createHeader();

        bb.put((byte) (COMMAND_SET_SHIFT_REGISTER_LENGTH & 0xff));

        bb.putShort((short) (length & 0xffff));

        addFooter(bb);
        return bb;
    }

    public static ByteBuffer setShiftRegisterData(int startByte, byte[] data) {
        ByteBuffer bb = createHeader();

        bb.put((byte) (COMMAND_SHIFT_REGISTER_DATA & 0xff));

        bb.putShort((short) (startByte & 0xffff));

        //TODO check data is less than SHIFT_REG_BYTES_PER_MESSAGE long
        bb.put(data);

        addFooter(bb);
        return bb;
    }

    public static ByteBuffer commandWithNoData(int command) {
        ByteBuffer bb = createHeader();

        bb.put((byte) (command & 0xff));

        addFooter(bb);
        return bb;
    }

    /**
     * Turn programming or main track on or off
     *
     * @param trackType MAIN_TRACK or PROG_TRACK //TODO real enum?
     * @param powered true for powered, false for unpowered
     * @return
     */
    public static ByteBuffer setTrackPower(int trackType, boolean powered) {
        ByteBuffer bb = createHeader();

        bb.put((byte) (COMMAND_SET_POWER & 0xff));
        bb.put((byte) trackType);
        bb.put((byte) (powered ? 1 : 0));

        addFooter(bb);
        //System.out.println("crc = "+bb.array()[MESSAGE_SIZE-1]);
        return bb;
    }

    public static ByteBuffer requestCurrentDraw() {
        return commandWithNoData(COMMAND_REQUEST_CURRENT);
    }

    /**
     * Given a DCC packet, return a bytebuffer for sending to my DCC system
     *
     * @param packet
     * @param repeats how many times to tell the command station to repeat this
     * packet
     * @return
     */
    public static ByteBuffer createFromDCCPacket(byte[] packet, int repeats) {

        ByteBuffer bb = createHeader();
        /**
         *
         * Message format (in bytes): sync 1 0xff sync 2 0xcc sync 3 0xcc sync 4
         * 0xff command type (4 = custom packet) priority (unused atm) packet -
         * 6 bytes databytes - number of valid bytes in packet repeat (also
         * unused atm)
         */
        //message type byte
        bb.put((byte) (COMMAND_OPERATIONS_MODE_PACKET & 0xff));
//        //priority byte (repeats atm)
//        bb.put((byte) (1 & 0xff));

        //insert the bytes
        for (byte b : packet) {
            bb.put(b);
        }

        //fill rest of data with zeros
        for (int i = 0; i < MAX_DCC_DATA_BYTES - packet.length; i++) {
            bb.put((byte) 0);
        }
        //data bytes
        bb.put((byte) (packet.length & 0xff));

        //repeat
        bb.put((byte) (repeats & 0xff));

//        bb.flip();
        addFooter(bb);

        return bb;
    }
}
