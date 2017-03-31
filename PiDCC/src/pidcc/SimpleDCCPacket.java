/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package pidcc;

import java.nio.ByteBuffer;

/**
 *
 * @author Luke
 */
public class SimpleDCCPacket {

    //TODO integrate with PiDCC
    //MESSAGE_SIZE includes sync bytes
    public static int SYNC_BYTES = 4, MESSAGE_SIZE = (10 + 4), OPERATIONS_MODE = 1, PROG_DIRECT_BYTE = 0,
            MAX_DATA_BYTES = 6, PROG_ADDRESS = 2;//note one more than on AVR because this includes address

    private static ByteBuffer createHeader() {
        ByteBuffer bb = ByteBuffer.allocate(MESSAGE_SIZE);
        //sync bytes
        bb.put((byte) 0xff);
        bb.put((byte) 0xcc);
        bb.put((byte) 0xcc);
        bb.put((byte) 0xff);

        return bb;
    }

    private static void addFooter(ByteBuffer bb) {
        //fill in the rest of the message with zeroes
        while (bb.remaining() > 0) {
            //System.out.println("pad");
            bb.put((byte) 0x00);
        }
        
        bb.flip();
    }
    /**
     * TODO details about this, can't remember anything about it
     * @param cv
     * @param newValue
     * @return 
     */
    public static ByteBuffer createProgrammeDirectByte(int cv, int newValue) {
        ByteBuffer bb = createHeader();

        //message type byte
        bb.put((byte) (PROG_DIRECT_BYTE & 0xff));

        //cv (uint16)
        //bb.putChar((char) cv);
        //lowest byte
        bb.put((byte)(cv & 0xff));
        //highest byte
        bb.put((byte)((cv >> 8) & 0xff));
        
        //new value (byte)
        bb.put((byte) newValue);

        addFooter(bb);
        
        
        
        return bb;
    }
    
    /**
     * For programming in address-only mode (need this for my n-gauge bachmann decoders)
     * @param newAddress
     * @return 
     */
    public static ByteBuffer createProgrammeAddress(int newAddress) {
        ByteBuffer bb = createHeader();

        //message type byte
        bb.put((byte) (PROG_ADDRESS & 0xff));

        bb.put((byte) (newAddress & 0xff));

        addFooter(bb);
        
        return bb;
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
        bb.put((byte) (OPERATIONS_MODE & 0xff));
//        //priority byte (repeats atm)
//        bb.put((byte) (1 & 0xff));

        //insert the bytes
        for (byte b : packet) {
            bb.put(b);
        }

        //fill rest of data with zeros
        for (int i = 0; i < MAX_DATA_BYTES - packet.length; i++) {
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
