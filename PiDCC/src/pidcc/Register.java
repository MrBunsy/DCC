package pidcc;

import java.nio.ByteBuffer;
import java.util.ArrayList;

/**
 * Represents a DCC decoder. Holds speed, direction and functions
 *
 * TODO also have my own system for points and signals (think signals are
 * accessory decoders?)
 *
 * @author Luke
 */
public class Register {

    public int speed;
    public int address;
    public boolean forwards;
    private int maxFunctionsUsed;
    public boolean functions[];
    private static int REPEATS = 2;
//    private static int MAX_SPEED = 127; //using speed mode 128

    public Register() {
        this.functions = new boolean[24];
        this.maxFunctionsUsed = 0;
        this.address = 0;
    }

    /**
     * Is this register in use?
     *
     * @return
     */
    public boolean inUse() {
        return this.address != 0;
    }

    /**
     * Turn a function on and off, function 0 is usually the lights
     *
     * @param id id of function
     * @param on on or off
     */
    public void setFunction(int id, boolean on) {
        this.functions[id] = on;
        if (on) {
            //more packets are required to send the full set of functions, so if we're only using the lights track this and don't send all of the extended functions
            this.maxFunctionsUsed = Math.max(this.maxFunctionsUsed, id);
        }
    }

    /**
     * Set decoder (engine) speed and direction
     *
     * @param speed 0-126, or -1 for emergency stop (resets SPEED to 0) (not
     * supported yet)
     * @param forwards direction
     */
    public void setSpeed(int speed, boolean forwards) {
        this.forwards = forwards;
        this.speed = speed;
    }

    /**
     * Set address of decoder (engine) this register is controlling
     * @param address 
     */
    public void setAddress(int address) {
        this.address = address;
    }

    /**
     * return array of all simpleDCC packets ready to send to the AVR
     *
     * @return
     */
    public ArrayList<ByteBuffer> getSimpleDCCPackets() {
        ArrayList<ByteBuffer> packetArray = new ArrayList<ByteBuffer>();

        byte[] functionPacket = NmraPacket.function0Through4Packet(address, address > 127,
                this.functions[0], this.functions[1], this.functions[2], this.functions[03], this.functions[4]);

        packetArray.add(SimpleDCCPacket.createFromDCCPacket(functionPacket, REPEATS));

        //TODO other functions (not that I have any trains with them yet)
        //TODO allow emergency stop to work
        int value = (int) ((127 - 1) * speed);     // -1 for rescale to avoid estop
        if (value > 0) {
            value = value + 1;  // skip estop
        }
        if (value > 127) {
            value = 127;    // max possible speed
        }
        if (value < 0) {
            value = 1;        // emergency stop
        }
        byte[] speedPacket = NmraPacket.speedStep128Packet(address, address > 127, this.speed, this.forwards);

        packetArray.add(SimpleDCCPacket.createFromDCCPacket(speedPacket, REPEATS));

        return packetArray;

    }
}
