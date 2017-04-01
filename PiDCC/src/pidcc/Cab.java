package pidcc;

import java.nio.ByteBuffer;
import java.util.ArrayList;

/**
 * Represents a DCC decoder. Holds speed, direction and functions
 * 
 * Dccpp has a system where this have their own index, despite always supplying cab address.
 * this doesn't mesh well when there isn't a register for lights, so I'm 
 * choosing to entirely ignore the register address and cache everythign based on decoder address
 * 
 * cab == decoder
 *
 * TODO also have my own system for points and signals (think signals are
 * accessory decoders?)
 *
 * @author Luke
 */
public class Cab {

    public int speed;
    public int address;
    public boolean forwards;
    private int maxFunctionsUsed;
    public boolean functions[];
    private static int REPEATS = 2;
//    private static int MAX_SPEED = 127; //using speed mode 128

    public Cab() {
        this.functions = new boolean[24];
        this.maxFunctionsUsed = 0;
        this.address = 0;
    }

    /**
     * Is this register in use?
     *
     * @return
     */
    public synchronized boolean inUse() {
        return this.address != 0;
    }

    /**
     * Turn a function on and off, function 0 is usually the lights
     *
     * @param id id of function
     * @param on on or off
     */
    public synchronized void setFunction(int id, boolean on) {
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
    public synchronized void setSpeed(int speed, boolean forwards) {
        this.forwards = forwards;
        this.speed = speed;
    }

    /**
     * Set address of decoder (engine) this register is controlling
     * @param address 
     */
    public synchronized void setAddress(int address) {
        this.address = address;
    }
    
    public synchronized int getAddress(){
        return address;
    }
    
    public synchronized ByteBuffer getSpeedMessage(){
        //TODO other functions (not that I have any trains with them yet)
        //TODO allow emergency stop to work
        int value = speed;//(int) ((127 - 1) * speed);     // -1 for rescale to avoid estop
        if (value > 0) {
            value = value + 1;  // skip estop
        }
        if (value > 127) {
            value = 127;    // max possible speed
        }
        if (value < 0) {
            value = 1;        // emergency stop
        }
        byte[] speedPacket = NmraPacket.speedStep128Packet(address, address > 127,  value, this.forwards);

        return SimpleDCCPacket.createFromDCCPacket(speedPacket, REPEATS);
    }
    
    public synchronized ByteBuffer getFunction0_4Message(){
        byte[] functionPacket = NmraPacket.function0Through4Packet(address, address > 127,
                this.functions[0], this.functions[1], this.functions[2], this.functions[03], this.functions[4]);
        return SimpleDCCPacket.createFromDCCPacket(functionPacket, REPEATS);
    }
    
    /**
     * return array of all simpleDCC packets ready to send to the AVR
     *
     * @return
     */
    public synchronized ArrayList<ByteBuffer> getSimpleDCCPackets() {
        ArrayList<ByteBuffer> packetArray = new ArrayList<ByteBuffer>();

        
        packetArray.add(getFunction0_4Message());

        
        
        packetArray.add(getSpeedMessage());

        return packetArray;

    }
}
