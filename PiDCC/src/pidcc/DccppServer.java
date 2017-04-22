package pidcc;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.logging.Level;
import java.util.logging.Logger;
import com.google.gson.Gson;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Date;
import java.util.List;

import static pidcc.SimpleDCCPacket.MESSAGE_SIZE;

/**
 * Take ascii input and give out binary to pipe directly to the AVR (DCC packets
 * with a header) Also query anything that needs a response and spit out ASCII
 * for that
 *
 * trying single threaded for now
 *
 * @author Luke
 */
public class DccppServer extends SocketCommsServer {

    //currently a static list, TODO make it dynamic now we're not using the register system.
    private ArrayList<Cab> cabList;
//    private final static int MAX_MAIN_REGISTERS = 100;
    private boolean mainTrackEnabled = false;
    private boolean progTrackEnabled = false;
    private ArrayBlockingQueue<ByteBuffer> uartQueue;
    private final static int UART_QUEUE_LENGTH = 10;
    private ArrayList<Turnout> turnoutList;
    private int current = 0;
    private int shiftRegisterLength;
    private TCPReadThread tcpRead;
    private UARTWriteThread uartWrite;
    private UARTReadThread uartRead;
    private final Gson gson = new Gson();
//    private File settingsFile;
    private String settingsFilePath;
    private final static double currentFilterAlpha = 0.5;
    //how many times to repeat a packet to programme a CV in ops mode
    public final static int REPEAT_OPS_MODE_PROGRAMMING = 4;
    private ServiceModeRequest serviceModeRequest;

    public DccppServer(Socket socket, TwoWaySerialComm serialComms, String settingsFilePath, int shiftRegisterLength) {
        super(socket, serialComms);

        this.cabList = new ArrayList<>();
        this.uartQueue = new ArrayBlockingQueue<>(UART_QUEUE_LENGTH);
        this.turnoutList = new ArrayList<>();
        //set from command line, but overriden if a settings file is provided.
        this.shiftRegisterLength = shiftRegisterLength;
        this.settingsFilePath = settingsFilePath;
        try {
//            reader = new FileReader(settingsFilePath);
            List<String> jsonLines = Files.readAllLines(Paths.get(settingsFilePath));
            String json = "";
            for (String line : jsonLines) {
                json += line;
            }
            loadJson(json);

        } catch (IOException ex) {
            System.out.println("Settings file not found or failed to be processed: " + settingsFilePath + " (" + ex.getMessage() + ")");
        }
        serviceModeRequest = null;
        updateShiftRegister();
//        reader.r

//        for (int i = 0; i < MAX_MAIN_REGISTERS; i++) {
//            this.cabList[i] = new Cab();
//        }
    }

    private void loadJson(String json) {
        Gson gson = new Gson();
        StoredState state = gson.fromJson(json, StoredState.class);
        shiftRegisterLength = state.shiftRegisterLength;
        turnoutList = state.turnouts;
    }

    public void updateCurrentDraw(int current) {
        //this.current = current;
        this.current = (int) Math.round(currentFilterAlpha * current + (1 - currentFilterAlpha) * this.current);
    }

    public void stopEverything() {
        uartWrite.stop();
        uartRead.stop();
        //this is probably the one to have called stopeverything, but just for completeness!
        tcpRead.stop();
        stop();
        //pop a posion pill on the queue so that uartWrite isn't stuck waiting for an empty queue for ever
        //stand in poision pill that the AVR won't care about for now!
        //IDEA - last message is also a shutdown power for the track!
        queueMessage(SimpleDCCPacket.requestAVRPacketBufferSize());
    }

    /**
     * run until finished, then return
     */
    @Override
    public void run() {

        //fire up a load of threads
        //read the TCP stream IN (this will call processDccppCommand)
        tcpRead = new TCPReadThread();
        (new Thread(tcpRead)).start();

        //write UART queue OUT (this will write everthing in uartQueue)
        uartWrite = new UARTWriteThread();
        (new Thread(uartWrite)).start();

        //read UART IN (this will call processUARTCommand)
        uartRead = new UARTReadThread();
        (new Thread(uartRead)).start();

//        transmitMessageNow(SimpleDCCPacket.setShiftRegisterLength(1));
//        transmitMessageNow(SimpleDCCPacket.setShiftRegisterLength(1));
//        transmitMessageNow(SimpleDCCPacket.setShiftRegisterLength(1));
        queueMessage(SimpleDCCPacket.setShiftRegisterLength(3));
////        byte[] testdata = 
//        transmitMessageNow(SimpleDCCPacket.setShiftRegisterData(0, new byte[]{(byte) 0xff, (byte) 0xff, (byte) 0xff}));
//
//        transmitMessageNow(SimpleDCCPacket.outputShiftRegister());

        running = true;
        while (running) {
            if (uartQueue.isEmpty()) {

//                fillUARTQueue();
                requestAVRPacketBufferSize();
                checkServiceModeTimeouts();
            }
            try {
                //TODO ...do this better. A proper wait?
                Thread.sleep(100);
            } catch (InterruptedException ex) {
                Logger.getLogger(DccppServer.class.getName()).log(Level.SEVERE, null, ex);
            }
        }

        System.out.println("Stopping DCC++ Server");
    }

    /**
     * Check to see if we haven't received a response from a service mode request
     */
    private void checkServiceModeTimeouts(){
        if(this.serviceModeRequest !=null){
            Date now = new Date();
            long seconds = (now.getTime()-serviceModeRequest.start.getTime())/1000;
            if(seconds > ServiceModeRequest.TIMEOUT_SECONDS){
                //todo: generalise from just readcv
                
                returnString("<r" + serviceModeRequest.callback + "|" + serviceModeRequest.callbacksub + "|" + serviceModeRequest.cv + " " + 0 + ">");
                serviceModeRequest=null;
            }
        }
    }
    
    public void requestAVRPacketBufferSize() {
//        try {
            //uartQueue.put(SimpleDCCPacket.requestAVRPacketBufferSize());
            queueMessage(SimpleDCCPacket.requestAVRPacketBufferSize());
//        } catch (InterruptedException ex) {
//            Logger.getLogger(DccppServer.class.getName()).log(Level.SEVERE, null, ex);
//        }
    }

    /**
     * Trundle through the registers to generate some packets to give to the AVR
     * TODO locks! the registers are accessed from two different threads
     */
    public void fillUARTQueueWithRegisterInfo() {
        ArrayList<ByteBuffer> messages = new ArrayList<>();

        //trundle through the registers to get some new messages to add to the queue
        for (Cab r : this.cabList) {
            if (r.inUse()) {
                ArrayList<ByteBuffer> registerMessages = r.getSimpleDCCPackets();
                messages.addAll(registerMessages);

            }
        }
        for (ByteBuffer message : messages) {
//            try {
                //TODO check queue large enough, with ltos of registers could eaisly not be
                //need proper system there
                //uartQueue.put(message);
                queueMessage(message);
//            } catch (InterruptedException ex) {
//                Logger.getLogger(DccppServer.class.getName()).log(Level.SEVERE, null, ex);
//            }
        }
    }

    /**
     * Sends a list of all turnouts to the DCCPP client over TCP
     */
    public void listAllTurnouts() {
        //< H ID ADDRESS SUBADDRESS THROW > 
        // or <H ID THROW> ?
        //or <X>
        if (turnoutList.isEmpty()) {
            returnString("<X>");
        } else {
            for (Turnout t : turnoutList) {
                returnString("<H " + t.getId() + " " + (t.getThrown() ? "1" : "0") + ">");//+ t.getAddress() + " " + t.getSubAddress() + " "
            }
        }
    }

    /**
     * retransmit the new state of the shift register
     *
     * TODO - have a shiftRegisterItem interface, supply that and only send part
     * of the shift register, rather than the whole lot?
     */
    public void updateShiftRegister() {
        byte[] data = new byte[shiftRegisterLength];

        //is this necessary?
        for (int i = 0; i < shiftRegisterLength; i++) {
            data[i] = 0;
        }

        for (Turnout t : turnoutList) {
            if (t.getAddress() < shiftRegisterLength && t.getSubAddress() < 8) {
                //set the relevant bit
                data[t.getAddress()] |= (byte) (((t.getThrown() ? 1 : 0) << t.getSubAddress()) & 0xff);
            }
        }

        //TODO probably not do this every time?
        queueMessage(SimpleDCCPacket.setShiftRegisterLength(shiftRegisterLength));

        //transmit the shift register in chunks
        for (int i = 0; i < SimpleDCCPacket.SHIFT_REG_BYTES_PER_MESSAGE; i += SimpleDCCPacket.SHIFT_REG_BYTES_PER_MESSAGE) {
            int endOfRange = i + SimpleDCCPacket.SHIFT_REG_BYTES_PER_MESSAGE;
            if (endOfRange > shiftRegisterLength) {
                endOfRange = shiftRegisterLength;
            }
            queueMessage(SimpleDCCPacket.setShiftRegisterData(i, Arrays.copyOfRange(data, i, endOfRange)));

        }

        queueMessage(SimpleDCCPacket.outputShiftRegister());
    }

    /**
     * strip any unnecessary whitespace, so all valid commands should be of the
     * same format
     *
     * @param command
     * @return tided up command
     */
    private String tidyCommand(String command) {
        //remove leading and trailing whitespace
        String tidyCommand = command.trim();

        //remove optional whitespace
        tidyCommand = tidyCommand.replace("< ", "<");
        tidyCommand = tidyCommand.replace(" >", ">");

        //only required whitespace should now remain
        return tidyCommand;
    }

    /**
     * Send a string back to the socket
     *
     * TODO use this in a thread?
     *
     * @param returnCommand
     */
    private void returnString(String returnCommand) {
        Logger.getLogger(DccppServer.class.getName()).log(Level.INFO, "RETURN: " + returnCommand);
        try {
            out.write(returnCommand.getBytes());
        } catch (IOException ex) {
            Logger.getLogger(DccppServer.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    /**
     * Find a register for a decoder address, if none found set up next register
     * and return it
     *
     * @param address
     * @return
     */
    private Cab findCabFor(int address) {
        for (Cab r : cabList) {
            if (r.getAddress() == address) {
                return r;
            }
        }
        //none in cablist, add new entry
        Cab c = new Cab();
        c.setAddress(address);
        cabList.add(c);
        return c;
    }

    private Turnout getTurnout(int id) {
        for (Turnout t : turnoutList) {
            if (t.getId() == id) {
                return t;
            }
        }
        return null;
    }

    /**
     * Transmit a SimpleDCC message to the AVR now
     *
     * @param packet
     */
    private void queueMessage(ByteBuffer message) {
        try {
            //send this one right now, to reduce delay from the throttle
            //I think this will be fine, as any old messges will be furhter ahead in the queue and any after this will ahve the new speedvalue
            if(this.serviceModeRequest == null){
            uartQueue.put(message);
            }else{
                System.out.println("Can't send message, in service mode");
            }
        } catch (InterruptedException ex) {
            Logger.getLogger(DccppServer.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public void setTrackPower(boolean power) {
        this.mainTrackEnabled = power;
        //this.progTrackEnabled = power;
        queueMessage(SimpleDCCPacket.setTrackPower(SimpleDCCPacket.MAIN_TRACK, power));
        //queueMessage(SimpleDCCPacket.setTrackPower(SimpleDCCPacket.PROG_TRACK, power));
    }

    private void processDccppCommand(String command) {
        command = tidyCommand(command);

        //remove encasing syntax
        command = command.replace("<", "");
        command = command.replace(">", "");

        String[] splitCommand = command.split(" ");

        Logger.getLogger(DccppServer.class.getName()).log(Level.INFO, "Received: " + command);
        if (splitCommand[0].length() > 1) {
            Logger.getLogger(DccppServer.class.getName()).log(Level.SEVERE, "Invalid command");
        }
        int register;
        int cabAddress;
        Cab cab;
        Turnout turnout;
        int id;
        int subaddress;
        byte[] nmra;

        switch (splitCommand[0].charAt(0)) {

            /**
             * *** SET ENGINE THROTTLES USING 128-STEP SPEED CONTROL ***
             */
            case 't':       // <t REGISTER CAB SPEED DIRECTION>
                /*
                *    sets the throttle for a given register/cab combination 
                *    
                *    REGISTER: an internal register number, from 1 through MAX_MAIN_REGISTERS (inclusive), to store the DCC packet used to control this throttle setting
                *    CAB:  the short (1-127) or long (128-10293) address of the engine decoder
                *    SPEED: throttle speed from 0-126, or -1 for emergency stop (resets SPEED to 0)
                *    DIRECTION: 1=forward, 0=reverse.  Setting direction when speed=0 or speed=-1 only effects directionality of cab lighting for a stopped train
                *    
                *    returns: <T REGISTER SPEED DIRECTION>
                *    
                 */
                //      mRegs->setThrottle(com+1);
                register = Integer.parseInt(splitCommand[1]);

                //IGNORING the register, we're caching it with the DCC address of the cab instead
                cabAddress = Integer.parseInt(splitCommand[2]);
                cab = findCabFor(cabAddress);
                int speed = Integer.parseInt(splitCommand[3]);
                boolean forwards = Integer.parseInt(splitCommand[4]) == 1;

                cab.setSpeed(speed, forwards);
                cab.setAddress(cabAddress);

                queueMessage(cab.getSpeedMessage());

                this.returnString("<T " + register + " " + speed + " " + (forwards ? 1 : 0) + ">");
                break;

            /**
             * *** OPERATE ENGINE DECODER FUNCTIONS F0-F28 ***
             */
            case 'f':       // <f CAB BYTE1 [BYTE2]>
                /*
                 *    turns on and off engine decoder functions F0-F28 (F0 is sometimes called FL)  
                 *    NOTE: setting requests transmitted directly to mobile engine decoder --- current state of engine functions is not stored by this program
                 *    
                 *    CAB:  the short (1-127) or long (128-10293) address of the engine decoder
                 *    
                 *    To set functions F0-F4 on (=1) or off (=0):
                 *      
                 *    BYTE1:  128 + F1*1 + F2*2 + F3*4 + F4*8 + F0*16
                 *    BYTE2:  omitted
                                binary: 1 0 0 f0 f4 f3 f2 f1
                 *   
                 *    To set functions F5-F8 on (=1) or off (=0):
                 *   
                 *    BYTE1:  176 + F5*1 + F6*2 + F7*4 + F8*8
                 *    BYTE2:  omitted
                                binary: 1 0 1 1 f8 f7 f6 f5
                 *   
                 *    To set functions F9-F12 on (=1) or off (=0):
                 *   
                 *    BYTE1:  160 + F9*1 +F10*2 + F11*4 + F12*8
                 *    BYTE2:  omitted
                                binary: 1 0 1 0 f12 f11 f10 f9
                 *   
                 *    To set functions F13-F20 on (=1) or off (=0):
                 *   
                 *    BYTE1: 222 (1101 1110)
                 *    BYTE2: F13*1 + F14*2 + F15*4 + F16*8 + F17*16 + F18*32 + F19*64 + F20*128
                 *   
                 *    To set functions F21-F28 on (=1) of off (=0):
                 *   
                 *    BYTE1: 223 (1101 1111)
                 *    BYTE2: F21*1 + F22*2 + F23*4 + F24*8 + F25*16 + F26*32 + F27*64 + F28*128
                 *   
                 *    returns: NONE
                 * 
                 */

                cabAddress = Integer.parseInt(splitCommand[1]);

                cab = findCabFor(cabAddress);

                int byte1 = Integer.parseInt(splitCommand[2]);
                int byte2 = 0;
                if (byte1 == 222 || byte1 == 223) {
                    byte2 = Integer.parseInt(splitCommand[3]);
                }
                if ((byte1 & 0xe0) == 128) {
                    //functions 0-4
                    //set the functions for the relevant bits
                    for (int i = 0; i < 4; i++) {
                        cab.setFunction(i + 1, (byte1 & 0x1 << i) > 0);
                    }
                    //lights :) (probably the only one that's actually ever going to be used)
                    cab.setFunction(0, (byte1 & 0x1 << 4) > 0);
                    //send it straight away
                    queueMessage(cab.getFunction0_4Message());
                } else if ((byte1 & 0xf0) == 176) {
                    //functions 5 to 8
                    for (int i = 0; i < 4; i++) {
                        cab.setFunction(i + 5, (byte1 & 0x1 << i) > 0);
                    }
                    queueMessage(cab.getFunction5_8Message());
                } else if ((byte1 & 0xf0) == 160) {
                    //functions 9 to 12
                    for (int i = 0; i < 4; i++) {
                        cab.setFunction(i + 9, (byte1 & 0x1 << i) > 0);
                    }
                    queueMessage(cab.getFunction9_12Message());
                } else if ((byte1 & 0xff) == 222) {
                    //functions F13-F20
                    for (int i = 0; i < 8; i++) {
                        cab.setFunction(i + 13, (byte2 & 0x1 << i) > 0);
                    }
                    queueMessage(cab.getFunction13_20Message());
                } else if ((byte1 & 0xff) == 223) {
                    //functions F21-F28
                    for (int i = 0; i < 8; i++) {
                        cab.setFunction(i + 21, (byte2 & 0x1 << i) > 0);
                    }
                    queueMessage(cab.getFunction21_28Message());
                }
                break;

            /**
             * *** OPERATE STATIONARY ACCESSORY DECODERS ***
             */
            case 'a':       // <a ADDRESS SUBADDRESS ACTIVATE>
                /*
                 *    turns an accessory (stationary) decoder on or off
                 *    
                 *    ADDRESS:  the primary address of the decoder (0-511)
                 *    SUBADDRESS: the subaddress of the decoder (0-3)
                 *    ACTIVATE: 1=on (set), 0=off (clear)
                 *    
                 *    Note that many decoders and controllers combine the ADDRESS and SUBADDRESS into a single number, N,
                 *    from  1 through a max of 2044, where
                 *    
                 *    N = (ADDRESS - 1) * 4 + SUBADDRESS + 1, for all ADDRESS>0
                 *    
                 *    OR
                 *    
                 *    ADDRESS = INT((N - 1) / 4) + 1
                 *    SUBADDRESS = (N - 1) % 4
                 *    
                 *    returns: NONE
                 */
                cabAddress = Integer.parseInt(splitCommand[1]);
                subaddress = Integer.parseInt(splitCommand[2]);
                int activate = Integer.parseInt(splitCommand[3]);
                //throwing in support for this, but no way of testing atm (don't have any accessory decoders!)
                //assuming that subaddress is the same as 'output' in NMRA/JMRI speak
                nmra = NmraPacket.accDecoderPkt(cabAddress, activate, subaddress);
                queueMessage(SimpleDCCPacket.createFromDCCPacket(nmra, Cab.REPEATS));
                break;

            /**
             * *** CREATE/EDIT/REMOVE/SHOW & OPERATE A TURN-OUT ***
             */
            case 'T':       // <T ID THROW>
                /*
                 *   <T ID THROW>:                sets turnout ID to either the "thrown" or "unthrown" position
                 *   
                 *   ID: the numeric ID (0-32767) of the turnout to control
                 *   THROW: 0 (unthrown) or 1 (thrown)
                 *   
                 *   returns: <H ID THROW> or <X> if turnout ID does not exist
                 *   
                 *   *** SEE ACCESSORIES.CPP FOR COMPLETE INFO ON THE DIFFERENT VARIATIONS OF THE "T" COMMAND
                 *   USED TO CREATE/EDIT/REMOVE/SHOW TURNOUT DEFINITIONS
                
                    Command to define a Turnout: < T ID ADDRESS SUBADDRESS >:
                    Creates a new turnout ID, with specified ADDRESS and SUBADDRESS if turnout ID already exists, it is updated (over written) with the new specified ADDRESS and SUBADDRESS
                    Returns: < O > if successful and < X > if unsuccessful (e.g. out of memory)
                    Command to Delete a turnout < T ID >:
                    Deletes the definition of a turnout with this ID
                    Returns: < O > if successful and < X > if unsuccessful (e.g. ID does not exist)
                    Command to List all defined turnouts: < T >:
                    Lists all defined turnouts.
                    Returns: < H ID ADDRESS SUBADDRESS THROW > for each defined turnout or < X > if no turnouts have beed defined or saved.
                    ID: The numeric ID (0-32767) of the turnout to control.
                    (You pick the ID & They ares shared between Turnouts, Sensors and Outputs)
                    ADDRESS: the primary address of the decoder controlling this turnout (0-511)
                    SUBADDRESS: the subaddress of the decoder controlling this turnout (0-3)
                 */
                switch (splitCommand.length) {
                    case 1:
                        //< T >
                        //listing all turnouts
                        listAllTurnouts();

                        break;
                    case 2:
                        //< T ID >
                        id = Integer.parseInt(splitCommand[1]);
                        //deleting a turnout
                        turnout = getTurnout(id);
                        turnoutList.remove(turnout);
                        returnString("<O>");
                        break;
                    case 3:
                        //<T ID THROWN>
                        //turning on or off
                        id = Integer.parseInt(splitCommand[1]);
                        boolean thrown = Integer.parseInt(splitCommand[2]) == 1;
                        //TODO - combine with my plans for controlling points!
                        turnout = getTurnout(id);
                        if (turnout != null) {
                            turnout.setThrown(thrown);
                            //seems to be in contrast to what hte wiki says, but this is what the code does in PacketRegister.cpp
                            returnString("<H " + id + " " + (thrown ? "1" : "0") + ">");
                        } else {
                            returnString("<X>");
                        }
                        break;
                    case 4:
                        //<T ID ADDR SUBADDR>
                        //adding new turnout
                        id = Integer.parseInt(splitCommand[1]);
                        //byte number
                        int address = Integer.parseInt(splitCommand[2]);
                        //byte position
                        subaddress = Integer.parseInt(splitCommand[3]);

                        turnout = new Turnout(id, address, subaddress);
                        turnoutList.add(turnout);
                        //seems to be what is returned
                        returnString("<O>");
                        break;
                }

                updateShiftRegister();

                break;

            /**
             * *** CREATE/EDIT/REMOVE/SHOW & OPERATE AN OUTPUT PIN ***
             */
            case 'Z':       // <Z ID ACTIVATE>
                /*
                 *   <Z ID ACTIVATE>:          sets output ID to either the "active" or "inactive" state
                 *   
                 *   ID: the numeric ID (0-32767) of the output to control
                 *   ACTIVATE: 0 (active) or 1 (inactive)
                 *   
                 *   returns: <Y ID ACTIVATE> or <X> if output ID does not exist
                 *   
                 *   *** SEE OUTPUTS.CPP FOR COMPLETE INFO ON THE DIFFERENT VARIATIONS OF THE "O" COMMAND
                 *   USED TO CREATE/EDIT/REMOVE/SHOW TURNOUT DEFINITIONS
                 */
                //just say it doesn't exist, no plans to support this yet
                //no particular reason why I couldn't GPIO on the AVR, but I'm not going to need it and it strikes me as a niche feature
                this.returnString("<X>");
                break;

            /**
             * *** CREATE/EDIT/REMOVE/SHOW A SENSOR ***
             */
            case 'S':
                /*   
                *   *** SEE SENSOR.CPP FOR COMPLETE INFO ON THE DIFFERENT VARIATIONS OF THE "S" COMMAND
                *   USED TO CREATE/EDIT/REMOVE/SHOW SENSOR DEFINITIONS
                 */

                //major TODO, but not urgent
                //might never support this, and instead design a new system for intergrating the computer vision sensors?
                //return none for now
                this.returnString("<X>");
                break;

            /**
             * *** SHOW STATUS OF ALL SENSORS ***
             */
            case 'Q':         // <Q>
                /*
                 *    returns: the status of each sensor ID in the form <Q ID> (active) or <q ID> (not active)
                 */
                this.returnString("<X>");
                break;

            /**
             * *** WRITE CONFIGURATION VARIABLE BYTE TO ENGINE DECODER ON MAIN
             * OPERATIONS TRACK ***
             */
            case 'w': // <w CAB CV VALUE>
            /*
                 *    writes, without any verification, a Configuration Variable to the decoder of an engine on the main operations track
                 *    
                 *    CAB:  the short (1-127) or long (128-10293) address of the engine decoder 
                 *    CV: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
                 *    VALUE: the value to be written to the Configuration Variable memory location (0-255)
                 *    
                 *    returns: NONE
             */ {
                cabAddress = Integer.parseInt(splitCommand[1]);
                int cv = Integer.parseInt(splitCommand[2]);
                int value = Integer.parseInt(splitCommand[3]);
                nmra = NmraPacket.opsCvWriteByte(cabAddress, cabAddress > 127, cv, value);

                queueMessage(SimpleDCCPacket.createFromDCCPacket(nmra, REPEAT_OPS_MODE_PROGRAMMING));
            }
            break;

            /**
             * *** WRITE CONFIGURATION VARIABLE BIT TO ENGINE DECODER ON MAIN
             * OPERATIONS TRACK ***
             */
            case 'b': // <b CAB CV BIT VALUE>
            /*
                 *    writes, without any verification, a single bit within a Configuration Variable to the decoder of an engine on the main operations track
                 *    
                 *    CAB:  the short (1-127) or long (128-10293) address of the engine decoder 
                 *    CV: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
                 *    BIT: the bit number of the Configurarion Variable regsiter to write (0-7)
                 *    VALUE: the value of the bit to be written (0-1)
                 *    
                 *    returns: NONE
             */ {
                cabAddress = Integer.parseInt(splitCommand[1]);
                int cv = Integer.parseInt(splitCommand[2]);
                int bit = Integer.parseInt(splitCommand[3]);
                int value = Integer.parseInt(splitCommand[4]);
                nmra = NmraPacket.opsCvWriteBit(cabAddress, cabAddress > 127, cv, value, bit);

                queueMessage(SimpleDCCPacket.createFromDCCPacket(nmra, REPEAT_OPS_MODE_PROGRAMMING));
            }
            break;

            /**
             * *** WRITE CONFIGURATION VARIABLE BYTE TO ENGINE DECODER ON
             * PROGRAMMING TRACK ***
             */
            case 'W':      // <W CV VALUE CALLBACKNUM CALLBACKSUB>
                /*
                 *    writes, and then verifies, a Configuration Variable to the decoder of an engine on the programming track
                 *    
                 *    CV: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
                 *    VALUE: the value to be written to the Configuration Variable memory location (0-255) 
                 *    CALLBACKNUM: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
                 *    CALLBACKSUB: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
                 *    
                 *    returns: <r CALLBACKNUM|CALLBACKSUB|CV Value)
                 *    where VALUE is a number from 0-255 as read from the requested CV, or -1 if verificaiton read fails
                 */
                //      pRegs->writeCVByte(com+1);
                //looking in the soruce code, those are ascii | symbols, not binary or
                break;

            /**
             * *** WRITE CONFIGURATION VARIABLE BIT TO ENGINE DECODER ON
             * PROGRAMMING TRACK ***
             */
            case 'B':      // <B CV BIT VALUE CALLBACKNUM CALLBACKSUB>
                /*
                 *    writes, and then verifies, a single bit within a Configuration Variable to the decoder of an engine on the programming track
                 *    
                 *    CV: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
                 *    BIT: the bit number of the Configurarion Variable memory location to write (0-7)
                 *    VALUE: the value of the bit to be written (0-1)
                 *    CALLBACKNUM: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
                 *    CALLBACKSUB: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
                 *    
                 *    returns: <r CALLBACKNUM|CALLBACKSUB|CV BIT VALUE)
                 *    where VALUE is a number from 0-1 as read from the requested CV bit, or -1 if verificaiton read fails
                 */
                //      pRegs->writeCVBit(com+1);
                break;

            /**
             * *** READ CONFIGURATION VARIABLE BYTE FROM ENGINE DECODER ON
             * PROGRAMMING TRACK ***
             */
            case 'R': // <R CV CALLBACKNUM CALLBACKSUB>
            /*    
                 *    reads a Configuration Variable from the decoder of an engine on the programming track
                 *    
                 *    CV: the number of the Configuration Variable memory location in the decoder to read from (1-1024)
                 *    CALLBACKNUM: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
                 *    CALLBACKSUB: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
                 *    
                 *    returns: <r CALLBACKNUM|CALLBACKSUB|CV VALUE)
                 *    where VALUE is a number from 0-255 as read from the requested CV, or -1 if read could not be verified
             */ {
                int cv = Integer.parseInt(splitCommand[1]);
                int callback = Integer.parseInt(splitCommand[2]);
                int callbacksub = Integer.parseInt(splitCommand[3]);
                queueMessage(SimpleDCCPacket.readCVDirectByte(cv, callback, callbacksub));
                serviceModeRequest=new ServiceModeRequest();
                serviceModeRequest.callback=callback;
                serviceModeRequest.callbacksub=callbacksub;
                serviceModeRequest.start=new Date();
                serviceModeRequest.cv = cv;
                //the AVR should respond, and that will be processed in the UART read thread
                //TODO a proper timeout system?
            }
            break;

            /**
             * *** TURN ON POWER FROM MOTOR SHIELD TO TRACKS ***
             */
            case '1':      // <1>
                /*   
                 *    enables power from the motor shield to the main operations and programming tracks
                 *    
                 *    returns: <p1>
                 */
                setTrackPower(true);
                returnString("<p1>");
                break;

            /**
             * *** TURN OFF POWER FROM MOTOR SHIELD TO TRACKS ***
             */
            case '0':     // <0>
                /*   
                 *    disables power from the motor shield to the main operations and programming tracks
                 *    
                 *    returns: <p0>
                 */
                setTrackPower(false);
                returnString("<p0>");
                break;

            /**
             * *** READ MAIN OPERATIONS TRACK CURRENT ***
             */
            case 'c':     // <c>
                /*
                 *    reads current being drawn on main operations track
                 *    
                 *    returns: <a CURRENT> 
                 *    where CURRENT = 0-1024, based on exponentially-smoothed weighting scheme
                 */
                //request a current read from the AVR and handle the return string in the AVR message processing section
//                transmitMessageNow(SimpleDCCPacket.requestCurrentDraw());
                //because of an issue I haven't got to the bottom of with transmitting too many messages from the AVR,
                // this current is now reported constantly with the buffer size check
                //therefore just immediately reply with the current current!
                //also avoids any timeouts if a message is lost
                //TODO leaky bucket filter?
                returnString("<a " + current + ">");
                break;

            /**
             * *** READ STATUS OF DCC++ BASE STATION ***
             */
            case 's':      // <s>
                /*
             *    returns status messages containing track power status, throttle status, turn-out status, and a version number
             *    NOTE: this is very useful as a first command for an interface to send to this sketch in order to verify connectivity and update any GUI to reflect actual throttle and turn-out settings
             *    
             *    returns: series of status messages that can be read by an interface to determine status of DCC++ Base Station and important settings
                 */
                //Dccpp only supports turning them both on or off, it seems
                if (this.mainTrackEnabled) {
                    this.returnString("<p1>");
                } else {
                    this.returnString("<p0>");
                }
                for (int i = 1; i < cabList.size(); i++) {
                    if (this.cabList.get(i).speed == 0) {
                        continue;
                    }
                    StringBuilder s = new StringBuilder();
                    s.append("<T");
                    //pretend this is a dccpp register
                    s.append(i);
                    s.append(" ");
                    if (this.cabList.get(i).speed > 0) {
                        s.append(this.cabList.get(i).speed);
                        s.append(" 1>");
                    } else {
                        s.append(-this.cabList.get(i).speed);
                        s.append(" 0>");
                    }
                    this.returnString(s.toString());
                }

                //TODO actual version info
                this.returnString("<iDCC++ BASE STATION FOR ARDUINO ");
                this.returnString("ATMEGA644");
                this.returnString(" / ");
                this.returnString("ARDUINO");
                this.returnString(": V-");
                this.returnString("1234");
                this.returnString(" / ");
                this.returnString("2017");
                this.returnString(" ");
                this.returnString("00:00");
                this.returnString(">");

                this.returnString("<N 1: " + this.socket.getInetAddress().toString().replace("/", "") + ">");
                //      Turnout::show();
                //      Output::show();
                //TODO support outputs and points
                listAllTurnouts();
                //outputs - none supported
                returnString("<X>");

                break;

            /**
             * *** STORE SETTINGS IN EEPROM ***
             */
            case 'E':     // <E>
                /*
                 *    stores settings for turnouts and sensors EEPROM
                 *    
                 *    returns: <e nTurnouts nSensors>
                 */
                StoredState storeMe = new StoredState();
                storeMe.shiftRegisterLength = shiftRegisterLength;
                storeMe.turnouts = turnoutList;
                String jsonString = gson.toJson(storeMe);
                 {
                    try {
                        //java 7 does python style try-with-resources!
                        try (FileWriter writer = new FileWriter(new File(settingsFilePath))) {
                            writer.write(jsonString);
                        }
                    } catch (IOException ex) {
                        Logger.getLogger(DccppServer.class.getName()).log(Level.SEVERE, null, ex);
                    }
                }
                //TODO write to file
                //TODO sensors?
                returnString("<e " + turnoutList.size() + " " + 0 + ">");

                break;

            /**
             * *** CLEAR SETTINGS IN EEPROM ***
             */
            case 'e':     // <e>
                /*
                 *    clears settings for Turnouts in EEPROM
                 *    
                 *    returns: <O>
                 */
                File settingsFile = new File(settingsFilePath);
                settingsFile.delete();

                returnString("<O>");
                break;

            /**
             * *** PRINT CARRIAGE RETURN IN SERIAL MONITOR WINDOW ***
             */
            case ' ':     // < >                
                /*
                 *    simply prints a carriage return - useful when interacting with Ardiuno through serial monitor window
                 *    
                 *    returns: a carriage return
                 */
                //might as well support this
                returnString("\r\n");
                break;

///          
/// THE FOLLOWING COMMANDS ARE NOT NEEDED FOR NORMAL OPERATIONS AND ARE ONLY USED FOR TESTING AND DEBUGGING PURPOSES
/// PLEASE SEE SPECIFIC WARNINGS IN EACH COMMAND BELOW
///
            /**
             * *** ENTER DIAGNOSTIC MODE ***
             */
            case 'D':       // <D>  
                /*
                 *    changes the clock speed of the chip and the pre-scaler for the timers so that you can visually see the DCC signals flickering with an LED
                 *    SERIAL COMMUNICAITON WILL BE INTERUPTED ONCE THIS COMMAND IS ISSUED - MUST RESET BOARD OR RE-OPEN SERIAL WINDOW TO RE-ESTABLISH COMMS
                 */
                break;

            /**
             * *** WRITE A DCC PACKET TO ONE OF THE REGSITERS DRIVING THE MAIN
             * OPERATIONS TRACK ***
             */
            case 'M':       // <M REGISTER BYTE1 BYTE2 [BYTE3] [BYTE4] [BYTE5]>
                /*
                 *   writes a DCC packet of two, three, four, or five hexidecimal bytes to a register driving the main operations track
                 *   FOR DEBUGGING AND TESTING PURPOSES ONLY.  DO NOT USE UNLESS YOU KNOW HOW TO CONSTRUCT NMRA DCC PACKETS - YOU CAN INADVERTENTLY RE-PROGRAM YOUR ENGINE DECODER
                 *   
                 *    REGISTER: an internal register number, from 0 through MAX_MAIN_REGISTERS (inclusive), to write (if REGISTER=0) or write and store (if REGISTER>0) the packet 
                 *    BYTE1:  first hexidecimal byte in the packet
                 *    BYTE2:  second hexidecimal byte in the packet
                 *    BYTE3:  optional third hexidecimal byte in the packet
                 *    BYTE4:  optional fourth hexidecimal byte in the packet
                 *    BYTE5:  optional fifth hexidecimal byte in the packet
                 *   
                 *    returns: NONE   
                 */
                //wouldn't be hard to support this
                //      mRegs->writeTextPacket(com+1);
                break;

            /**
             * *** WRITE A DCC PACKET TO ONE OF THE REGSITERS DRIVING THE
             * PROGRAMMING TRACK ***
             */
            case 'P':       // <P REGISTER BYTE1 BYTE2 [BYTE3] [BYTE4] [BYTE5]>
                /*
             *   writes a DCC packet of two, three, four, or five hexidecimal bytes to a register driving the programming track
             *   FOR DEBUGGING AND TESTING PURPOSES ONLY.  DO NOT USE UNLESS YOU KNOW HOW TO CONSTRUCT NMRA DCC PACKETS - YOU CAN INADVERTENTLY RE-PROGRAM YOUR ENGINE DECODER
             *   
             *    REGISTER: an internal register number, from 0 through MAX_MAIN_REGISTERS (inclusive), to write (if REGISTER=0) or write and store (if REGISTER>0) the packet 
             *    BYTE1:  first hexidecimal byte in the packet
             *    BYTE2:  second hexidecimal byte in the packet
             *    BYTE3:  optional third hexidecimal byte in the packet
             *    BYTE4:  optional fourth hexidecimal byte in the packet
             *    BYTE5:  optional fifth hexidecimal byte in the packet
             *   
             *    returns: NONE   
                 */
//      pRegs->writeTextPacket(com+1);
                break;

            /**
             * *** ATTEMPTS TO DETERMINE HOW MUCH FREE SRAM IS AVAILABLE IN
             * ARDUINO ***
             */
            case 'F':     // <F>
                /*
                 *     measure amount of free SRAM memory left on the Arduino based on trick found on the internet.
                 *     Useful when setting dynamic array sizes, considering the Uno only has 2048 bytes of dynamic SRAM.
                 *     Unfortunately not very reliable --- would be great to find a better method
                 *     
                 *     returns: <f MEM>
                 *     where MEM is the number of free bytes remaining in the Arduino's SRAM
                 */
                //claim there's two gigs and see what happens
                returnString("<F 2147483648>");
                break;

            /**
             * *** LISTS BIT CONTENTS OF ALL INTERNAL DCC PACKET REGISTERS ***
             */
            case 'L':     // <L>
                /*
                 *    lists the packet contents of the main operations track registers and the programming track registers
                 *    FOR DIAGNOSTIC AND TESTING USE ONLY
                 */
                //no point supporting this
                break;

        } // switch

    }

    /**
     * take an array without sync bytes and check the crc byte is valid
     *
     * @param message from AVR
     * @return pass/fail
     */
    public boolean checkMessageCRC(byte[] message) {
        CRC16 crc = new CRC16();
        //last byte in the message is the CRC byte
        for (int i = 0; i < message.length - 1; i++) {
            crc.update(message[i]);
        }
        byte crcResult = crc.getCrc();
        return message[message.length - 1] == crcResult;
    }

    public void processUARTResponse(byte[] message) {
        if (!checkMessageCRC(message)) {
            Logger.getLogger(DccppServer.class.getName()).log(Level.INFO, "Message from AVR failed CRC");
            return;
        }
        int responseType = 0xff & message[0];

        switch (responseType) {
            case SimpleDCCPacket.RESPONSE_PACKET_BUFFER_SIZE:
                int packetsInBuffer = 0xff & message[1];

                int currentDraw = (0xff & message[2]);// | (message[3] << 8);
                //dccpp assumes reading the full 10 bits of the AVR's ADC, I only use 8bits, so shift left
                currentDraw = currentDraw << 2;

                updateCurrentDraw(currentDraw);

//                System.out.println("Received current draw of " + currentDraw);
                //gain of 11 on voltage over 0.15ohm resistor
                //1024bit ADC 0-3.3v (assuming 3.3v supply)
                double voltsMeasured = (((double) current) / 1024) * 3.3;
                double amps = (voltsMeasured / 11.0) / 0.15;

                //Logger.getLogger(DccppServer.class.getName()).log(Level.INFO, "packets in buffer on AVR: {0}. Current draw filtered:" + this.current + " new: "+currentDraw+" = " + amps + "A", packetsInBuffer);
                if (packetsInBuffer < 5) {
                    fillUARTQueueWithRegisterInfo();
                }
                break;
            case SimpleDCCPacket.RESPONSE_COMMS_ERROR:
                int errorType = 0xff & message[1];
                Logger.getLogger(DccppServer.class.getName()).log(Level.INFO, "Comms error from AVR type: {0}", errorType);
                break;
            case SimpleDCCPacket.RESPONSE_CV_READ: {
                int cv = (message[1] & 0xff) | ((message[2] & 0xff) << 8);
                int cvValue = message[3] & 0xff;
                int callBackNum = (message[4] & 0xff) | ((message[5] & 0xff) << 8);
                int callBackSub = (message[6] & 0xff) | ((message[7] & 0xff) << 8);
                //TODO, going to have to track callbacks or edit the messaging :(

                //<r CALLBACKNUM|CALLBACKSUB|CV VALUE>
                //seems to need no space between the r and callback?
                returnString("<r" + callBackNum + "|" + callBackSub + "|" + cv + " " + cvValue + ">");
                //go back into normal mode
                serviceModeRequest=null;

            }
            break;
//            case SimpleDCCPacket.RESPONSE_CURRENT:
//                int currentDraw = 0xff & message[1];
//                //dccpp assumes reading the full 10 bits of the AVR's ADC, I only use 8bits, so shift left
//                currentDraw = currentDraw << 2;
//                returnString("<a "+currentDraw+">");
//                System.out.println("Received current draw of "+currentDraw);
        }
    }

    class TCPReadThread implements Runnable {

        private boolean running;

        public void stop() {
            running = false;
        }

        @Override
        public void run() {
            running = true;
            while (running) {
                StringBuilder sb = new StringBuilder();
                do {
                    try {
                        sb.append((char) in.read());

                    } catch (IOException ex) {
                        //Logger.getLogger(DccppServer.class.getName()).log(Level.SEVERE, null, ex);
                        System.out.println("Lost TCP Connection: " + ex.getMessage());
                        running = false;
                    }
                } while (running && sb.indexOf(">") < 0);

                //sb has collected an entire instruction!
                if (running) {
                    processDccppCommand(sb.toString());
                } else {
                    //got here and not running any more, power off the track
                    setTrackPower(false);
                }
            }
            //stop everything else
            stopEverything();
        }
    }

    class UARTReadThread implements Runnable {

        private boolean running;
        private InputStream streamIn;

        /**
         * this thread will grab messages from uartqueue and put them on the
         * UART
         *
         * @param serialComms
         * @param uartQueue
         */
        public UARTReadThread() {
            this.streamIn = serialComms.getInputStream();
        }

        public void stop() {
            running = false;
        }

        /**
         * Keep reading UART until we've finished a sync message
         */
        void readUntilSync() throws IOException {

            int in;
            int syncPos = 0;
            while (true) {
                in = streamIn.read();

                if ((in & 0xff) == SimpleDCCPacket.syncBytes[syncPos]) {
                    syncPos++;

                    if (syncPos == SimpleDCCPacket.SYNC_BYTES) {
                        //read all the sync bytes
                        return;
                    }
                } else {
                    //failed, keep reading
                    syncPos = 0;
                }
            }
        }

        @Override
        public void run() {
            running = true;
            while (running) {
                try {
                    readUntilSync();
                    ByteBuffer message = ByteBuffer.allocate(MESSAGE_SIZE - SimpleDCCPacket.SYNC_BYTES);

                    for (int i = 0; i < MESSAGE_SIZE - SimpleDCCPacket.SYNC_BYTES; i++) {
                        message.put((byte) (streamIn.read() & 0xff));
                    }
                    processUARTResponse(message.array());
                } catch (IOException ex) {
                    Logger.getLogger(DccppServer.class.getName()).log(Level.SEVERE, null, ex);
                    this.stop();
                }
            }

            this.stop();
        }

    }

    class UARTWriteThread implements Runnable {

        private boolean running;
        private OutputStream streamOut;

        /**
         * this thread will grab messages from uartqueue and put them on the
         * UART
         *
         * @param serialComms
         * @param uartQueue
         */
        public UARTWriteThread() {
            this.streamOut = serialComms.getOutputStream();
        }

        public void stop() {
            running = false;
        }

        @Override
        public void run() {
            running = true;
            while (running) {

                try {
                    ByteBuffer message = uartQueue.take();
//                    System.out.println("writing UART out");
                    byte[] messageBytes = message.array();
//                    System.out.println(messageBytes.length);
                    streamOut.write(messageBytes);
                    streamOut.flush();
                } catch (InterruptedException | IOException ex) {
                    Logger.getLogger(DccppServer.class.getName()).log(Level.SEVERE, null, ex);
                    this.stop();
                }

            }
            this.stop();
        }

    }

    class ServiceModeRequest {
        public static final int TIMEOUT_SECONDS = 5;
        public int callback;
        public int callbacksub;
        public int cv;
        public Date start;
    }
}
