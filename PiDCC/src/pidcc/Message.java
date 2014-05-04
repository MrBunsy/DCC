/*
 * Copyright Luke Wallin 2012
 */
package pidcc;

import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.channels.Channels;
import java.nio.channels.WritableByteChannel;

/**
 *
 * @author Luke
 */
public class Message {

    public static int MESSAGE_SIZE = (8+4), MAX_PRIORITY=255;
    
    public static Train BROADCAST_TRAIN = new Train(0);
    
    
    
    public enum MessageType{
        PROGRAMME_ADDRESS (0),
        SET_SPEED (1),
        ENABLE_LIGHTS (2),
        EMERGANCY_STOP (3);
        
        private final int id;
        
        MessageType(int id){
            this.id=id;
        }
        
        int id(){
            return this.id;
        }
    }
    
    public MessageType messageType;
    
    protected Train train;
    protected int priority;

    public Message(Train train, int priority) {
        this.train = train;
        this.priority = priority;
    }

    public ByteBuffer getHeader() {
        ByteBuffer b=  ByteBuffer.allocate(MESSAGE_SIZE);
        //sync bytes
        b.put((byte)0xff);
        b.put((byte)0xcc);
        b.put((byte)0xcc);
        b.put((byte)0xff);
        //message type byte
        b.put((byte)(messageType.id() & 0xff));
        //priority byte
        b.put((byte) (priority & 0xff));
        //address byte
        b.put((byte) (train.getAddress() & 0xff));
        
        
        return b;
    }

    /**
     * messages are different sizes, but all are read in fully on the AVR side (for better or worse as a design decision...)
     * so pad out the missing bytes
     * @param buffer 
     */
    public void addFooter(ByteBuffer buffer){
        while(buffer.remaining() > 0){
            //System.out.println("pad");
            buffer.put((byte)0x00);
        }
        buffer.flip();
    }
    
    public ByteBuffer getByteBuffer() {
        return ByteBuffer.allocate(MESSAGE_SIZE);
    }

    public static void writeBuffer(ByteBuffer buffer, OutputStream stream) throws IOException {
        WritableByteChannel channel = Channels.newChannel(stream);
 
        channel.write(buffer);
        stream.flush();
    }
}
