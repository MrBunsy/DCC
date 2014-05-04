/*
 * Copyright Luke Wallin 2012
 */

package pidcc;

import java.nio.ByteBuffer;

/**
 *
 * @author Luke
 * 
 * Very simple message, enables or disables the front light
 */
public class LightsMessage extends Message{
    private boolean on;
    public LightsMessage(Train train, boolean on) {
        super(train, 0);
        this.on=on;
        this.messageType=MessageType.ENABLE_LIGHTS;
    }
    
    @Override
    public ByteBuffer getByteBuffer(){
        ByteBuffer bb = getHeader();
        
        bb.put((byte)(on ? 0x1 : 0x0));
        
        addFooter(bb);
        
        return bb;
    }
    
}
