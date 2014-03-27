/*
 * Copyright Luke Wallin 2012
 */

package pidcc;

import java.nio.ByteBuffer;

/**
 *
 * @author Luke
 */
public class SpeedMessage extends Message{
    
    private boolean forwards;
    private int speed;
    
    public SpeedMessage(Train train,boolean forwards, int speed){
        super(train,0);
        this.forwards=forwards;
        this.speed=speed;
    }
    
    public ByteBuffer getByteBuffer(){
        
        ByteBuffer bb = getHeader(SET_SPEED);
        
        bb.put((byte)(this.speed & 0xff));
        bb.put((byte)(this.forwards ? 1 : 0));
        
        addFooter(bb);

        return bb;
    }
}
