/*
 * Copyright Luke Wallin 2012
 */

package pidcc;

/**
 *
 * @author Luke
 */
public class StopAllMessage extends Message{

    public StopAllMessage() {
        super(BROADCAST_TRAIN,MAX_PRIORITY);
    }
    
}
