/*
 * Copyright Luke Wallin 2012
 */

package pidcc;

/**
 *
 * @author Luke
 */
public interface CommLink {
    void sendMessage(Message message);
    
    //TBD
    void receivedMessage(byte[] bytes, int len);
}
