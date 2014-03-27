/*
 * Copyright Luke Wallin 2012
 */

package pidcc;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import gnu.io.*;
import java.util.logging.Level;
import java.util.logging.Logger;
/**
 *
 * @author Luke
 */
public class PiDCC extends TwoWaySerialComm{
    
    public PiDCC() throws Exception{
        connect("/dev/ttyS80");
        
        Train train = new Train(5);
        
        boolean b = true;
        
//        while(b){
//            out.write(128);
//        }
        
        while(true){
            try {
                Thread.sleep(1500);
                
                SpeedMessage s = new SpeedMessage(train, true, 80);
                Message.writeBuffer(s.getByteBuffer(), out,'a');
                Message.writeBuffer(s.getByteBuffer(), out,'b');
                
                Thread.sleep(1500);
                s = new SpeedMessage(train, true, 0);
                Message.writeBuffer(s.getByteBuffer(), out,'c');
                Message.writeBuffer(s.getByteBuffer(), out,'d');
                
                Thread.sleep(1500);
                s = new SpeedMessage(train, false, 80);
                Message.writeBuffer(s.getByteBuffer(), out,'e');
                Message.writeBuffer(s.getByteBuffer(), out,'f');
                
                Thread.sleep(1500);
                s = new SpeedMessage(train, false, 0);
                Message.writeBuffer(s.getByteBuffer(), out,'g');
                Message.writeBuffer(s.getByteBuffer(), out,'h');
                
            } catch (InterruptedException ex) {
                Logger.getLogger(PiDCC.class.getName()).log(Level.SEVERE, null, ex);
            } catch (IOException ex) {
                Logger.getLogger(PiDCC.class.getName()).log(Level.SEVERE, null, ex);
            }
            
        }
    }
    public static void main(String[] args) {
        try {
            
//            Enumeration t = CommPortIdentifier.getPortIdentifiers();
//            System.out.println("ports:");
//            while(t.hasMoreElements())
//            {
//                System.out.println(t.nextElement());
//            }
            PiDCC p = new PiDCC();
            //(new PiDCC()).connect("/dev/ttyS80");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
