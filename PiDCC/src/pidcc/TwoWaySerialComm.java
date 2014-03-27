/*
 * Copyright Luke Wallin 2012
 */
package pidcc;

/**
 *
 * @author Luke
 */
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import gnu.io.*;
import java.util.Enumeration;

public class TwoWaySerialComm {
    
    //protected SerialWriter outWriter;
    protected InputStream in;
    protected OutputStream out;
    
    void connect(String portName) throws Exception{
        connect(portName, 19200);
    }
    void connect(String portName, int baud) throws Exception {
        CommPortIdentifier portIdentifier = CommPortIdentifier
                .getPortIdentifier(portName);
        if (portIdentifier.isCurrentlyOwned()) {
            System.out.println("Error: Port is currently in use");
        } else {
            int timeout = 2000;
            CommPort commPort = portIdentifier.open(this.getClass().getName(), timeout);

            if (commPort instanceof SerialPort) {
                SerialPort serialPort = (SerialPort) commPort;
                serialPort.setSerialPortParams(baud,//57600,
                        SerialPort.DATABITS_8,
                        SerialPort.STOPBITS_1,
                        SerialPort.PARITY_NONE);

                in = serialPort.getInputStream();
                out = serialPort.getOutputStream();
                
                //this.outWriter = new SerialWriter(out);
                
                (new Thread(new SerialReader(in,this))).start();
                //(new Thread(this.outWriter)).start();

            } else {
                System.out.println("Error: Only serial ports are handled by this example.");
            }
        }
    }

    public void dataIn(byte[] buffer, int len){
        System.out.print(new String(buffer, 0, len));
    }
    
    public static class SerialReader implements Runnable {

        InputStream in;
        TwoWaySerialComm comm;

        public SerialReader(InputStream in,TwoWaySerialComm comm) {
            this.in = in;
            this.comm=comm;
        }

        public void run() {
            byte[] buffer = new byte[1024];
            int len = -1;
            try {
                while ((len = this.in.read(buffer)) > -1) {
                    comm.dataIn(buffer,len);
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public static class SerialWriter implements Runnable {

        OutputStream out;

        public SerialWriter(OutputStream out) {
            this.out = out;
        }

        public void run() {
            try {
                int c = 0;
                while ((c = System.in.read()) > -1) {
                    this.out.write(c);
                }
            } catch (IOException e) {
                e.printStackTrace();
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
            
            
            (new TwoWaySerialComm()).connect("/dev/ttyS80");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
