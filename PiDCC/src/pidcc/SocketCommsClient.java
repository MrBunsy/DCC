/*
 * Copyright Luke Wallin 2012
 */

package pidcc;

import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Luke
 */
public class SocketCommsClient implements CommLink{
    private Socket socket;
    private OutputStream out;
    
//    public SocketCommsClient(){
//        
//    }
    
    public void connectTo(String serverIp, int port) {
        System.out.println("Attempting to connect to "+serverIp+":"+port);
        socket = GetSocket(serverIp, port);
        if (socket != null) {
            try {
                
                out = socket.getOutputStream();
                System.out.println("Connected to "+serverIp+":"+port);
                
                (new Thread(new InputStreamReader(socket.getInputStream(),this))).start();

            } catch (IOException e) {
                System.err.println("Couldn't get I/O for the connection: " + e.getMessage());
            }
        }
    }

    public static Socket GetSocket(String host, int port) {
        InetAddress address = null;
        Socket _socket = null;

        try {
            address = InetAddress.getByName(host);

        } catch (UnknownHostException e) {
            System.out.println("Failed to find " + host + " " + e.getMessage());
        }
        if (address != null) {
            try {
                _socket = new Socket(address, port);
            } catch (IOException e) {
                System.out.println("Failed to connect to " + host + ":" + port + " " + e.getMessage());
            }

        }
        return _socket;
    }
    
    @Override
    public void sendMessage(Message message) {
        try {
            Message.writeBuffer(message.getByteBuffer(), out);
        } catch (IOException ex) {
            Logger.getLogger(TwoWaySerialComm.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    @Override
    public void receivedMessage(byte[] bytes, int len) {
        System.out.print(new String(bytes, 0, len));
    }
    
    
}
