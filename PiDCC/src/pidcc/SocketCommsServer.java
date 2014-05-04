/*
 * Copyright Luke Wallin 2012
 */
package pidcc;

import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Luke
 */
public class SocketCommsServer {

    private OutputStream out;
    private InputStream in;
    private Socket socket;
    private TwoWaySerialComm serialComms;
    private boolean running;

    public SocketCommsServer(Socket socket, TwoWaySerialComm serialComms) {
        this.socket = socket;
        this.serialComms = serialComms;
        System.out.println(this.socket.getInetAddress().toString() + " connected");

        try {

            out = this.socket.getOutputStream();
            in = this.socket.getInputStream();

        } catch (IOException e) {
            System.err.println("Couldn't get I/O for the connection: " + e.getMessage());
        }
    }

    public void stop() {
        running = false;
    }

    public void run() {
        running = true;
        LinkStreams socketToSerial = new LinkStreams(in, serialComms.getOutputStream(), this);
        LinkStreams serialToSocket = new LinkStreams(serialComms.getInputStream(), out, this);

        (new Thread(socketToSerial)).start();
        (new Thread(serialToSocket)).start();

        while (running){
            try {
                Thread.sleep(10);
            } catch (InterruptedException ex) {
                Logger.getLogger(SocketCommsServer.class.getName()).log(Level.SEVERE, null, ex);
            }
        }

        socketToSerial.stop();
        serialToSocket.stop();
        System.out.println("Stopping Serial to Socket Link");
    }

    public class LinkStreams implements Runnable {

        private InputStream streamIn;
        private OutputStream streamOut;
        private SocketCommsServer server;
        private boolean running;

        public LinkStreams(InputStream streamIn, OutputStream streamOut, SocketCommsServer server) {
            this.streamIn = streamIn;
            this.streamOut = streamOut;
            this.server = server;
        }

        public void stop() {
            running = false;
        }

        @Override
        public void run() {
            running = true;
            try {
                byte[] buffer = new byte[1024]; // Adjust if you want
                int bytesRead;
                while ((bytesRead = streamIn.read(buffer)) != -1 && running) {
                    streamOut.write(buffer, 0, bytesRead);
                    streamOut.flush();
                }
            } catch (IOException ex) {
                System.out.println(ex.getMessage());
            }
            this.server.stop();
        }

    }

}
