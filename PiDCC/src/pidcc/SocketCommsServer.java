/*
 * Copyright Luke Wallin 2012
 */
package pidcc;

import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Luke
 */
public class SocketCommsServer {

    protected OutputStream out;
    protected InputStream in;
    protected Socket socket;
    protected TwoWaySerialComm serialComms;
    protected boolean running;

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

        while (running) {
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
        private boolean printAscii;

        public LinkStreams(InputStream streamIn, OutputStream streamOut, SocketCommsServer server, boolean printAscii) {
            this.streamIn = streamIn;
            this.streamOut = streamOut;
            this.server = server;
            this.printAscii = printAscii;
        }

        public LinkStreams(InputStream streamIn, OutputStream streamOut, SocketCommsServer server) {
            this(streamIn, streamOut, server, false);
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
                    if (this.printAscii) {
                        byte[] slice = Arrays.copyOfRange(buffer, 0, bytesRead);
                        System.out.print(new String(slice));
                    }
                }
            } catch (IOException ex) {
                System.out.println(ex.getMessage());
            }
            this.server.stop();
        }

    }

}
