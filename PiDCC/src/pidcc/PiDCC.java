/*
 * Copyright Luke Wallin 2012
 */
package pidcc;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import jargs.gnu.CmdLineParser;
import jargs.gnu.CmdLineParser.Option;

import gnu.io.*;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Luke
 */
public class PiDCC {
    //private TwoWaySerialComm serialComms;

    private CommLink comms;
    TwoWaySerialComm serialComms;
    SocketCommsClient socketComms;

    /**
     *
     * @param uartPort serial port to connect to, eg COM1 elsewhere on the
     * network
     * @param port Port to listen on for acting as server
     * @param pipeThrough Just pipe everything received on the socket through to
     * the UART
     */
    public PiDCC(String uartPort, int port, boolean pipeThrough) {
        //not connecting to the server, so must connect to serial port
        serialComms = new TwoWaySerialComm();
        try {
            System.out.println("Connecting to serial port: " + uartPort);
            serialComms.connect(uartPort);
            comms = serialComms;
        } catch (Exception ex) {
            System.err.println("Failed to connect to " + uartPort + ": " + ex.getMessage());
            System.exit(1);
        }

        //we're a server, so listen for connections and deal with them
        Listener(port, serialComms, pipeThrough);

    }

    public static void Listener(int port, TwoWaySerialComm serialComms, boolean pipeThrough) {

        try {
            ServerSocket listener = new ServerSocket(port, 1);
            while (true) {
                //this blocks until a connection is received
                System.out.println("listening on port " + port);
                Socket socket = listener.accept();
                SocketCommsServer server = null;
                if (pipeThrough) {
                     server = new SocketCommsServer(socket, serialComms);
                } else {
                     server = new DccppServer(socket, serialComms);
                }
                //I want this to block so we aren't still listening for another client
                server.run();

                socket.close();
            }
        } catch (IOException ex) {
            System.out.println(ex.getMessage());
        }
    }

    public static void printUsage() {
        System.out.print("Usage: \n\n"
                + "-s --serial Serial port to use, default: /dev/ttyS80\n"
                + "-p --port Port for server to listen on (default 1234)\n"
                + "-t --through Just pipe everything received on the socket through to the UART");
    }

    public static void main(String[] args) {

        CmdLineParser parser = new CmdLineParser();

        Option serialOption = parser.addStringOption('s', "serial");
        Option portOption = parser.addIntegerOption('p', "port");
        Option pipethroughOption = parser.addBooleanOption('t', "through");

        try {
            parser.parse(args);
        } catch (CmdLineParser.OptionException e) {
            System.err.println(e.getMessage());
            printUsage();
        }

        String serial = (String) parser.getOptionValue(serialOption, "/dev/ttyS80");
        boolean through = (Boolean) parser.getOptionValue(pipethroughOption, false);
        int port = (Integer) parser.getOptionValue(portOption, 1234);

        PiDCC p = new PiDCC(serial, port, through);

    }
}
