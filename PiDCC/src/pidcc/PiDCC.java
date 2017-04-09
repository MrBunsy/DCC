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
import java.io.File;
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
    private int port;
    private boolean pipeThrough;
    TwoWaySerialComm serialComms;
    SocketCommsClient socketComms;
//    File settingsFile;
    private String settingsFilePath;
    private int shiftRegisterLength;

    /**
     *
     * @param uartPort serial port to connect to, eg COM1 elsewhere on the
     * network
     * @param port Port to listen on for acting as server
     * @param pipeThrough Just pipe everything received on the socket through to
     * the UART
     * @param settingsFilePath name of json file of saved shift register config
     * @param shiftRegisterLength length of the shift register used for signals and points
     */
    public PiDCC(String uartPort, int port, boolean pipeThrough, String settingsFilePath, int shiftRegisterLength) {
        //not connecting to the server, so must connect to serial port
        serialComms = new TwoWaySerialComm();
        try {
            System.out.println("Connecting to serial port: " + uartPort);
            serialComms.connect(uartPort, 19200);
            comms = serialComms;
        } catch (Exception ex) {
            System.err.println("Failed to connect to " + uartPort + ": " + ex.getMessage());
            System.exit(1);
        }

//        settingsFile = new File(settingsFilePath);
        this.settingsFilePath = settingsFilePath;
        this.pipeThrough = pipeThrough;
        this.shiftRegisterLength = shiftRegisterLength;
        this.port=port;

        //we're a server, so listen for connections and deal with them
        Listener();

    }

    private void Listener() {

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
                    server = new DccppServer(socket, serialComms, settingsFilePath, shiftRegisterLength);
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
                + "-t --through Just pipe everything received on the socket through to the UART\n"
                + "-f --file Settings file for retreiving and storing turnouts (default settings.json)\n"
                + "-r --register Length of shift register controlling points and lights (default 1)");
    }

    public static void main(String[] args) {

        CmdLineParser parser = new CmdLineParser();

        Option serialOption = parser.addStringOption('s', "serial");
        Option portOption = parser.addIntegerOption('p', "port");
        Option registerOption = parser.addIntegerOption('r', "register");
        Option pipethroughOption = parser.addBooleanOption('t', "through");
        Option fileOption = parser.addStringOption('f', "file");

        try {
            parser.parse(args);
        } catch (CmdLineParser.OptionException e) {
            System.err.println(e.getMessage());
            printUsage();
        }

        String serial = (String) parser.getOptionValue(serialOption, "/dev/ttyS80");
        String file = (String) parser.getOptionValue(fileOption, "settings.json");
        boolean through = (Boolean) parser.getOptionValue(pipethroughOption, false);
        int port = (Integer) parser.getOptionValue(portOption, 1234);
        int register = (Integer) parser.getOptionValue(registerOption, 1);

        PiDCC p = new PiDCC(serial, port, through, file, register);

    }
}
