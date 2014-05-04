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
     * @param uartPort serial port to connect to, eg COM1
     * @param demoMode Run a simple backwards and forwards demo?
     * @param demoAddress Which decoder to tell to go backwards and forwards
     * @param server Act as a server to forward commands to the serial link from
     * elsewhere on the network
     * @param port Port to listen on for acting as server
     * @param client Connect to a server rather than a serial link
     * @param host Who to connect to if acting as a client
     * @param gui Run a GUI to control trains?
     */
    public PiDCC(String uartPort, boolean demoMode, int demoAddress, boolean server, int port, boolean client, String host, boolean gui) {
        if (!client) {
            //not connecting to the server, so must connect to serial port
            serialComms = new TwoWaySerialComm();
            try {
                System.out.println("Connecting to serial port: "+uartPort);
                serialComms.connect(uartPort);
                comms = serialComms;
            } catch (Exception ex) {
                System.err.println("Failed to connect to " + uartPort+": "+ex.getMessage());
                System.exit(1);
            }

            if (server) {
                //we're a server, so listen for connections and deal with them
                Listener(port, serialComms);
            }

        } else {
            //client!
            socketComms = new SocketCommsClient();
            System.out.println("Connecting to "+host+":"+port);
            socketComms.connectTo(host, port);
            
            comms = socketComms;
        }
        
        if(demoMode){
            System.out.println("Running a demo for decoder address: "+demoAddress);
            runDemo(demoAddress);
        }

    }

    private void runDemo(int address) {

        Train train = new Train(address);
        LightsMessage lights = new LightsMessage(train, true);
        while (true) {
            try {
                Thread.sleep(3000);
                comms.sendMessage(lights);
                
                SpeedMessage s = new SpeedMessage(train, true, 120);
                comms.sendMessage(s);

                Thread.sleep(1500);
                s = new SpeedMessage(train, true, 0);
                comms.sendMessage(s);

                Thread.sleep(3000);
                s = new SpeedMessage(train, false, 80);
                comms.sendMessage(s);

                Thread.sleep(1500);
                s = new SpeedMessage(train, false, 0);
                comms.sendMessage(s);

            } catch (InterruptedException ex) {
                Logger.getLogger(PiDCC.class.getName()).log(Level.SEVERE, null, ex);
            }

        }
    }

    public static void Listener(int port, TwoWaySerialComm serialComms) {

        try {
            ServerSocket listener = new ServerSocket(port, 1);
            while (true) {
                //this blocks until a connection is received
                System.out.println("listening on port " + port);
                Socket socket = listener.accept();
                SocketCommsServer server = new SocketCommsServer(socket, serialComms);
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
                + "-d --demo Run in demo mode\n"
                + "-a --address Address to command for demo mode (default 3)\n"
                + "-s --server Act as a server to forward commands from elsewhere on the network\n"
                + "-p --port Port for server to listen on (default 1234)\n"
                + "-c --client Act as a client, connect to a server rather than UART\n"
                + "-h --host IP/hostname to connect to\n"
                + "-g --gui Run a GUI to control the trains\n");
    }

    public static void main(String[] args) {

        CmdLineParser parser = new CmdLineParser();

        Option serialOption = parser.addIntegerOption('s', "serial");
        Option demoModeOption = parser.addBooleanOption('d', "demo");
        Option addressOption = parser.addIntegerOption('a', "address");
        Option serverOption = parser.addBooleanOption('s', "server");
        Option portOption = parser.addIntegerOption('p', "port");
        Option clientOption = parser.addBooleanOption('c', "client");
        Option hostOption = parser.addStringOption('h', "host");
        Option guiOption = parser.addBooleanOption('g', "gui");

        try {
            parser.parse(args);
        } catch (CmdLineParser.OptionException e) {
            System.err.println(e.getMessage());
            printUsage();
        }

        String serial = (String) parser.getOptionValue(serialOption, "/dev/ttyS80");
        boolean demoMode = (Boolean) parser.getOptionValue(demoModeOption, false);
        int address = (Integer) parser.getOptionValue(addressOption, 3);
        boolean server = (Boolean) parser.getOptionValue(serverOption, false);
        int port = (Integer) parser.getOptionValue(portOption, 1234);
        boolean client = (Boolean) parser.getOptionValue(clientOption, false);
        String host = (String) parser.getOptionValue(hostOption, "192.168.1.21");
        boolean gui = (Boolean) parser.getOptionValue(guiOption, false);

        PiDCC p = new PiDCC(serial, demoMode, address, server, port, client, host, gui);

    }
}
