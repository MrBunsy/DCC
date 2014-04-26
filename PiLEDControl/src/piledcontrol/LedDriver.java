package piledcontrol;

import com.pi4j.wiringpi.Gpio;
import java.util.ArrayList;
import java.util.Arrays;

import com.pi4j.wiringpi.Spi;
import java.util.logging.Level;
import java.util.logging.Logger;
/**
 *
 * @author Luke
 */
public class LedDriver {
    
    private static int LE_PIN = 6;
    
    private ArrayList<Signal> signals;
    //how many bytes long is the shift register?
    private int numDrivers;
    
    public LedDriver(int numDrivers, ArrayList<Signal> signals) {
        this.signals = signals;
        this.numDrivers=numDrivers;
        setupSPI();
    }
    
    public LedDriver(int numDrivers, Signal[] signals) {
        this(numDrivers,new ArrayList<Signal>(Arrays.asList(signals)));
    }

    private void setupSPI(){
        if(com.pi4j.wiringpi.Gpio.wiringPiSetup() < 0){
            System.err.println("failed to setup wiringpi");
        }
        
        if (Gpio.wiringPiSetup() == -1) {
            System.out.println(" ==>> GPIO SETUP FAILED");
            return;
        }
        
        Spi.wiringPiSPISetup(Spi.CHANNEL_0,1000000);
        
        
        
        Gpio.pinMode(LE_PIN, Gpio.OUTPUT);
    }
    
    /**
     * Push current configuration out to the hardware
     */
    public void update(){
        byte[] data = new byte[numDrivers];
        
        
        
        //build up the data to send over SPI
        for(Signal s : this.signals){
            s.setSignalsArray(data);
        }
        
        //send it!
//        System.out.println("bytes = "+data.length+ " byte1 = "+data[0]);
        Spi.wiringPiSPIDataRW(Spi.CHANNEL_0,data,data.length);
//        try {
//            Thread.sleep(10);
//        } catch (InterruptedException ex) {
//            Logger.getLogger(LedDriver.class.getName()).log(Level.SEVERE, null, ex);
//        }
        Gpio.digitalWrite(LE_PIN, true);
        Gpio.digitalWrite(LE_PIN, false);
        
//        System.out.println("received byte1 = "+data[0]);
    }
}
