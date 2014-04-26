package piledcontrol;

import java.util.ArrayList;

/**
 * A signal holds several LEDs
 *
 * @author Luke
 */
public class Signal {

    private LED[] leds;

    public Signal(LED[] leds) {
        this.leds = leds;
    }

    public Signal(ArrayList<LED> leds) {
        this.leds = leds.toArray(new LED[1]);
    }

    /**
     * Given a reference to an array, fill the right bits to set all the signals
     * that are currently on
     *
     * @param signalArray
     */
    public void setSignalsArray(byte[] signalArray) {
        for (LED l : this.leds) {
            if (l.isLit()) {
                signalArray[l.getByteNum()] |= l.getByte();
            }
        }
    }

    /**
     * Set a specific LED on/off, based on where it is in our LED array
     * @param index location in leds array
     * @param lit lit or not?
     */
    public void setSignal(int index, boolean lit) {
        if (index < this.leds.length) {
            this.leds[index].setLit(lit);
        }
    }
    
    /**
     * Set all LEDs of this colour ON, and the rest OFF
     * @param colour 
     */
    public void setSignal(LED.LedColour colour){
        for (LED l : this.leds) {
            if(l.getColour()==colour){
                l.setLit(true);
            }else{
                l.setLit(false);
            }
        }
    }

}
