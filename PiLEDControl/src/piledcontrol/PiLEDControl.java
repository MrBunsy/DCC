package piledcontrol;

import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Intended to send data over SPI to fill up a shift register which is actually
 * a series of linked up LED drivers.
 *
 * @author Luke
 */
public class PiLEDControl {

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {

        LED[] threeWayLeds = {new LED(0, 2, LED.LedColour.GREEN), new LED(0, 3, LED.LedColour.YELLOW), new LED(0, 4, LED.LedColour.RED)};
        Signal threeWay = new Signal(threeWayLeds);

        LED[] twoWay1Leds = {new LED(0, 0, LED.LedColour.RED), new LED(0, 6, LED.LedColour.GREEN)};
        Signal twoWay1 = new Signal(twoWay1Leds);

        LED[] twoWay2Leds = {new LED(0, 1, LED.LedColour.RED), new LED(0, 5, LED.LedColour.GREEN)};
        Signal twoWay2 = new Signal(twoWay2Leds);

        Signal[] signals = {threeWay, twoWay1, twoWay2};

        LedDriver driver = new LedDriver(1, signals);

        driver.update();

        int i = 0;

        while (true) {

            i++;

            switch (i % 3) {
                case 0:
                    threeWay.setSignal(LED.LedColour.RED);
                    break;
                case 1:
                    threeWay.setSignal(LED.LedColour.YELLOW);
                    break;
                case 2:
                    threeWay.setSignal(LED.LedColour.GREEN);
                    break;
            }

            switch (i % 2) {
                case 0:
                    twoWay1.setSignal(LED.LedColour.RED);
                    break;
                case 1:
                    twoWay1.setSignal(LED.LedColour.GREEN);
                    break;
            }

            switch (i % 5) {
                case 2:
                    twoWay2.setSignal(LED.LedColour.RED);
                    break;
                case 4:
                    twoWay2.setSignal(LED.LedColour.GREEN);
                    break;
            }
            driver.update();
            try {
                Thread.sleep(1500);
//
//                threeWay.setSignal(LED.LedColour.RED);
//                driver.update();
//                Thread.sleep(5000);
//
//                threeWay.setSignal(LED.LedColour.YELLOW);
//                driver.update();
//                Thread.sleep(5000);
//
//                threeWay.setSignal(LED.LedColour.GREEN);
//                driver.update();
            } catch (InterruptedException ex) {
                Logger.getLogger(PiLEDControl.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }

}
