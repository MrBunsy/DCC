package pidcc;

import java.util.ArrayList;

/**
 * Class to hold state so it can be serialised and un-serialised from a file
 * @author Luke
 */
public class StoredState {
    public ArrayList<Turnout> turnouts;
    public int shiftRegisterLength;
}
