package pidcc;

/**
 * Class to represent and store information about a point motor
 *
 * @author Luke
 */
public class Turnout {

    public final int id;
    private boolean thrown;
    //usually these are DCC addresses, I'm going to use them as byte and bit in my shift register output
    public final int address;
    public final int subaddress;

    public Turnout(int id, int address, int subaddress) {
        this.id = id;
        this.address = address;
        this.subaddress = subaddress;
    }
    
    public void setThrown(boolean thrown){
        this.thrown = thrown;
    }
    
    public boolean getThrown(){
        return this.thrown;
    }
}