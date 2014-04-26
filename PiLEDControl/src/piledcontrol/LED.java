package piledcontrol;

/**
 * Represents a single LED (output of driver)
 * @author Luke
 */
public class LED {
    
    public enum LedColour{
        RED,
        YELLOW,
        GREEN
    }
    //position in shift register of LED drivers
    private int byteNum, bitNum;
    private LedColour colour;
    private boolean lit;
    
    /**
     * Using LED like a struct (but using getters and setters in case I change my mind)
     * @param byteNum
     * @param bitNum
     * @param colour 
     */
    public LED(int byteNum, int bitNum, LedColour colour){
        this.byteNum=byteNum;
        this.bitNum=bitNum;
        this.colour=colour;
        this.lit=false;
    }
    
    public void setLit(boolean lit){
        this.lit=lit;
    }
    
    public boolean isLit(){
        return this.lit;
    }
    
    public int getByteNum(){
        return this.byteNum;
    }
    
    public int getBitNum(){
        return this.bitNum;
    }
    
    public byte getByte(){
        return (byte)(1 << this.bitNum);
    }
    
    public LedColour getColour(){
        return this.colour;
    }
}
