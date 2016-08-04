/**
* This Class represents an Color in RGB space.
* Each colour is represented by three values from [0,255] for the values of the red, green, and blue channels
* @author AlgoDat
*
*/
public class RGBColor {

    private int red;
    private int green;
    private int blue;
        
    /**
     * creates an RGB-Color
     * @param r red-value
     * @param g green-value
     * @param b blue-value
     */
    public RGBColor(int r, int g, int b){
        this.red=r;
        this.green=g;
        this.blue=b;
    }

    public RGBColor(RGBColor rgb){
        this.red=rgb.red;
        this.green=rgb.green;
        this.blue=rgb.blue;
    }
    
    /**
     * sets the colors
     * @param r red-value
     * @param g green-value
     * @param b blue-value
     */
    public void setRGB(int r, int g, int b){
        this.red= r;
        this.green= g;
        this.blue = b;
    }
    
    /**
     * check if this object is White
     * @return true if white, else false
     */
    public boolean isWhite(){
        return (red==255 && green==255 && blue==255);
    }
        

    
//Getters
    
    /**
     * @return red-value
     */
    public int getRed(){
        return this.red;
    }
    /**
     * @return green-value
     */
    public int getGreen(){
        return this.green;
    }
    
    /**
     * @return blue-value
     */
    public int getBlue(){
        return this.blue;
    }
}   

