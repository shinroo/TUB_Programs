
/**
* this class <code> Picture </code> describes a RGB picture with dimensions width x height.
* Each pixels coulour is represented by an RGBColour
* 
* @author AlgoDat
*
*/

import javax.imageio.*;
import java.io.IOException;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.*;
import java.io.FileInputStream;
import java.io.InputStream;

public class Picture {

    /**
    * A 2d array containing the colours of the individual pixels
    */
    private RGBColor imageMatrix[][];

    /**
    * The width of the image in pixels
    */
    private int width;

    /**
    * height of the image in pixels
    */
    private int height;
    
    /**
     * intitialize a picture by creating a white picture
     */
    public Picture(int width, int height){
        this.height = height;
        this.width = width;
        createWhitePicture();
    }
    /**
     * initialize a picture by opening given file
     * @param picUrl path of *.bmp picture
     */
    public Picture(String picUrl){      
        openAndSetPicture(picUrl);      
    }

    /**
    * intialize a picture by giving an image matrix
    * @param imageMatrix two dimansionar RGBColor array
    */
    public Picture(RGBColor imageMatrix[][]){
        this.width = imageMatrix.length;
        this.height = imageMatrix[0].length;
        this.imageMatrix = imageMatrix;
    }

    /**
     * turns this picture 90 degrees to the right
     *
     */
    public void rot90DegRight(){
		//TODO
    	Picture newPic = new Picture(this.height,this.width);
    	int maxY = this.width -1;
    	for(int x = 0; x < this.width; x++) {
    		for(int y = 0; y < this.height; y++) {
    			newPic.imageMatrix[maxY - y][x] = this.imageMatrix[x][y];
    		}
    	}
    	this.imageMatrix = newPic.imageMatrix;
    }
    
    /**
     * turns this picture 180 degrees
     *
     */
    public void rot180Deg(){
		//TODO
    	Picture newPic = new Picture(this.height,this.width);
    	int maxY = this.width -1;
    	int maxX = this.height -1;
    	for(int x = 0; x < this.width; x++) {
    		for(int y = 0; y < this.height; y++) {
    			newPic.imageMatrix[maxX - x][maxY - y] = this.imageMatrix[x][y];
    		}
    	}
    	this.imageMatrix = newPic.imageMatrix;
    }
    
    /**
     * finds white pixels and approximates their new color by using the average of neighbour colors
     *
     */
    public void repairPicture(){     
		//TODO
    	for (int lat = 0; lat < width; lat++) {
			for (int vert = 0; vert < height; vert++){
				if (imageMatrix[lat][vert].isWhite()) {
					/*determine colour*/ //screw American spelling
					int red = 0;
					int green = 0;
					int blue = 0;
					int div = 0;
					
					//left
					if (lat != 0) {
						red += imageMatrix[lat-1][vert].getRed();
						green += imageMatrix[lat-1][vert].getGreen();
						blue += imageMatrix[lat-1][vert].getBlue();
						div++;
					}
					
					//right
					if (lat != width) {
						red += imageMatrix[lat+1][vert].getRed();
						green += imageMatrix[lat+1][vert].getGreen();
						blue += imageMatrix[lat+1][vert].getBlue();
						div++;
					}
					
					//up
					if (vert != 0) {
						red += imageMatrix[lat][vert-1].getRed();
						green += imageMatrix[lat][vert-1].getGreen();
						blue += imageMatrix[lat][vert-1].getBlue();
						div++;
					}
					
					//down
					if (vert != height) {
						red += imageMatrix[lat][vert+1].getRed();
						green += imageMatrix[lat][vert+1].getGreen();
						blue += imageMatrix[lat][vert+1].getBlue();
						div++;
					}
					
					//up left
					if ((vert != 0) && (lat != 0)){
						red += imageMatrix[lat-1][vert-1].getRed();
						green += imageMatrix[lat-1][vert-1].getGreen();
						blue += imageMatrix[lat-1][vert-1].getBlue();
						div++;
					}
					
					//down left
					if ((vert != height) && (lat != 0)) {
						red += imageMatrix[lat-1][vert+1].getRed();
						green += imageMatrix[lat-1][vert+1].getGreen();
						blue += imageMatrix[lat-1][vert+1].getBlue();
						div++;
					}
					
					//up right
					if ((vert != 0) && (lat != width)) {
						red += imageMatrix[lat-1][vert+1].getRed();
						green += imageMatrix[lat-1][vert+1].getGreen();
						blue += imageMatrix[lat-1][vert+1].getBlue();
						div++;
					}
					
					//down right
					if ((vert != height) && (lat != width)) {
						red += imageMatrix[lat-1][vert+1].getRed();
						green += imageMatrix[lat-1][vert+1].getGreen();
						blue += imageMatrix[lat-1][vert+1].getBlue();
						div++;
					}
					
					imageMatrix[lat][vert].setRGB(red/div, green/div, blue/div); 
				}
			}
		}
    }
        
    /**
     * Creates a completely white picture
     *
     */
    public void createWhitePicture(){
        this.imageMatrix = new RGBColor[this.width][this.height];
        for (int w=0; w< this.width; w++){
            for(int h=0; h< this.height; h++){
                //set this colors in picture
                this.imageMatrix[w][h] = new RGBColor(255, 255, 255);                
            }
        }
    }

    /**
    * DO NOT CHANGE ANYTHING BELOW THIS LINE!
    **/

    // Getters
    
    /**
     * 
     * @return the width of the picture
     */
    public int getWidth(){
        return this.width;
    }

    /**
     * 
     * @return the height of the picture
     */
    public int getHeight(){
        return this.height;
    }

    /**
     * 
     * @return the the picture matrix
     */
    public RGBColor[][] getImageMatrix(){
        return this.imageMatrix;
    }
    
    /**
     * reads an 24-bit(8,8,8) Bitmap and store it into picture-array
     * @param picUrl The url to the pic
     * @return true, if successful else false
     */
    private boolean openAndSetPicture(String picUrl){
         
         BufferedImage pic;
         
         
         try {
             InputStream iS= new FileInputStream(picUrl);
             // get buffer of the picture
             pic = ImageIO.read(iS);    
             
             // get additional picture informations
             this.height = pic.getHeight();
             this.width = pic.getWidth();            
             
             // store rgb colors in picture
             this.imageMatrix = new RGBColor[this.width][this.height];
             ColorModel cm= ColorModel.getRGBdefault();
             for (int w=0; w< this.width; w++){
                 for(int h=0; h< this.height; h++){
                     
                     // read out every RGBcolor
                     int pixel = pic.getRGB(w, h);
                     int rVal= cm.getRed(pixel);
                     int gVal= cm.getGreen(pixel);
                     int bVal= cm.getBlue(pixel);
                     
                     //set this colors in picture
                     this.imageMatrix[w][h] = new RGBColor(rVal, gVal, bVal);                
                 }
             }
             return true;
             
             
         }catch (IOException e) {
             e.printStackTrace();       
         }
         return false;
         
    }
    
    public BufferedImage getImage(){
        BufferedImage buf = new BufferedImage(this.width, this.height, BufferedImage.TYPE_INT_RGB);
        for(int w=0; w<this.width; w++){
            for(int h=0; h<this.height; h++){
                int red= this.imageMatrix[w][h].getRed();
                int blue= this.imageMatrix[w][h].getBlue();
                int green= this.imageMatrix[w][h].getGreen();
                int rgbVal= new Color(red, green, blue).getRGB();
                buf.setRGB(w, h, rgbVal);
            }
    }
    
        return buf;
    }


    
    
}

