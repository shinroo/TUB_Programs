/**
 * Represents a Cuboid
 * http://en.wikipedia.org/wiki/Cuboid
 * 
 * @author AlgoDat
 *
 */
public class Cuboid extends Body{

	double height, length, width;
	
	/*
	 * Constructor for a Cuboid object
	 */
	public Cuboid() {
		//TODO
		this.height = 0;
		this.length = 0;
		this.width = 0;
	}
	/*
	 * Constructor for a Cuboid object
	 * 
	 * @param h height
	 * @param l length
	 * @param w width
	 */
	public Cuboid(double h, double l, double w) {
		//TODO
		this.height = h;
		this.length = l;
		this.width = w;
	}

	public double getHeight(){
		return this.height;
	}
	public double getLength(){
		return this.length;
	}
	public double getWidth(){
		return this.width;
	}
	//TODO: ggf. weitere Methoden und member implementieren
	@Override
	double calculateVolume() {
		// TODO Auto-generated method stub
		return this.width * this.height * this.length;
	}
	@Override
	double calculateSurface() {
		// TODO Auto-generated method stub
		return (2 * (this.width * this.height)) + (2 * (this.height * this.length)) + (2 * (this.width * this.length));
	}


}

