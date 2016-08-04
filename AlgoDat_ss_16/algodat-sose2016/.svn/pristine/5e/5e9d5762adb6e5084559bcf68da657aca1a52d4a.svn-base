/**
 * Represents a Cube
 * @author AlgoDat
 */
public class Cube extends Cuboid{

	/*
	 * Constructor without parameter
	 */
	public Cube() {
		//TODO
		this.length = 0;
		this.width = 0;
		this.height = 0;
	}
	/*
	 * Constructor with one parameter
	 * 
	 * @param length the side length of the cube
	 */
	public Cube(double length) {
		// TODO
		this.length = length;
		this.width = length;
		this.height = length;
	}

	public double getLength(double length){
		return this.length;
	}
	//TODO: ggf. weitere Methoden und member implementieren
	@Override
	double calculateVolume() {
		// TODO Auto-generated method stub
		return this.length * this.width * this.height;
	}
	@Override
	double calculateSurface() {
		// TODO Auto-generated method stub
		return (2 * (this.width * this.height)) + (2 * (this.height * this.length)) + (2 * (this.width * this.length));
	}
}

