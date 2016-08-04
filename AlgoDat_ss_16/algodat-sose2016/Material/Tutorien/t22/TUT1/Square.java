/**
 * 
 */

/**
 * Representiert ein Quadrat
 * 
 * @author Marcin
 *
 */
public class Square {
	
	/**
	 * The side length of a square
	 */
	double sideLength; 
	
	/**
	 * Construct a square
	 * 
	 * @param sl The side length
	 */
	Square (double sl) {
		this.sideLength = sl;
	}
	
	/**
	 * @param s
	 */
	void scaleBy (double s) {
		this.sideLength = this.sideLength*s;
	}
	
}
