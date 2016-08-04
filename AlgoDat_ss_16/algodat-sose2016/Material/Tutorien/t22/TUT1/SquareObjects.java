/**
 * 
 */

/**
 * @author Marcin
 *
 */
public class SquareObjects {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		Square s1, s2;
		s1 = new Square(8);
		s2 = new Square(15);
		
		System.out.println("Square 1 = " + s1.sideLength + " Square 2 = " + s2.sideLength);
		
		s2.scaleBy(0.5);
		
		System.out.println("Square 1 = " + s1.sideLength + " Square 2 = " + s2.sideLength);
		
	}

}
