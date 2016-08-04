/**
 * Any 2d Entity that fills some area.
 * @author AlgoDat team
 *
 */
public abstract class Shape implements Comparable<Shape> {
    
	/**
	 * if difference below tolerance parameter, shapes are considered equal
	 */
	private static double tol = .1;

    /**
     * Calculates the area of a shape.
     * @return the area the shape fills
     */
    abstract double calculateArea();
    
    /**
     * Scales the shape by a factor.
     * @param factor the scaling factor
     */
    abstract void scale(double factor);
    
    /**
     * This compare to function compares the surface 
     * area of two shape objects ("this" with Shape s)
     * and returns
     *     1 when the Shape s is larger than this
     *     0 when the size is the same (with tolerance considered)
     *     -1 when this is larger than the Shape s
     */
    @Override
    public int compareTo(Shape s) {
    		double myArea = this.calculateArea();
    		double otherArea = s.calculateArea();
    		
    		if (otherArea < myArea - tol) {
    			return -1;
    		} else if (otherArea > myArea + tol) {
    			return 1;
    		} else {
    			return 0;
    		}
    }
    

    public static void main(String[] args) {
    	
		RectangleShape rec = new RectangleShape(5, 6);
		RectangleShape rec2 = new RectangleShape(6, 5);
		
		System.out.println("rec compared to rec2: " + rec.compareTo(rec2));
		
		CircleShape circle = new CircleShape(3.5);
		
		System.out.println("rec compared to cirlce: " + rec.compareTo(circle));
		
		//We create an instance of fenster, a class that implements Comparable<Shape>
		Fenster f = new Fenster();
		System.out.println(f.compareTo(circle));
		//Even though the class Fenster is not of type Shape, we can compare Shapes with it
		
		System.out.println("fenster compared to circle");
		
	}
    
    
}

