
/**
 * Any 2d Entity that fills some area.
 * @author AlgoDat team
 *
 */
public abstract class Shape implements Comparable<Shape>  {
    
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
     * Compares two shapes using the area to compare.
     *
     * @param s the other Shape
     */
    @Override
    public int compareTo(Shape s) {    
        double myarea = this.calculateArea();
        double otherarea = s.calculateArea();
        if(otherarea < myarea - tol) return 1;
        else if(otherarea > myarea + tol) return -1;
		return 0;
    }    
}

