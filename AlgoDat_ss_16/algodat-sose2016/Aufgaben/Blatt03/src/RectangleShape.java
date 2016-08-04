
/**
 * A Rectangle with width and height in 2d space.
 * @author AlgoDat team
 *
 */
public class RectangleShape extends Shape {
	/**
	 * the height of the rectangle
	 */
	private double height = 1;
	/**
	 * the width of the rectangle
	 */
	private double width = 1;

	/**
	 * Creates a Rectangle of size w x h.
	 * @param w the width of the rectangle
	 * @param h the height of the rectangle
	 */
	RectangleShape(double w, double h) {
		height = h;
		width = w;
	}

	@Override
	double calculateArea() {
		return height * width;
	}

	@Override
	void scale(double factor) {
		height *= factor;
		width *= factor;
	}
	
	@Override
	public String toString() {
		return "Rectangle | w:"+width+" h:"+height
				+ " at " + super.toString();
	}

	@Override
	public boolean equals(Object o) {
		if (o != null && o instanceof RectangleShape) {
			RectangleShape p = (RectangleShape) o;
			return p.width == width && p.height == height;
		}
		return false;
	}
}

