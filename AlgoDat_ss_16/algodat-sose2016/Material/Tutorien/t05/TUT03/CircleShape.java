

/**
 * A Circle in 2d space.
 * @author AlgoDat team
 *
 */
public class CircleShape extends Shape {
	/**
	 * The Circle radius.
	 */
	private double radius = 1;

	/**
	 * Creates a new circle
	 * @param r the circle's radius
	 */
	CircleShape(double r) {
		radius = r;
	}

	@Override
	double calculateArea() {
		return Math.PI * radius * radius;
	}

	@Override
	void scale(double factor) {
		radius *= factor;
	}
	
	@Override
	public String toString() {
		return "Circle | r:"+radius + " at " 
				+ super.toString();
	}

	@Override
	public boolean equals(Object o) {
		if (o != null && o instanceof CircleShape) {
			CircleShape p = (CircleShape) o;
			return p.radius == radius;
		}
		return false;
	}
}

