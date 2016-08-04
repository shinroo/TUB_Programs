

/**
 * A Hexagon in 2d space.
 * @author AlgoDat team
 *
 */
public class HexagonShape extends Shape {

	/**
	 * The Hexagon radius.
	 */
	private double radius = 1;

	/**
	 * Creates a new Hexagon
	 * @param r the Hexagon radius
	 */
	HexagonShape(double r) {
		radius = r;
	}

	@Override
	double calculateArea() {
		return 1.5 * Math.sqrt(3) * radius * radius;
	}

	@Override
	void scale(double factor) {
		radius *= factor;
	}
	
	@Override
	public String toString() {
		return "Hexagon | r:"+radius + " at "
				+ super.toString();
	}

	@Override
	public boolean equals(Object o) {
		if (o != null && o instanceof HexagonShape) {
			HexagonShape p = (HexagonShape) o;
			return p.radius == radius;
		}
		return false;
	}
}

