/**
 * Regular tetrahedron
 */
public class Tetrahedron extends Body {

	/**
	 * side length of tetrahedron
	 */
	private double sideLength;

	/**
	 * Create a tetrahedron
	 * 
	 * @param sideLength
	 *            the side length
	 */
	public Tetrahedron(double sideLength) {
		this.sideLength = sideLength;
	}

	@Override
	public double calculateVolume() {
		return Math.pow(sideLength, 3) / (6 * Math.sqrt(2));
	}

	@Override
	public double calculateSurface() {
		return Math.pow(sideLength, 2) * Math.sqrt(3);
	}

}
