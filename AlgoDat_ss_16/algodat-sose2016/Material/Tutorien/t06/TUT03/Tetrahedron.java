public class Tetrahedron extends Body {
    // TODO: Vererbungshierarchie und Implementation
	
	double length;
	
	/**
	 * Constructs a Tetrahedron object by initializing its length with 0
	 */
	public Tetrahedron() {
		this.length = 0;
	}
	
	/**
	 * Constructs a Tetrahedron object by initializing its length with the given value
	 * @param length	The initial value for length
	 */
	public Tetrahedron(double length) {
		this.length = length;
	}
	
	/**
	 * Calculates the volume of the Tetrahedron
	 * @see Body#calculateVolume()
	 */
	@Override
	double calculateVolume() {
		return (Math.pow(this.length, 3)/12)*Math.sqrt(2);
	}
	
	/**
	 * Calculates the surface of the Tetrahedron
	 * @see Body#calculateSurface()
	 */
	@Override
	double calculateSurface() {
		return Math.pow(this.length, 2)*Math.sqrt(3);
	}
}

