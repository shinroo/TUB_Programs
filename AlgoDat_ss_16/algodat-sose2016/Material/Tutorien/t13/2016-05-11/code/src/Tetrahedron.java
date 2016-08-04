
public class Tetrahedron extends Body {

	private double sideLength;

	public Tetrahedron(double sideLength) {
		this.sideLength = sideLength;
	}
	
	@Override
	public double calculateVolume() {
		return Math.pow(sideLength, 3) / 12 * Math.sqrt(2);
	}

	@Override
	public double calculateSurface() {
		return Math.pow(sideLength, 2) * Math.sqrt(3);
	}
	
	/**
	 * @deprecated sdsgsg
	 */
	@Deprecated
	public void m() {
		
	}

}
