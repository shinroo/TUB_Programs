/**
 * @author Marcin
 *
 */
public class Tetrahedron extends Body {
    
	double length;
	
	public Tetrahedron() {
		this.length = 0;
	}
	
	public Tetrahedron(double length) {
		this.length = length;
	}
	
	@Override
	double calculateVolume() {
		return Math.pow(this.length, 3) / (6 * Math.sqrt(2));
	}
	
	@Override
	double calculateSurface() {
		return Math.sqrt(3) * Math.pow(this.length, 2);
	}
	
	
}

