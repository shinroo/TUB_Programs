public class Tetrahedron extends Body {
    
	private double length;
	
	public Tetrahedron() {
		this.length = 0;
	}
	
	public Tetrahedron(double length) {
		this.length = length;
	}
	
	@Override
	double calculateVolume(){
		return Math.pow(this.length, 3) / (6 * Math.sqrt(2));
	}
	
	@Override
	double calculateSurface(){
		return Math.sqrt(3) * Math.pow(this.length, 2);
	}
	
	@Override
	void set(double length) {
		this.length = length;
	}
	
	@Override
	double get() {
		return this.length;
	}
	
}

