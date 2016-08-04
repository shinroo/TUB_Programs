public class Tetrahedron extends Body{

	/**
	 * Beschreibt Seitenlänge vom Thera....
	 */
	double length;
	
	/**
	 * Leerer Konstruktor...defieniert fpr Länge 0
	 */
	public Tetrahedron(){
		length=0;
	}
	
	public Tetrahedron(double length){
		this.length = length;
	}
	
	
	/**
	 * 
	 * Berechnet Volumen vom Tetra...
	 * 
	 * (non-Javadoc)
	 * @return Volume of the Tetrahedron
	 * @see Body#calculateVolume()
	 */
	@Override
	double calculateVolume() {
		return Math.pow(this.length, 3) / (6 * Math.sqrt(2));
	}

	@Override
	double calculateSurface() {
		// TODO Auto-generated method stub
		return 0;
	}

  
}

