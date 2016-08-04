package groundskeeper;

public class Ball {

	// =======================
	// members 
	// =======================

	protected String type;  // protected: subclass also has access
	protected String color;
	protected double diameter;

	// =======================
	// constructors
	// =======================
	
	public Ball ( String type, String color, double diameter ){
		this.type = type;
		this.color = color;
		this.diameter = diameter;
	}
		
	// this is how the default constructor looks
	// this would be an example of OVERLOADING
	public Ball (){
		// I could put some code here
	}
	// =======================
	// methods
	// =======================
	
	public void performMaintenance(){
		System.out.println("Performing Maintenance in superclass Ball. "
				+ "Something is wrong with polymorphism.");
	}

	// set/get methods
	public void setColor ( String color ){
		this.color = color;
	}
		
	public String  getColor (){
		return this.color;
	}
		
	// we do not allow to set the diameter or type - this can only 
	// be done at the time of creation of the object through the constructor
	public double getDiameter (){
		return diameter;
		// Question: why don't we need to write this.diameter? 
	}
		
	public String getType (){
		return type;
	}

}
