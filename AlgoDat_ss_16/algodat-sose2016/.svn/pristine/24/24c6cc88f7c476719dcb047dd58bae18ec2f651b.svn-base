package groundskeeper;

import java.lang.Math;

public class InflatableBall extends Ball{

	// =======================
	// types
	// =======================
	
	public enum ValveType { PRESTA, SHRADER, NEEDLE, PLUG };
		
	// =======================
	// members 
	// =======================

	private double desiredAirPressure;  // in bar
	private double currentAirPressure;  // in bar
	private ValveType valveType;        
	
	// =======================
	// constructors
	// =======================
	
	public InflatableBall ( String type, String color, double diameter, double airPressure, 
			ValveType valveType ){
		super ( type, color, diameter );
		this.desiredAirPressure = this.currentAirPressure = airPressure;
		this.valveType = valveType;
	}
	
	public InflatableBall ( String type, String color, double diameter, ValveType valveType ){
		super ( type, color, diameter );
		this.desiredAirPressure = this.currentAirPressure = 0.0;
		this.valveType = valveType;
	}
	
	// =======================
	// methods
	// =======================
	
	public double getCurrentAirPressure (){
		return currentAirPressure;
	}
	
	public double getDesiredAirPressure (){
		return desiredAirPressure;
	}
	
	// the valve type can only be set in the constructor
	public String getValveType (){
		return valveType.toString();
	}
	
	public void performMaintenance (){
		double absoluteAirPressureDeviation = 
				Math.abs ( currentAirPressure - desiredAirPressure );
		if ( absoluteAirPressureDeviation > 0.2 ){
			setAirPressure ( desiredAirPressure );  // Klaus Hornig is taking care of things!
			System.out.println ("Inflated " + color + " " + type + " with diameter " + 
			   diameter + " to " + desiredAirPressure + ".");
		}
		System.out.println ("Everything okay with "+ color + " " + type + " with diameter " + 
				diameter + ".");
	}	

	// private methods
	private void setAirPressure ( double airPressure ){
		if ( airPressure < 1000.0 )
		  this.currentAirPressure = airPressure;
		else
	      this.currentAirPressure = 0.0;	
	}
	
}
