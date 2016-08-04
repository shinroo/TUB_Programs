package groundskeeper;

public class SolidBall extends Ball{

	// =======================
	// types
	// =======================

	public enum SurfaceType { HARD, SOFT_PLASTIC, HARD_PLASTIC, LEATHER };
	
	// =======================
	// members 
	// =======================

	private SurfaceType surfaceProperty;
	
	// =======================
	// constructors
	// =======================
	
	public SolidBall ( String type, String color, double diameter, 
			SurfaceType surfaceProperty ){
		super( type, color, diameter );
		this.surfaceProperty = surfaceProperty;
	}
	
	// =======================
	// methods
	// =======================
	
	public SurfaceType getSurfaceProperty (){
		return this.surfaceProperty;
	}
	
	public void performMaintenance () {
		
		int type = 10;
		
		System.out.println(type);
		System.out.println(this.type);
		
		switch ( surfaceProperty ){
			case HARD:	
				System.out.print ("Used metal brush on ");
				break;
			case SOFT_PLASTIC:
			    System.out.print ("Used wet cloth on ");
			    break;
			case HARD_PLASTIC: 
			    System.out.print ("Used moist cloth on ");
			    break;
			case LEATHER: 
			    System.out.print("Used shoe polish on ");
			    break;
		    default:
			    System.out.print("Did not know what to do with ");
		}
			
		System.out.println ( this.color + " " + this.type + ".");
	}

}

