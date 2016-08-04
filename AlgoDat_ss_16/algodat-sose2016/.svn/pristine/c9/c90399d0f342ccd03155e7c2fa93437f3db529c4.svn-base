package groundskeeper;

import java.util.Iterator;
import java.util.List;

import groundskeeper.InflatableBall.ValveType;
import groundskeeper.SolidBall.SurfaceType;

import java.util.ArrayList;

public class KlausHornig {

	public static void main(String[] args) {
		
		// create a stray ball (just to show how we can create objects)
		Ball strayBall= new SolidBall ("invisible ball", "transparent", 
												0.0, SurfaceType.LEATHER);
		strayBall.performMaintenance();
		
		if (strayBall instanceof SolidBall)
			System.out.println("-----------Yes!-------------");
		
		System.out.println("This is a useless output: " + strayBall.getType());
		
		Ball impossibleBall = new Ball ("not possible", "no color", 0.0);
		
		// create an ArrayList for objects of type Ball
		List<Ball> allBalls = new ArrayList<Ball>();
		
		// Klaus Hornig finds a lot of different balls
		allBalls.add(new InflatableBall ( "volleyball",   "white",       
												30.0, 1.5, ValveType.NEEDLE ));
		allBalls.add(new InflatableBall ( "basketball",   "orange",      
												38.0, 2.5, ValveType.SHRADER ));
		allBalls.add(new InflatableBall ( "volleyball",   "white",       
												20.0, 1.5, ValveType.NEEDLE ));
		allBalls.add(new InflatableBall ( "basketball",   "orange",      
												38.0, 2.5, ValveType.NEEDLE ));
		allBalls.add(new InflatableBall ( "volleyball",   "white",       
												30.0, 1.5, ValveType.NEEDLE ));
		allBalls.add(new InflatableBall ( "basketball",   "orange",      
												18.0, 2.5, ValveType.PLUG ));
		allBalls.add(new SolidBall      ( "bowling ball", "purple",      
												32.0, SurfaceType.HARD));
		allBalls.add(new SolidBall      ( "medicine ball","brown",       
												45.0, SurfaceType.LEATHER));
		allBalls.add(new SolidBall      ( "soft plastic ball", "rainbow", 
												120.0, SurfaceType.SOFT_PLASTIC));
		
		// Klaus Hornig is getting ready to take care of all of them
		Iterator<Ball> myIterator = allBalls.iterator();
		
		// ... and he is doing it one ball at a time
		while ( myIterator.hasNext() )
			myIterator.next().performMaintenance();

		// Klaus Hornig hat Feierabend
	}
}