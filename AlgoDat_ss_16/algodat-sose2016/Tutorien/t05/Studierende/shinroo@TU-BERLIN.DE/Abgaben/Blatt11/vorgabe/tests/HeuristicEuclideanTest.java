import java.io.IOException;
import java.util.List;
import java.util.LinkedList;

import org.junit.Test;
import org.junit.Before;
import org.junit.Assert;

import java.lang.Math;


public class HeuristicEuclideanTest {

	CellNode goal;
	HeuristicEuclidean heuristic;
	
	@Before
	public void setUp() throws Exception, IOException {
		goal = new CellNode(20,20);		
		heuristic = new HeuristicEuclidean(goal);
	}
	
	@Test
	public void testIdenticalNodes() throws IOException {	
		CellNode n1 = new CellNode(25,20);
		n1.distance = 0;
	    Assert.assertTrue( "Violated: g(n1) == g(n1)", heuristic.compare(n1, n1) == 0);
    }

	@Test
	public void testDistanceToGoalEstimate() throws IOException {	
		CellNode n1 = new CellNode(25,20);
		CellNode n2 = new CellNode(15,15);
        heuristic.estimateDistanceToGoal(n1); 
        heuristic.estimateDistanceToGoal(n2); 
	    Assert.assertEquals(5.0, n1.distanceToGoalEstimate, 1e-3);
	    Assert.assertEquals(7.071, n2.distanceToGoalEstimate, 1e-3);
    }
	
	@Test
	public void testHeuristicComparison() throws IOException {	
		CellNode n1 = new CellNode(25,20);
		CellNode n2 = new CellNode(15,15);
		
		n1.distance = 0;
		n2.distance = 0;
	    Assert.assertTrue( "Violated: g(n1) < g(n2)", heuristic.compare(n1, n2) == -1);
	    Assert.assertTrue( "Violated: g(n2) > g(n1)", heuristic.compare(n2, n1) == 1);

		
		n1.distance = 200;
		n2.distance = 1;
	    Assert.assertTrue( "Violated: g(n1) > g(n2)", heuristic.compare(n1, n2) == 1);
	    Assert.assertTrue( "Violated: g(n2) < g(n1)", heuristic.compare(n2, n1) == -1);
    }	
	

}





