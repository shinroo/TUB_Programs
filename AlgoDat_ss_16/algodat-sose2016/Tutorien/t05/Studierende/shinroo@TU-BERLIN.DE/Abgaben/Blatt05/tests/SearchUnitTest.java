import static org.junit.Assert.*;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;


public class SearchUnitTest {
	private Graph g1;

	@Before
	public void setUp() throws Exception {
		// read graph from file
		try {
			g1 = GraphIO.loadGraph("tests/testgraphen/graphBFS_VS_DFS.txt");
			g1.setShowSteps(false);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	@Test
	public void testDepthFirstOrderEmptyGraph() {
		fail("Not yet implemented");
	}	

	@Test
	public void testDepthFirstOrderNonExistingNode() {
		fail("Not yet implemented");
	}

	@Test
	public void testDepthFirstOrder() {
		fail("Not yet implemented");
	}

	@Test
	public void testBreadthFirstOrderEmptyGraph() {
		fail("Not yet implemented");
	} 

	@Test
	public void testBreadthFirstOrderNonExistingNode() {
		fail("Not yet implemented");
	}
  
	@Test
	public void testBreadthFirstOrder() {
		fail("Not yet implemented");
	}	
	

}

