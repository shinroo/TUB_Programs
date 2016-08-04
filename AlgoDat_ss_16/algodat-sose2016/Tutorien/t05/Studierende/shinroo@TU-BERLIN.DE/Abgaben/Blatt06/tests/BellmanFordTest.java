import java.io.IOException;
import java.util.List;
import java.util.LinkedList;

import org.junit.Test;
import org.junit.Before;
import static org.junit.Assert.assertEquals;

public class BellmanFordTest {
	
	private Graph g2;

	@Before
	public void setUp() throws Exception {
		// read graph from file
		try {
			g2 = GraphIO.loadGraph("tests/testgraphen/graphBellmanFord.txt");
			g2.setShowSteps(true);
			
			VisualGraph v = new VisualGraph(g2);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	@Test
	public void testBellmanFordShortestPath(){
		
		// fill distance and predecessor values
		g2.populateBellmanFordFrom(0);
		
		// expected list
		LinkedList<Integer> expect = new LinkedList<Integer>();
		expect.addFirst(4);
		expect.addFirst(3);
		expect.addFirst(2);
		expect.addFirst(1);
		expect.addFirst(0);
		
		// test actual getShortestPath from 0 to 4
		List<Node> actualNodes = g2.getShortestPathBellmanFord(0,4);
		
		assertEquals(
				"getShortestPath didn't find the path with the right number of nodes",
				actualNodes.size(), expect.size());
		
		LinkedList<Integer> actual = new LinkedList<Integer>();
		for(Node n : actualNodes){
			actual.addLast(n.getID());
		}
		
		assertEquals("getShortestPath didn't return the shortest path", actual.toString(), expect.toString());
	}

 }








