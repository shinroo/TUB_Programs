import java.io.IOException;

import org.junit.Test;
import org.junit.Assert;

public class GraphTest {

	// Test correct total edge weight of MST
	@Test
	public void testWeight() {
		Graph g1;
		try {
			g1 = GraphIO.loadGraph("tests/testgraphen/test_kruskal01.txt");

			// get mst
			Graph mst = g1.toMinSpanTree();

			// get actual weight
			int actual = 0;
			for (Node n : mst.getNodes()) {
				for (Edge e : n.getIncidentEdges()) {
					actual += e.getWeight();
				}
			}

			// get correct weight
			int expected = 12;
			
			Assert.assertEquals("This is not the minimum spanning tree",
					expected, actual);
			
		} catch (IOException e1) {
			e1.printStackTrace();
		}
	}
	
	//You can implement the following tests yourself.
	//We recommend to add more test than just these examples.
	
	// test if the MinSpanTree has the correct number of edges
	@Test
	public void testEdgeCount() {
	}

	// Test if every node has at least one edge (in an MST with more than 1 node)
	@Test
	public void testEveryNodeHasEdges() {
	}

	// Test if the MinSpanTree actually connects all nodes
	@Test
	public void testConnectivity() {
	}


}

