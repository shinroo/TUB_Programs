import java.io.IOException;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;


public class MinCutTest
{



	@Test
	public void testReachableNodes() throws IOException{
		Network g;
		g = GraphIO.loadGraph("tests/testgraphen/graphMC_2.txt");
		
		g.fordFulkerson();
		List<Node> nodes = g.residualGraph.findReachableNodes(0);
		
		Assert.assertEquals("findReachableNodes() does not find the correct number of nodes.", 2, nodes.size());
		int node0inSet=0;
		int node1inSet=0;
		for (Node n : nodes) {
			if (0 == n.getID()) {
				node0inSet++;
			} else if (1 == n.getID()){
				node1inSet++;
			} else {
				Assert.fail("The set contains an unexpected node");
			}
		}			
		Assert.assertEquals("Node 0 is missing in the Source set", 1, node0inSet);		
		Assert.assertEquals("Node 1 is missing in the Source set", 1, node1inSet);		
		
	}

	
	@Test
	public void testMinCutGraph1() throws IOException{
		Network g;
		g = GraphIO.loadGraph("tests/testgraphen/graphMC_1.txt");
		
		List<Edge> cutset = g.findMinCut();
		Assert.assertEquals("size of cutset is wrong.", 1, cutset.size());
		Edge e = cutset.get(0);
		Assert.assertEquals("Wrong edge was cut (wrong startnode).", 0, e.getStartnode().getID());
		Assert.assertEquals("Wrong edge was cut (wrong endnode).", 1, e.getEndnode().getID());
	}

	@Test
	public void testMinCutGraph2() throws IOException{
		Network g;
		g = GraphIO.loadGraph("tests/testgraphen/graphMC_2.txt");
		List<Edge> cutset = g.findMinCut();
		String msg = String.format("Number of edges cut expected: %d, but got %d", 2, cutset.size());
		Assert.assertEquals(msg, 2, cutset.size());
	}

 }


