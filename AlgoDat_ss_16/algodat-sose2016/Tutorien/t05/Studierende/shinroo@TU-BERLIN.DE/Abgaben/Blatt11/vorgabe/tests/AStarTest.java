import java.io.IOException;
import java.util.List;
import java.util.LinkedList;

import org.junit.Test;
import org.junit.Before;
import org.junit.Assert;

import java.lang.Math;


public class AStarTest {

	private GridGraph g1;
	Picture pictureAllwhite;
	Picture pictureA;
	Picture pictureB;
	Picture pictureC;
	
	@Before
	public void setUp() throws Exception, IOException {
		// read graph from file
		
		pictureAllwhite = new Picture("tests/testgrids/16x16_allwhite.png");
		pictureA = new Picture("tests/testgrids/16x16_A.png");
		pictureB = new Picture("tests/testgrids/10x100.png");
//		pictureC = new Picture("tests/testgrids/1000x1000_slot.png");
//		pictureC = new Picture("tests/testgrids/1000x1000_bugtraps.png");
		pictureC = new Picture("tests/testgrids/1000x1000_narrowpassage.png");
		
	}


	@Test
	public void testTrivialVertical() throws IOException {		
		g1 = new GridGraph(pictureB);
		g1.setShowSteps(false);

		Node startNode = g1.getCellNode(4,50);
		Node targetNode = g1.getCellNode(5,50);
		List<Node> path_astar = g1.getShortestPathAStar(startNode, targetNode);
		List<Node> path_expected = new LinkedList<Node>();
		path_expected.add(startNode);
		path_expected.add(targetNode);
		Assert.assertEquals(path_astar, path_expected);
    }

	@Test
	public void testTrivialGoalIsTarget() throws IOException {		
		g1 = new GridGraph(pictureA);
		g1.setShowSteps(false);

		CellNode startNode = g1.getCellNode(12,2);
		CellNode targetNode = g1.getCellNode(12,2);
		List<Node> path_astar = g1.getShortestPathAStar(startNode, targetNode);
		List<Node> path_expected = new LinkedList<Node>();
		path_expected.add(g1.getCellNode(12,2));
		Assert.assertEquals(path_expected, path_astar);
    }

    
	@Test
	public void testAStarDrawPathPictureC() throws IOException {		
		// This is not an automatic test. It will always succeed. You have to visually inspect the written pictures 
		// found_path_dijkstra.png and found_path_astar.png written in the project directory.
		// Grey pixels are visited nodes, white pixels are unvisited nodes, red pixels are nodes on the path
		// (black pixels are empty areas with no nodes) 
		// The goal node is in the lower-right of the picture (start node is in the middle)
		g1 = new GridGraph(pictureC);
		g1.setShowSteps(false);
		// fill distance and predecessor values

		//This start and end node can be connected in a straight line:
		//CellNode startNode = g1.getCellNode(CellNode.computeID(0,500));
		//CellNode targetNode = g1.getCellNode(CellNode.computeID(999,500));
		
		//This pair of nodes can be connected quickly as they are close together:
		CellNode startNode = g1.getCellNode(CellNode.computeID(500,500));
		CellNode targetNode = g1.getCellNode(CellNode.computeID(600,600));

		
		System.out.print("Running Dijkstra...");
		List<Node> path_dijkstra = g1.getShortestPathDijkstra(startNode, targetNode);
		System.out.println("finished.");
		g1.toPicture(path_dijkstra).savePicture("found_path_dijkstra.png");
		
		g1.resetState();
		
		System.out.print("Running A*...");
		List<Node> path_astar = g1.getShortestPathAStar(startNode, targetNode);
		System.out.println("finished.");		
		g1.toPicture(path_astar).savePicture("found_path_astar.png");

	}
}





