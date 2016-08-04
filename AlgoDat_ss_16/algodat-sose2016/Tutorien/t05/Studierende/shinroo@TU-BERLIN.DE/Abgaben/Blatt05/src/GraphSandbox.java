
import java.io.IOException;
import java.util.List;

public class GraphSandbox{
	
	
	private static void testDFSearch() {
		try {
			DiGraph graph = GraphIO.loadGraph("./tests/testgraphen/graphBFS_VS_DFS.txt");
			graph.setShowSteps(true);
			VisualGraph visualization = new VisualGraph(graph);
			
			System.out.println(graph.depthFirstSearch(0));
			
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	private static void testBFSearch() {
		try {
			DiGraph graph = GraphIO.loadGraph("./tests/testgraphen/graphBFS_VS_DFS.txt");
			graph.setShowSteps(true);
			VisualGraph visualization = new VisualGraph(graph);
			
			System.out.println(graph.breadthFirstSearch(0));
			
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private static void testVisualization() {
		try {
			DiGraph graph = GraphIO.loadGraph("./tests/testgraphen/graph02.txt");
			graph.setShowSteps(true);
			VisualGraph visualization = new VisualGraph(graph);
			
			graph.showGraph(0);
			
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	private static void testHasCycle() {
		try {
			// graph01 has a cycle
			DiGraph graph = GraphIO.loadGraph("./tests/testgraphen/graph01.txt");
			graph.setShowSteps(false);
			
			// should print true
			System.out.println(graph.hasCycle());
			
			// graph02 has no cycle
			graph = GraphIO.loadGraph("./tests/testgraphen/graph02.txt");
			graph.setShowSteps(false);
			
			// should print false
			System.out.println(graph.hasCycle());
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	private static void testTopSort() {
		try {
			DiGraph graph = GraphIO.loadGraph("./tests/testgraphen/graph04.txt");
			graph.setShowSteps(false);
			VisualGraph visualization = new VisualGraph(graph);
			// what should be printed here?
			List<Node> list = graph.topSort();
			if (list == null)
				return;
			for (Node n : list) {
				System.out.print(" " + n.id);
			}
			
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public static void main(String args[]) {
		testHasCycle();
		testVisualization();
		testDFSearch();
		testBFSearch();
		testHasCycle();
		//
		testTopSort();
	}


 }

