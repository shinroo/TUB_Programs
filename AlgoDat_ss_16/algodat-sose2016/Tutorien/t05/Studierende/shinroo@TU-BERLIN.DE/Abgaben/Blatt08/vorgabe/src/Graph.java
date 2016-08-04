import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;


/**
 * A data structure to store a graph consisting of a set of unlabeled nodes and
 * a set of directed, weighted edges connecting the nodes.
 * 
 * The interface provides methods to add nodes and edges. Both methods can be
 * called at all time and the data structure should not assume a limitation on
 * the number of nodes or edges.
 * 
 * Furthermore, there are methods to access a single edge or to check whether
 * two nodes are connected as well as methods to access all nodes connected with
 * a specific node and thus all outgoing edges.
 * 
 * @author Joerg Schneider, modified by AlgoDat-Team
 */
public interface Graph {

	/**
	 * Returns all nodes of the graph.
	 * 
	 * @return the hashmap containing all nodes
	 */
	public List<Node> getNodes();
		
	
	/** This weight is returned for non-existent edges. */
	public static final int WEIGHT_NO_EDGE = 0;

	/**
	 * Adds a node to the graph.
	 * 
	 * As the nodes are unlabeled this method has no parameter. An identifier is
	 * returned which can be used to access this specific node when calling the
	 * other operations.
	 * 
	 * @return the added Node.
	 */
	public Node addNode();

	/**
	 * Adds an edge to the graph.
	 * 
	 * The edge is specified by a start node and a target node. If there is
	 * already an edge between these two nodes the behavior of the method is
	 * unspecified. The edge is labeled with a positive weight value.
	 * 
	 * @param startNode the start node
	 * @param targetNode the target node
	 * @param weight a positive, non-zero value assigned to this edge
	 */
	public void addEdge(int startNode, int targetNode, int weight);
	
	/**
	 * Adds an undirected edge to the graph.
	 * 
	 * The edge is specified by a start node and a target node. If there is
	 * already an edge between these two nodes the behavior of the method is
	 * unspecified. The edge is labeled with a positive weight value.
	 * 
	 * @param startNode the start node
	 * @param targetNode the target node
	 * @param weight a positive, non-zero value assigned to this edge
	 */
	public void addUndirectedEdge(int startNode, int targetNode, int weight);

	/**
	 * For a node returns all nodes the outgoing edges of the node directly lead to.
	 * 
	 * @return list containing all directly connected nodes.
	 */
	public List<Node> getAdjacentNodes(int startNode);
	
	
	/**
	 * Tests if the current execution of a graph-algorithm was stopped
	 * @return true if the processing is stopped, false otherwise. 
	 */
	public boolean isStopped();
	
	/**
	 * Stops or resumes the current executed graph algorithm
	 * @param status true to stop, false to resume execution
	 */
	public void setStopped(boolean status);
	
	// ---- Methods for Kruskal -----------------------------------------------
	
	/**
	 * calculates the minimum span tree using Kruskal's algorithm
	 * @param graph the graph to get the MST for
	 * @return a new Graph representing the minimum span tree
	 */
	public Graph toMinSpanTree();
	
		
	// ========================= METHODS BELOW USED BY AUXILIARY CLASSES ========================= //
	
	/**
	 * Determines if the given nodes are directly connected, that is whether
	 * there is some directed edge that links startNode to targetNode.
	 * For all existing nodes: isConnected(n1, n2) <=> getAdjacentNodes(n1).contains(n2)
	 * 
	 * @param startNode
	 *            the start node as provided by addNode()
	 * @param targetNode
	 *            the target node as provided by
	 *            addNode()
	 * @return true iff there is a directed edge between the given start node
	 *         and target node.
	 */
	public boolean isConnected(int startNode, int targetNode);
	
	
	/**
	 * Returns the weight of the directed edge between the given nodes.
	 * 
	 * @param startNode
	 *            the the start node as provided by addNode()
	 * @param targetNode
	 *            the target node as provided by
	 *            addNode()
	 * @return WEIGHT_NO_EDGE, if there is no directed edge between the start
	 *         node and the target node or the positive, non-zero weight stored
	 *         by addEdge().
	 */
	public int getWeight(int startNode, int targetNode);
	
	/**
	 * Returns an index, representing the color of the given node.
	 * 0:black, 1:blue, 2:red, 3:green, 4:orange, 5:gray
	 * You may want to paint nodes that don't exist in gray.
	 * 
	 * @param nodeNumber the node for which the color
	 * 		is requested, as returned by addNode()
	 * 
	 * @return an index between 0 and 5, representing the color of the given node
	 */
	public int getColorOfNode(int nodeID);

	/**
	 * Returns an index, representing the color of the given edge.
	 * 0:black, 1:blue, 2:red, 3:green, 4:orange, 5:gray
	 * You may want to paint edges that don't exist in gray.
	 * 
	 * @param sourceNodeNumber the index of the node where the edge starts
	 * 	, as returned by addNode()
	 * @param targetNodeNumber the index of the node where the edge ends
	 * 	, as returned by addNode()
	 * 
	 * @return an index between 0 and 5, representing the color of the given edge
	 */
	public int getColorOfEdge(int start, int end);

	/**
	 * Sets the showSteps flag.
	 * @param show the flag
	 */
	public void setShowSteps(boolean show);

	/**
	 * travels randomly in the graph.
	 * used to demonstrate visualization class
	 * @param index of startnode
	 */
	public void showGraph(int startNode);

	/**
	 * resets all colors in visual representation of the graph to white
	 */
	void clearMarks();
}


