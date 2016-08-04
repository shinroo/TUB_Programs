import java.util.LinkedList;

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
 * @author Joerg Schneider, modified by AlgoDat-Tutoren
 */
public interface INetwork {

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
	 * @param startNode
	 *            the start node
	 * @param targetNode
	 *            the target node
	 * @param weight
	 *            a positive, non-zero value assigned to this edge
	 */
	public void addEdge(Node startNode, Node targetNode, int weight);
	public void addEdge(int startNode, int targetNode, int weight);

	/**
	 * Returns graph edge specified by source and destination indices.
	 * 
	 * 
	 * @param startNodeInd
	 *            index of start node
	 * @param targetNode
	 *            index of target node
	 */
	public Edge getEdge(int startNodeInd, int targetNodeint);

	/**
	 * returns the nodes
	 */
	public LinkedList<Node> getNodes();

	// ---- Methods for FordFulkerson -----------------------------------------

	/**
	 * Method to calculate the maxFlow in this graph
	 * 
	 * @return maximum flow value
	 */
	public int fordFulkerson();

	/**
	 * Builds the residual graph to a flow graph
	 * 
	 * @return the residual graph to this flow graph
	 */
	public INetwork initializeResidualGraph();


	/**
	 * Finds the nodes reachable from the source via edges of 
	 * nonzero capacity 
	 * 
	 * This method is intended to only be called on a residual graph
	 * 
	 * @param startNode
	 * 			The id of the start node
	 * 
	 * @return List of nodes 
	 */
	public LinkedList<Node> findReachableNodes(int startNode);
	
	/**
	 * Finds the nodes reachable from the source via edges of 
	 * nonzero capacity 
	 * 
	 * This method is intended to only be called on a residual graph
	 * 
	 * @param startNode
	 * 			The start node
	 * 
	 * @return List of nodes 
	 */	
	public LinkedList<Node> findReachableNodes(Node startNode);

	/**
	 * Finds an augmenting path from start to end in the graph
	 * A path is augmenting if all it's edges have residual capacity > 0
	 * 
	 * @param startNodeId
	 *            the id of the start node from where we should start the search
	 * @param endNodeId
	 *            the id of the end node which we want to reach
	 * 
	 * @return the path from start to end or an empty list if there is none
	 */
	public LinkedList<Node> findAugmentingPath(int start, int end);

	/**
	 * Finds the minimal residual capacity over the given path
	 * 
	 * @param path
	 * 		List of the nodes along the path
	 * 
	 * @return the minimal capacity
	 */
	public int findMinCapacity(LinkedList<Node> path);

	/**
	 * Update capacity on given path, to be executed on a residual graph
	 * 
	 * @params minCapacity
	 * 		the minimum capacity of the path
	 * @params path
	 * 		List of nodes along the path
	 */
	public void updateResidualCapacity(int minCapacity, LinkedList<Node> path);	
	
	/**
	 * Calculates the current flow in this graph.
	 * 
	 * @param source
	 *            the source of the flow
	 * @return the value of the flow
	 */
	public int getFlow(Node source);

	/**
	 * traverses the graph by breadth first
	 * 
	 * @param start
	 *            id of the startnode
	 * @return list containing nodes in traversed order
	 */
	public LinkedList<Node> breadthFirstSearch(int start);

}

