import java.util.List;


/**
 * An interface to manipulate a graph structure consisting of a set of unlabeled nodes and
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
 * @author Joerg Schneider, modified by MPGI2-Tutoren
 */
public interface Graph {

	/** This weight is returned for non-existent edges. */
	public static final int WEIGHT_NO_EDGE = 0;

	/**
	 * Creates a new node and adds it to the graph.
	 * 
	 * As the nodes are unlabeled this method has no parameter. An reference is
	 * returned which can be used to access this specific node when calling the
	 * other operations.
	 * 
	 * @return the added Node.
	 */
	public Node addNode();

	/**
	 * Adds a directed edge between two nodes.
	 * 
	 * The directed edge is specified by a start node and a target node. If there
	 * already is an edge between these two nodes the behavior of the method is
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
		
	/**
	 * Returns all nodes of the graph.
	 * 
	 * @return list containing all valid node identifiers.
	 */
	public List<Node> getNodes();

	/**
	 * Returns all adjacent nodes of a node, as defined by the directed edges in the graph.
	 * 
	 * @return list containing all directly connected nodes.
	 */
	public List<Node> getAdjacentNodes(Node startNode);
	
	
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
	
	
	/**
	 * Traverses the Graph using breadth-first search
	 * @param startNode the node to start the search with
	 * @return a list of the visited nodes, in the sequence visited visited during the search
	 */
	public List<Node> breadthFirstSearch(Node startNode);
	
	/**
	 * Traverses the Graph using breadth-first search
	 * For all existing nodes: <code>breadthFirstSearch(node.id) == breadthFirstSearch(node)</code>
	 * @param startNodeID the id of the node to start the search with
	 * @return a list of the visited nodes, in the sequence visited visited during the search
	 */
	public List<Node> breadthFirstSearch(int startNodeID);
	
	/**
	 * Traverses the Graph using depth-first search
	 * @param startNode the node to start the search with
	 * @return a list containing the reachable nodes, ordered as visited during the search
	 */
	public List<Node> depthFirstSearch(Node startNode);
	
	/**
	 * Traverses the Graph using depth-first search.
	 * For all existing nodes: <code>depthFirstSearch(node.id) == depthFirstSearch(node)</code>
	 * @param startNodeID the id of the node to start the search with
	 * @return a list containing the reachable nodes, ordered as visited during the search
	 */
	public List<Node> depthFirstSearch(int startNodeID);
	
	/**
	 * function to find Cycles in a graph by using depth-first search
	 * 
	 * @return true if an cycle was found else false
	 */
	public abstract boolean hasCycle();
	
	/**
	 * function for sorting a graph by topological order. the graph has to be
	 * free of cycles
	 * 
	 * @return a list of nodes, ordered by depenendcy: ABC if C depends on B depends on A
	 */
	public abstract List<Node> topSort();
	
	// ========================= METHODS BELOW USED BY AUXILIARY CLASSES ========================= //
	
	/**
	 * Adds an edge to the graph using node ids
	 * Serves as a wrapping call for addEdge(Node, Node)
	 * 
	 * The edge is specified by a start node and a target node. If there is
	 * already an edge between these two nodes the behavior of the method is
	 * unspecified. The edge is labeled with a positive weight value.
	 * 
	 * @param startNode
	 *            the start node id as provided by addNode().id
	 * @param targetNode
	 *            the target node id as provided by
	 *            addNode().id
	 * @param weight
	 *            a positive, non-zero value assigned to this edge
	 */
	public void addEdge(int startNode, int targetNode, int weight);

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
	public boolean isConnected(Node startNode, Node targetNode);
	
	/**
	 * Determines if the node corresponding to the given ids are connected.
	 * Serves as a wrapping call for isConnected(Node, Node)
	 * 
	 * @param startNode
	 *            the start node id
	 * @param targetNode
	 *            the target node id 
	 *            addNode()
	 * @return true, if there is a directed edge between the given start node
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
	public int getWeight(Node startNode, Node targetNode);
	
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
	public int getColorOfEdge(Node sourceNode, Node targetNode);

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
	void showGraph(Node startNode);

	/**
	 * getter for weight
	 * @param startnode start of the edge
	 * @param endnode end of the edge
	 * @return weight of the edge
	 */
	int getWeight(int startnode, int endnode);

	/**
	 * calculates all adjacent nodes to the current node
	 * @param startnode determines whose neighbors should be calculated
	 * @return neighbors of given node
	 */
	List<Node> getAdjacentNodes(int startnode);

	/**
	 * resets all colors in visual representation of the graph to white
	 */
	void clearMarks();

	/**
	 * getter for color of a node
	 * @param node node, whose color to look for
	 * @return color of node
	 */
	int getColorOfNode(Node node);
}


