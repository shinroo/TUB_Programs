/**
 * Class to store an edge in a list of edges incident on a node.
 */
public class Edge {

	/** End points of this edge. */
	Node startNode;
	Node endNode;
	/** The capacity of this edge. */
	int capacity;
	/** The current flow of this edge. */
	int currentFlow;

	/**
	 * Stores the status of this edge, which is either WHITE, GRAY, or BLACK.
	 */
	public int status;
	static final int WHITE = 0;
	static final int GRAY = 1;
	static final int BLACK = 2;

	/**
	 * Creates an edge
	 * 
	 * @param startNode
	 *            Start point of this edge.
	 * @param endNode
	 *            End point of this edge.
	 * @param capacity
	 *            capacity of this edge.
	 */
	public Edge(Node startNode, Node endNode, int capacity) {
		this.startNode = startNode;
		this.endNode = endNode;
		this.capacity = capacity;
	}

	/**
	 * @return the startNode
	 */
	public Node getStartnode() {
		return startNode;
	}

	/**
	 * @return the endNode
	 */
	public Node getEndnode() {
		return endNode;
	}

	/**
	 * @param startNode
	 *            the startNode to set
	 */
	public void setStartnode(Node startNode) {
		this.startNode = startNode;
	}

	/**
	 * @param endNode
	 *            the endNode to set
	 */
	public void setEndnode(Node endNode) {
		this.endNode = endNode;
	}

	/**
	 * @return the capacity
	 */
	public int getCapacity() {
		return capacity;
	}

	/**
	 * @param capacity
	 *            the capacity to set
	 */
	public void setCapacity(int capacity) {
		this.capacity = capacity;
	}

	/**
	 * @return the currentFlow
	 */
	public int getCurrentFlow() {
		return currentFlow;
	}

	/**
	 * @param currentFlow
	 *            the currentFlow to set
	 */
	public void setCurrentFlow(int currentFlow) {
		this.currentFlow = currentFlow;
	}

	@Override
	public String toString() {
		return "(" + startNode.getID() + " --(" + currentFlow + "/" + capacity
				+ ")-> " + endNode.getID() + ")";
	}
}

