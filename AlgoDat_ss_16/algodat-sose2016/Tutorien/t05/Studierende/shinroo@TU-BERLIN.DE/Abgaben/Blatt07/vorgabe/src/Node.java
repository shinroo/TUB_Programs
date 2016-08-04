import java.util.ArrayList;
import java.util.LinkedList;

/**
 * The class <code>Node</code> implements a node in a network.
 * 
 * @author Uwe Hahne
 * @version 0.1
 */
public class Node {

	LinkedList<Edge> edges;

	int id;

	/**
	 * Stores the status of this node, which is either WHITE, GRAY, or BLACK.
	 */
	public int status;
	public static final int WHITE = 0;
	public static final int GRAY = 1;
	public static final int BLACK = 2;

	/**
	 * Constructor that creates specified node.
	 * 
	 * @param name
	 *            the drawn value in visualization
	 */
	public Node(int name) {
		// initialize list of edges
		edges = new LinkedList<Edge>();
		this.id = name;
	}

	/**
	 * An edge pointing from this node to end node is inserted at the beginning
	 * of the edge list.
	 * 
	 * @param startNode
	 *            Start point of this edge.
	 * @param endNode
	 *            End point of this edge.
	 * @param weight
	 *            Weight of this edge.
	 * 
	 */
	public void addEdge(Node startNode, Node endNode, int weight) {
		edges.addFirst(new Edge(startNode, endNode, weight));
	}

	public Edge getEdgeTo(Node targetNode) {
		for (Edge e : edges) {
			if (targetNode.equals(e.endNode)) {
				return e;
			}
		}
		return null;
	}

	public int getID() {
		return id;
	}

	/**
	 * Returns an list of nodes which are connected to this node.
	 * 
	 * @return list of nodes connected to this node
	 */
	public ArrayList<Node> getSuccessorNodes() {
		ArrayList<Node> succNodes = new ArrayList<Node>();
		for (Edge e : edges)
			succNodes.add(e.endNode);
		return succNodes;
	}

	/**
	 * Determines if this node is connected to a given one.
	 * 
	 * @param endNode
	 *            End point of the edge.
	 * 
	 * @return true if node is connected to given end node
	 */
	public boolean hasEdgeTo(Node endNode) {
		for (Edge e : edges) {
			if (e.endNode.equals(endNode)) {
				return true;
			}
		}
		return false;
	}

	/**
	 * Generates a collection of all edges that are incident with this node.
	 * 
	 * @return the collection of incident edges.
	 */
	public LinkedList<Edge> getIncidentEdges() {
		return new LinkedList<Edge>(edges);
	}

	@Override
	public String toString() {
		return "Node(id=" + id + ", edges=" + edges + ")";
	}

	@Override
	// essential for drawing. do not besmudge ye scurvy dog!
	public boolean equals(Object o) {
		if (o instanceof Node) {
			Node other = (Node) o;
			return id == other.getID();
		}
		return false;
	}

	@Override
	// essential for drawing. do not besmudge ye scurvy dog!
	public int hashCode() {
		return id;
	}
}

