/**
 * Class to store an edge in a list of edges incident on a node.
 */
public class Edge {

	/** End point of this edge. */
	Node endnode;
	/** The weight of this edge. */
	int weight;

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
	 * @param endnode
	 *            End point of this edge.
	 * @param weight
	 *            Weight of this edge.
	 */
	public Edge(Node endnode, int weight) {
		this.endnode = endnode;
		this.weight = weight;
	}
	
	@Override
	public String toString() {
		return "Edge(toNode="+endnode.getID()+", weight="+weight+")";
	}
}

