

/**
 * Class to store an edge in a list of edges incident on a node.
 * 
 * @author AlgoDat-Team
 */

/**

 */
public class Edge implements Comparable<Edge> {

	/**
	 * End point of this edge.
	 * */
	private Node endnode;

	/**
	 * Start point of this edge.
	 */
	private Node startnode;

	/**
	 * The weight of this edge.
	 * */
	private int weight;
	
	/**
	 * Stores the status of this edge
	 */
	public int status;

	/**
	 * Creates an edge
	 * 
	 * @param endnode
	 *            End point of this edge.
	 * @param weight
	 *            Weight of this edge.
	 */
	public Edge(Node startnode, Node endnode, int weight) {
		this.startnode = startnode;
		this.endnode = endnode;
		this.weight = weight;
	}
	
	/**
	 * returns the end node of this edge.
	 * 
	 * @return end node
	 */
	public Node getEndnode() {
		return this.endnode;
	}

	/**
	 * returns the start node of this edge
	 * 
	 * @return start node
	 */
	public Node getStartnode() {
		return this.startnode;
	}

	/**
	 * returns the weight of this edge
	 * 
	 * @return weight
	 */
	public int getWeight() {
		return this.weight;
	}

	/**
	 * Converts this node into string.
	 */
	@Override
	public String toString() {
		return "Edge(toNode="+endnode.getID()+", weight="+weight+")";
	}

	/**
	 * Two edges are equal if the got the same start and end node and the same
	 * weight
	 */
	@Override
	public boolean equals(Object o) {
		if (o instanceof Edge) {
			Edge other = (Edge) o;
			if (this.getStartnode().equals(other.getStartnode())
					&& this.getEndnode().equals(other.getEndnode())
					&& this.weight == other.weight) {
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Compares the weight of two given edges
	 */
	@Override
	public int compareTo(Edge o) {

		return this.weight - o.weight;
	}

}

