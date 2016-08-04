import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Random;

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
 * 
 * Furthermore, there are methods to access a single edge or to check whether
 * two nodes are connected as well as methods to access all nodes connected with
 * a specific node and thus all outgoing edges.
 * 
 * @author Joerg Schneider, modified by AlgoDat-Tutoren
 */
public class GridGraph extends DiGraph {

	public HashSet<CellNode> cellnodes; 
	protected Picture picture;

	int rows;
	int columns;
	
	public GridGraph() {
		super();
		cellnodes = new HashSet<CellNode>();
		rows = 0;
		columns = 0;
	}

	public GridGraph(Picture p) {
		super();
		picture = p;
		cellnodes = new HashSet<CellNode>();
		
		//Load if node is free
		rows = p.getHeight();
		columns = p.getWidth();
		RGBColor[][] pixels = p.getImageMatrix();
		for (int i = 0; i< rows; i++) {
			for (int j = 0; j< columns; j++) {
				RGBColor pixel = pixels[j][i];
				boolean isFree = pixel.isWhite();
				if (isFree) {
					this.addNode(i,j);
				}
			}
		}
		
		//connect all nodes with the given moves:
		for (CellNode n : getCellNodes()) {
			int i = n.getRow();
			int j = n.getColumn();
			
			//Connect every node to its 4 direct neighbors:
			CellNode neighbor;
			neighbor = getCellNode(i-1,j);			
			if (neighbor != null) {n.addEdge(neighbor, 1);}
			neighbor = getCellNode(i+1,j);			
			if (neighbor != null) {n.addEdge(neighbor, 1);}
			neighbor = getCellNode(i,j-1);			
			if (neighbor != null) {n.addEdge(neighbor, 1);}
			neighbor = getCellNode(i,j+1);			
			if (neighbor != null) {n.addEdge(neighbor, 1);}

			/*uncomment this to have diagonal moves:*/
//			neighbor = getCellNode(i+1,j+1);			
//			if (neighbor != null) {n.addEdge(neighbor, 1);}
//			neighbor = getCellNode(i-1,j+1);			
//			if (neighbor != null) {n.addEdge(neighbor, 1);}
//			neighbor = getCellNode(i+1,j-1);			
//			if (neighbor != null) {n.addEdge(neighbor, 1);}
//			neighbor = getCellNode(i-1,j-1);			
//			if (neighbor != null) {n.addEdge(neighbor, 1);}
			
		}
		
		
	}


	Picture toPicture(List<Node> path) {
		
		RGBColor[][] pixels = new RGBColor[rows][columns];

		//create color palette:
		RGBColor white = new RGBColor(255,255,255);
		RGBColor gray  = new RGBColor(128,128,128);
		RGBColor black = new RGBColor(0,0,0);
		RGBColor red   = new RGBColor(255,0,0);
		
		//Paint all pixels black
		for(int i=0; i<rows; i++)
			for(int j=0; j<columns; j++)
				pixels[i][j] = black;
		
		//Paint pixels represented in the graph 
		for(CellNode n : cellnodes) {
			RGBColor pixelcolor;
			if (n.status != CellNode.WHITE) {pixelcolor = gray;}
			else 						   {pixelcolor = white;}
			pixels[n.getRow()][n.getColumn()] = pixelcolor;
		}
		
		//Paint path red
		if(path != null)
		{
			for(Node p : path)				
				pixels[getCellNode(p).getRow()][getCellNode(p).getColumn()] = red;
		}
		
		Picture p = new Picture(pixels);
		return p;	
	}

	

	@Override
	public Node addNode() {
		System.out.println("You need to supply a row and column index. Not adding any node.");
		return null;
	}

	/**
	 * Adds a node to the graph.
	 * 
	 * As the nodes are unlabeled this method has no parameter. An identifier is
	 * returned which can be used to access this specific node when calling the
	 * other operations.
	 * 
	 * @return the added Node.
	 */
	public Node addNode(int i,int j) {
		CellNode newOne = null;
		newOne = new CellNode(i,j);
		nodes.put(newOne.id, newOne);
		cellnodes.add(newOne);
		return newOne;
	}


	
	public Collection<CellNode> getCellNodes() {
		return cellnodes;
	}

	public CellNode getCellNode(int id) {
		return getCellNode(nodes.get(id));  //Simply downcast, as we ensure that we only add CellNodes
	}
	
	public CellNode getCellNode(int i, int j) {
		if (i < 0 || j < 0 || i >= rows || j >= columns) {return null;} //that  node cannot exist
		int id = CellNode.computeID(i,j);
		CellNode n;
		try {
			n = getCellNode(nodes.get(id));  //Simply downcast, as we ensure that we only add CellNodes
		} catch (Exception e){
			n = null;
		}
		return n;
	}

	/**
	 * type-safe accessor to convert Node objects back to CellNode objects 
	 */
	public CellNode getCellNode(Node n) {
		if (cellnodes.contains(n)) {
			return (CellNode) n;   //downcast
		} else {
			throw new IllegalArgumentException("node is not in this graph!");			
		}
	}
	
	public int getNodeRow(Node n) {
		CellNode cellnode = (CellNode) n; //Downcast, as we ensure that we only add CellNodes
		return cellnode.getRow();
	}
	
	public int getNodeColumn(Node n) {
		CellNode cellnode = (CellNode) n; //Downcast, as we ensure that we only add CellNodes
		return cellnode.getRow();
	}
	
	
	public void populateAStar(int startNodeID, int targetNodeID) {
		populateAStar(getCellNode(startNodeID), getCellNode(targetNodeID));
	}
	
	public void populateAStar(CellNode startCNode, CellNode targetCNode) {
		this.resetState();
		if(startCNode == null || !(cellnodes.contains(startCNode))){
			return;
		}
		
		// priority queue for candidate nodes
		PriorityQueue<CellNode> distanceQueue = new PriorityQueue<CellNode>(new HeuristicManhattan(targetCNode));

		// initialize distance of all nodes
		for (CellNode anyNode : cellnodes) {
			anyNode.distance = Integer.MAX_VALUE;
			anyNode.predecessor = null;
		}

		// set start node to GRAY, its distance to zero, and add it to the queue
		startCNode.status = Node.GRAY;
		startCNode.distance = 0;
		startCNode.predecessor = null;
		distanceQueue.add(startCNode);
		this.stopExecutionUntilSignal();

		// now work on all nodes in the priority queue until it's empty
		while (!distanceQueue.isEmpty()) {
			// get first node in the queue, i.e. the one with correct distance
			CellNode current = distanceQueue.poll();
			current.status = Node.BLACK;
			if (current == targetCNode) { //if targetNode is null, we will never trigger the stop
				//finished.
				return;
			}								
			this.stopExecutionUntilSignal();

			for (Edge incidentEdge : current.getIncidentEdges()) {
				// extract a neighbor as the endnode of the current
				// outgoing edge,
				// i.e. e.next
				CellNode neighbor = getCellNode(incidentEdge.endnode);

				if (neighbor.status == Node.WHITE) {
					// set status to GRAY
					neighbor.status = Node.GRAY;
					this.stopExecutionUntilSignal();

					neighbor.distance = current.distance + incidentEdge.weight;
					neighbor.predecessor = current;

					distanceQueue.add(neighbor);
				} else if (neighbor.status == Node.GRAY) {

					int newDistance = current.distance + incidentEdge.weight;
					if (newDistance < neighbor.distance) {
						distanceQueue.remove(neighbor);
						// correct distance value
						neighbor.distance = newDistance;
						neighbor.predecessor = current;
						// re-insert neighbor into the priority queue
						distanceQueue.add(neighbor);
					}
				}
			}
		}
	}

	public List<Node> getShortestPathAStar(int startNodeID, int targetNodeID) {
		return getShortestPathAStar(getCellNode(startNodeID), getCellNode(targetNodeID));
	}

	/**
	 * Calculates a List of Node indices that describe the shortet path from
	 * start node to target node. The A*-algorithm is used for
	 * calculation.
	 * 
     * @param startNode
	 *            the the target node
	 * @param targetNode
	 *            the the target node
	 * @return the list of nodes, or null if no path exists
	 */
	public List<Node> getShortestPathAStar(Node startNode, Node targetNode) {
		CellNode s = getCellNode(startNode);
		CellNode t = getCellNode(targetNode);
		
		// TODO: Your implementation here
		// NOTE: you have to run populateAStar first before you can read
		// out the shortest path

		//This line stops program execution until a key is pressed
		stopExecutionUntilSignal();

		//Return an empty list. Replace this line by your shortest path!
		LinkedList<Node> l = new LinkedList<Node>();
		
		//check if its the trivial case where s==t
		if (s.equals(t)){
			l.add(startNode);
			return l;
		}
		
		populateAStar(s, t);
		
		if(targetNode.distance == Integer.MAX_VALUE || targetNode.predecessor == null){
			return l;
		}
		Node temp = targetNode;
		while (!(temp.equals(startNode)) && temp.predecessor != null){ 
			l.addFirst(temp);
			temp = temp.predecessor;
		}
		l.addFirst(startNode);
		return l;
	}


	// ---- Methods to visualize the operations of a graph algorithm -----




}

