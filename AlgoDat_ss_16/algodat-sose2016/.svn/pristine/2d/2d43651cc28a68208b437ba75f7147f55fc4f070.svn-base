import java.io.IOException;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Queue;

/**
 * @author Uwe + Damien
 * 
 */
public class Network implements INetwork {
	// -- attributes --
	private HashMap<Integer, Node> nodes;

	// -- constructor --
	public Network() {
		nodes = new HashMap<Integer, Node>();
	}

	// -- node functions --
	/**
	 * returns the nodes
	 */
	@Override
	public LinkedList<Node> getNodes() {
		return (LinkedList<Node>) nodes.values();
	}

	@Override
	public Node addNode() {
		int newId = nodes.size();
		Node newNode = new Node(newId);
		nodes.put(newId, newNode);
		return newNode;
	}

	// -- edge functions --
	public void addEdge(Node startNode, Node endNode, int capacity) {
		if (!(testEdgeNodes(startNode, endNode)))
			inputError();
		startNode.addEdge(startNode, endNode, capacity);
	}

	public void addEdge(int startnode, int endnode, int capacity) {
		addEdge(nodes.get(startnode), nodes.get(endnode), capacity);
	}

	/**
	 * Returns graph edge specified by source and destination indices.
	 * 
	 * @param startNodeInd
	 *            index of start node
	 * @param targetNode
	 *            index of target node
	 */
	public Edge getEdge(int startNodeInd, int targetNodeint) {
		Node n = nodes.get(startNodeInd);
		for (Edge e : n.getIncidentEdges())
			if (e.endNode.id == targetNodeint)
				return e;
		return null;
	}

	public boolean testEdgeNodes(Node startNode, Node endNode) {
		return (startNode != null) && (endNode != null)
				&& nodes.values().contains(startNode)
				&& nodes.values().contains(startNode);
	}

	// -- state reset functions --
	/**
	 * resets the state of all nodes and edges to white
	 */
	public void clearMarksAll() {
		clearMarksNodes();
		for (Node currentNode : nodes.values())
			for (Edge currentEdge : currentNode.getIncidentEdges())
				currentEdge.status = Edge.WHITE;
	}

	/**
	 * help function to reset the state of all nodes to white
	 */
	public void clearMarksNodes() {
		for (Node n : nodes.values())
			n.status = Node.WHITE;
	}

	public boolean isAdjacent(Node startNode, Node endNode) {
		if (!(testEdgeNodes(startNode, endNode)))
			inputError();
		return startNode.hasEdgeTo(endNode);
	}

	/**
	 * Searches for sources in the graph
	 * 
	 * @return All sources found in the graph
	 */
	public Node findSource() {
		LinkedList<Node> sources = new LinkedList<Node>();
		boolean isSource = true;
		// source <-> no incoming edges
		for (Node n : nodes.values()) {
			isSource = true;
			for (Node m : nodes.values()) {
				if (!m.equals(n) && isAdjacent(m, n)) {
					isSource = false;
					break;
				}
			}
			if (isSource)
				sources.add(n);
		}
		// error handling
		testSingle(sources);
		return sources.getFirst();
	}

	/**
	 * Searches the graph for sinks.
	 * 
	 * @return All sinks found in the graph
	 */
	public Node findSink() {
		LinkedList<Node> sinks = new LinkedList<Node>();
		// sink <-> no incoming edges
		for (Node n : nodes.values()) {
			if (n.getIncidentEdges().isEmpty())
				sinks.add(n);
		}
		// error handling
		testSingle(sinks);
		return sinks.getFirst();
	}

	public void testSingle(LinkedList<Node> nodes) {
		if (nodes.size() == 0 || nodes.size() > 1)
			inputError();
	}

	/**
	 * Finds an augmenting path from start to end in the graph A path is
	 * augmenting if all it's edges have residual capacity > 0 (You can choose
	 * from several algorithms to find a path)
	 * 
	 * @param startNodeId
	 *            the id of the start node from where we should start the search
	 * @param endNodeId
	 *            the id of the end node which we want to reach
	 * 
	 * @return the path from start to end or an empty list if there is none
	 */
	
	/*
	 * Adapted from following python code found on wikipedia:
	 * (https://en.wikipedia.org/wiki/Ford%E2%80%93Fulkerson_algorithm)
	 def find_path(self, source, sink, path):
        if source == sink:
            return path
        for edge in self.get_edges(source):
            residual = edge.capacity - self.flow[edge]
            if residual > 0 and edge not in path:
                result = self.find_path( edge.sink, sink, path + [edge]) 
                if result != None:
                    return result
	 */
	
	public LinkedList<Node> findAugmentingPathWithPaths(int startNodeId, int endNodeId, LinkedList<Node> path){
		LinkedList<Node> newPath = new LinkedList<Node>();
		Node startNode = nodes.get(startNodeId);
		
		if (startNodeId == endNodeId){
			return path;
		}
		
		int res = 0;
		
		for (Edge edge: startNode.getIncidentEdges()){
			res = edge.capacity - edge.currentFlow;
			if (res > 0 && !(path.contains(edge.getEndnode()))){
				path.add(edge.getEndnode());
				newPath = findAugmentingPathWithPaths(edge.getEndnode().id, endNodeId, path);
				if (!(newPath.isEmpty())){
					return newPath;
				}
			}
		}
		//if algorithm reaches this point, no augmenting path has been found: return empty list
		newPath.clear();
		return newPath;
	}
	
	public LinkedList<Node> findAugmentingPath(int startNodeId, int endNodeId) {
		// TODO: Your implementation here
		if (!(nodes.containsKey(startNodeId) && nodes.containsKey(endNodeId))) 
			throw new RuntimeException("Either the start or end node is not in the hash map!");
		
		Queue<Node> pathQueue = new LinkedList<Node>();
		LinkedList<Node> path = new LinkedList<Node>();
		
		//check if there is a path between the nodes, if not return empty list
		pathQueue = breadthFirstSearch(startNodeId);
		if (!(pathQueue.contains(nodes.get(endNodeId))) || pathQueue.isEmpty()) return path;
		
		path.add(nodes.get(startNodeId));
		path = findAugmentingPathWithPaths(startNodeId, endNodeId, path);
		
		return path;
	}

	// -- code to complete --

	/**
	 * Computes the maximum flow over the network with the Ford-Fulkerson
	 * Algorithm
	 * 
	 * @returns Value of maximal flow
	 */
	public int getMax(Node node){
		int max = 0;
		for (Edge edge: node.edges){
			max += edge.currentFlow;
		}
		return max;
	}
	
	public int fordFulkerson() {

		// These methods find the source and sink in the network
		Node source = findSource();
		Node sink = findSink();

		// You can use this method to create a residual network
		Network residualGraph = initializeResidualGraph();
		
		LinkedList<Node> path = new LinkedList<Node>();
		path = findAugmentingPath(source.id, sink.id);
		while(!(path.isEmpty())){
			residualGraph.updateResidualCapacity(residualGraph.findMinCapacity(path), path);
			path = findAugmentingPath(source.id, sink.id);
		}

		// TODO: Your implementation here
		return getMax(source);
	}


	/**
	 * Builds the residual graph to a flow graph
	 * 
	 * @return the residual graph to this flow graph
	 */
	public Network initializeResidualGraph() {

		Network residualGraph = new Network();

		// adding nodes
		for (Node n : nodes.values())
			residualGraph.addNode();
		// adding edges
		for (Node n : nodes.values()) {
			for (Edge e : n.getIncidentEdges()) {
				// Add forward edges with same capacity
				residualGraph.addEdge(n.id, e.endNode.id, e.capacity);
				// Add backwards edges
				residualGraph.addEdge(e.endNode.id, n.id, 0);
			}
		}

		return residualGraph;
	}

	/**
	 * Finds the minimal residual capacity over the given path
	 * 
	 * @return the minimal capacity
	 */
	public int findMinCapacity(LinkedList<Node> path) {
		// TODO: Your implementation here
		int minCap = Integer.MAX_VALUE;
		
		LinkedList<Node> secondList = new LinkedList<Node>();
		secondList.addAll(path);
		Node currNode = secondList.poll();
		Edge currEdge;
		for (Node next: secondList){
			currEdge = getEdge(currNode.getID(), next.getID());
			if (currEdge.getCapacity() - currEdge.getCurrentFlow() < minCap) 
				minCap = currEdge.getCapacity() - currEdge.getCurrentFlow();
			currNode = next;
		}
		
		return minCap;
	}

	/**
	 * Update capacity on given path, to be executed on residual graph
	 */
	public boolean isForward(Edge e){
		boolean forward = false;
		if (e.capacity > 0) forward = true;
		return forward;
	}
	
	public void updateResidualCapacity(int minCapacity, LinkedList<Node> path) {
		// TODO: Your implementation here
		LinkedList<Node> secondList = new LinkedList<Node>();
		secondList.addAll(path);
		Node currNode = secondList.poll();
		Edge first, second;
		for (Node next: secondList){
			first = getEdge(currNode.getID(), next.getID());
			second = getEdge(next.getID(), currNode.getID());
			//determine which edge is the forward edge:
			if(isForward(first)){
				first.currentFlow += minCapacity;
				second.currentFlow -= minCapacity;
			}
			if(isForward(second)){
				first.currentFlow -= minCapacity;
				second.currentFlow += minCapacity;
			}
			currNode = next;
		}
	}

	/**
	 * Calculates the current flow in this graph.
	 * 
	 * @param source
	 *            the source of the flow
	 * 
	 * @return the value of the flow
	 */
	public int getFlow(Node source) {
		int flow = 0;
		for (Edge e : source.getIncidentEdges()) {
			if (e.currentFlow > 0)
				flow += e.currentFlow;
		}
		return flow;
	}

	public LinkedList<Node> breadthFirstSearch(int startNode) {
		return breadthFirstSearch(nodes.get(startNode));
	}

	public LinkedList<Node> breadthFirstSearch(Node startNode) {
		LinkedList<Node> nodeList = null;
		clearMarksNodes();

		if (startNode == null || !nodes.values().contains(startNode)) {
			nodeList = new LinkedList<Node>();
		} else {
			nodeList = new LinkedList<Node>();
			LinkedList<Node> queue = new LinkedList<Node>();

			startNode.status = Node.GRAY;
			queue.addLast(startNode);

			while (!queue.isEmpty()) {
				Node current = queue.removeFirst();
				current.status = Node.BLACK;
				nodeList.addLast(current);

				for (Node neighbor : current.getSuccessorNodes()) {

					if (neighbor.status == Node.WHITE) {
						neighbor.status = Node.GRAY;
						queue.addLast(neighbor);
					}
				}
			}
		}
		return nodeList;
	}

	// -- utils --
	public void inputError() {
		System.out.println("Incorrect input.");
		System.exit(1);
	}
}
