import java.awt.Dimension;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.TreeMap;

import javax.swing.JFrame;

/**
 * This Class provides a visual representation of a graph data structure.
 * This class can only be used for the visualization, an object of the real
 * graph data structure is needed to perform the specified operations.
 *
 * If you want to display an already constructed graph just call
 * Graph visGraph = new VisualGraph(graph);
 * 
 * If you add or delete nodes or edges of the graph, the visualization will
 * continue to display them in a red color.
 * To delete them from the visualization, call
 * visGraph.rebuild()
 * 
 * The position of the nodes and edges will be automatically determined.
 * Therefore, this visualization reduces the speed of your program. Do not use this
 * class for benchmarks.
 * 
 * 
 * @author Joerg Schneider <komm@cs.tu-berlin.de>, modified by AlgoDat-Tutoren
 */
public class VisualGraph implements KeyListener, MouseListener{

	/** The visual representation of the graph */
	private GraphVisualizer panel;
	/** updates the visual graph every 100ms */
	private UpdateThread updateThread;

	/** The underlying graph data structure */
	private Graph graph;
	/**
	 * used to keep track of the mapping between the nodes on the display and
	 * node identifier in the graph data structure
	 */
	TreeMap<Integer, VisualNode> visualNodes = new TreeMap<Integer, VisualNode>();
	
	/**
	 * Set up a visual representation of the given Graph object.
	 * 
	 * By default the given graph will be traversed and all existing nodes and
	 * edges will be displayed.
	 * The graph object has to implement the
	 * getNodes() and getAdjacentNodes() method correctly, or the visualization 
	 * will fail
	 * 
	 * @param graph
	 *            the graph data structure to use
	 */
	public VisualGraph(Graph graph) {
		this.graph = graph;
		buildFrame();
		addAllNodes();
		this.updateThread = new UpdateThread(this);
		this.updateThread.start();
	}

	/**
	 * Traverses the data structure and creates a visual representation for each
	 * node and each edge in the graph.
	 */
	private void addAllNodes() {
		for (Node startNode : graph.getNodes()) {
			addVisualNode(startNode.getID());
		}
		for (Node startNode : graph.getNodes()) {
			for (Node targetNode : graph.getAdjacentNodes(startNode.getID())) {
				addVisualEdge(startNode.getID(), targetNode.getID(), graph.getWeight(startNode.getID(),
						targetNode.getID()));
			}
		}
	}

	/**
	 * Set up the window and initialize the graph drawing area.
	 */
	private void buildFrame() {
		JFrame frame = new JFrame("MPGI 2 VisualGraph: "
				+ graph.getClass().getName());
		frame.setSize(800, 600);
		panel = new GraphVisualizer(this.graph, new Dimension(800, 600));
		frame.add(panel);
		frame.setVisible(true);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.addKeyListener(this);
		frame.addMouseListener(this);
	}

	/**
	 * Generate the visual node and add it to the node mapping.
	 * 
	 * @param nodeNumber
	 *            the node number as provided by the underlying data structure
	 */
	private void addVisualNode(int nodeNumber) {
		// create visual representation
		VisualNode visualNode = panel.addNode(nodeNumber);
		// add mapping
		visualNodes.put(nodeNumber, visualNode);
	}


	/**
	 * Generate the visual edge.
	 * 
	 * @param startNodeNumber
	 *            the start node identifier as provided by the caller of
	 *            addEdge()
	 * @param targetNodeNumber
	 *            the target node identifier as provided by the caller of
	 *            addEdge()
	 * @param weight
	 *            the weight as provided by the caller of addEdge()
	 */
	private void addVisualEdge(int startNodeNumber, int targetNodeNumber,
			int weight) {
		// determine the visual nodes for the given node identifier
		VisualNode start = visualNodes.get(startNodeNumber);
		VisualNode target = visualNodes.get(targetNodeNumber);
		if ((start == null) || (target == null)) {
			throw new RuntimeException(
					"No visual node found for the given node identifier.");
		}
		// paint edge
		panel.addEdge(start, target, weight);
	}


	/** deletes all edges and nodes */
	private void clear(){
		this.panel.clear();
		this.visualNodes.clear();
	}

	/** 
	 * rebuilds the visual graph representation, 
	 * using current nodes and edges of the graph
	 * This may cause the position of nodes to change
	 */
	public void rebuild(){
		this.clear();
		this.addAllNodes();
	}
	
	/**
	 * repaints the graph, using current colors
	 */
	public void update(){
		this.panel.update();
	}
	
	/**
	 * Thread used to trigger an periodic repaint of the 
	 * graph visualization.
	 */
	private class UpdateThread extends Thread{
		VisualGraph graph;
		
		public UpdateThread(VisualGraph theGraph){
			this.graph = theGraph;
		}
		
		public void run(){
			while(true){				
				this.graph.update();
				try {
					Thread.sleep(100);
				} catch (InterruptedException e){
					// the VM doesn't want us to sleep anymore,
					// so get back to work
				}
			}
		}		
	}

	public void keyPressed(KeyEvent arg0) {
		// resume computation of graph algorithms if a key is pressed
		this.graph.setStopped(false);
	}

	public void keyReleased(KeyEvent arg0) {
		
	}

	public void keyTyped(KeyEvent arg0) {
		
	}

	public void mouseClicked(MouseEvent e) {
		// resume computation of graph algorithms if the mouse is clicked
		this.graph.setStopped(false);
	}

	public void mouseEntered(MouseEvent e) {
	}

	public void mouseExited(MouseEvent e) {
	}

	public void mousePressed(MouseEvent e) {
	}

	public void mouseReleased(MouseEvent e) {
	}

}

