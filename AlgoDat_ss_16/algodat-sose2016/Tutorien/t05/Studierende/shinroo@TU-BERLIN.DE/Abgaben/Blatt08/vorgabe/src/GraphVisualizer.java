import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.Point;
import java.awt.Polygon;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.image.BufferedImage;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Vector;

import javax.swing.JPanel;

/**
 * This class provides the methods to draw nodes and edges on the screen. Use
 * the class VisualGraph to display your graph data structure.
 * 
 * @author Ilya Shabanov, modified by AlgoDat-Team
 */
public class GraphVisualizer extends JPanel implements ComponentListener{

	/** The underlying graph data structure */
	private Graph graph;

	private static final long serialVersionUID = 7538698556836996121L;

	/** how much the edged are bowed */
	private static final int EDGE_BOW = 30;
	
	/** how much the edged are bowed */
	private static final int ARROW_SIZE = 15;

	/** Font used for edge labels */
	private static final Font EDGE_FONT = new Font("SansSerif", Font.BOLD, 28);

	/** Font used for node labels */
	private static final Font NODE_FONT = new Font("Serif", Font.BOLD, 36);

	/** The colors to highlight some nodes or edges */
	static final Color[] COLORS = new Color[] { new Color(255, 255, 255), // 0:white
																	// DEFAULT_COLOR
		new Color(192, 192, 192), // 1:gray
		new Color(0, 0, 0), // 2:black
			new Color(0, 128, 0), // 3:green
			new Color(234, 123, 2), // 4:orange
			new Color(250, 0, 0) // 5:red
	};

	/** default color */
	static final int DEFAULT_COLOR = 0;

	/** size of nodees */
	private final double NODE_GFX_SIZE = 60;

	/** color of labels */
//	private final Color LABEL_COLOR = new Color(0x00aa00);
	private final Color LABEL_COLOR = new Color(0x000000);

	/** size of the paintable area */
	private Dimension canvasSize;

	/** main buffer for drawing */
	private Image imgMain;

	/** object to perform the draw operation on */
	private Graphics graphicsMain;

	/** Node positions and node marks */
	private Vector<VisualNode> nodes = new Vector<VisualNode>();

	/** Edges with adjacent nodes and labels */
	private Map<VisualNode, Map<VisualNode, VisualEdge>> edges = new HashMap<VisualNode, Map<VisualNode, VisualEdge>>();

	/**
	 * create an area to paint the graph on.
	 * 
	 * @param d
	 *            the initial dimension of this area. The graph will be repaint,
	 *            if the dimension changes.
	 */
	public GraphVisualizer(Graph theGraph, Dimension d) {
		this.graph = theGraph;
		canvasSize = d;
		// react on resizing
		addComponentListener(this);
	}

	public void componentResized(ComponentEvent event) {
		canvasSize = getSize();

		if (!nodes.isEmpty()) {
			calculateNodePositions();
			drawGraphImage();
		}
	}

	public void componentMoved(ComponentEvent arg0) {
		// do nothing
	}

	public void componentShown(ComponentEvent arg0) {
		// do nothing
	}

	public void componentHidden(ComponentEvent arg0) {
		// do nothing
	}

	/**
	 * Calculates the position of the nodes.
	 */
	private void calculateNodePositions() {

		double maxRadius = canvasSize.width < canvasSize.height ? canvasSize.width
				: canvasSize.height;
		maxRadius /= 2;

		double angle = 2 * Math.PI / nodes.size();
		double radius = 0.9 * maxRadius;

		int i = 0;
		for (VisualNode node : nodes) {
			node.position = new Point(canvasSize.width / 2
					+ (int) (Math.cos(angle * i) * radius), canvasSize.height
					/ 2 + (int) (Math.sin(angle * i) * radius));
			i++;
		}
	}

	/** Add a node with the given label */
	public VisualNode addNode(int label) {
		VisualNode node = new VisualNode(label);
		nodes.add(node);

		calculateNodePositions();

		drawGraphImage();

		return node;
	}


	/** Adds an edge between the given nodes */
	public void addEdge(VisualNode start, VisualNode target, int weight) {
		Map<VisualNode, VisualEdge> targetNodes = edges.get(start);
		if (targetNodes == null) {
			targetNodes = new HashMap<VisualNode, VisualEdge>();
			edges.put(start, targetNodes);
		}
		VisualEdge edge = targetNodes.get(target);
		if (edge == null) {
			edge = new VisualEdge(start, target, weight);
			targetNodes.put(target, edge);
		} else {
			edge.weight = weight;
		}

		drawGraphImage();
	}

	/**
	 * Removes the node from the graph. All edges from and to the node will be
	 * removed, too.
	 */
	public void deleteNode(VisualNode node) {
		nodes.remove(node);
		edges.remove(node);
		for (VisualNode actNode : nodes) {
			Map<VisualNode, VisualEdge> targetNodes = edges.get(actNode);
			if (targetNodes != null) {
				targetNodes.remove(node);
			}
		}

		calculateNodePositions();

		drawGraphImage();
	}

	/** Removes the edge between the given nodes. */
	public void deleteEdge(VisualNode start, VisualNode target) {
		Map<VisualNode, VisualEdge> targetNodes = edges.get(start);
		if (targetNodes != null) {
			targetNodes.remove(target);
		}

		drawGraphImage();
	}

	/**
	 * Do the drawing of all nodes and edges and store it in a buffer. This
	 * method is called each time the graph changes.
	 */
	synchronized private void drawGraphImage() {
		// Main Gfx buffer
		imgMain = new BufferedImage(canvasSize.width, canvasSize.height,
				BufferedImage.TYPE_INT_ARGB);
		graphicsMain = imgMain.getGraphics();
		List<VisualEdge> edgesToDelete = new LinkedList<VisualEdge>();
		List<VisualNode> nodesToDelete = new LinkedList<VisualNode>();
		// set background
//		graphicsMain.setColor(new Color(0x000000));
//		graphicsMain.fillRect(0, 0, this.getWidth(), this.getHeight());
		
		for (VisualNode node : nodes) {
			
			if(graph.getNodes().contains(node)){
			// draw current Node
				drawNode(graphicsMain, node.position, "" + node.getLabel(), this.graph.getColorOfNode(node.getLabel()));
			
				Map<VisualNode, VisualEdge> adjacentNodes = edges.get(node);
				if (adjacentNodes != null) {
					for (VisualEdge edge : adjacentNodes.values()) {
						// draw edge and it's weight
						if(this.graph.isConnected(node.getID(), edge.target.getID())){
							drawEdge(graphicsMain, edge.start.position,
									edge.target.position, this.graph.getColorOfEdge(edge.start.getID(), edge.target.getID()), "" + edge.weight);
						}else{
							edgesToDelete.add(edge);
						}
					}
				}
			}else{
				nodesToDelete.add(node);
			}

		}

		repaint();
		for(VisualEdge e:edgesToDelete){
			deleteEdge(e.start, e.target);
		}
		
		for(VisualNode n:nodesToDelete){
			deleteNode(n);
		}
	}

	/** draw a bowed line with an arrow head and a label. */
	private void drawEdge(Graphics g, Point sIn, Point eIn, int color, String label) {
		
		// calculate new start / end point, so the arrows end outside the nodes		
		int deltaX = Math.abs(sIn.x - eIn.x);
		int deltaY = Math.abs(sIn.y - eIn.y);
		double hypo = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
		double alphaS = deltaY / hypo;
		
		int deltaX_circle = (int) (Math.cos(alphaS) * (NODE_GFX_SIZE/2)); 
		int deltaY_circle = (int) (Math.sin(alphaS) * (NODE_GFX_SIZE/2)); 		
		if(sIn.x > eIn.x) deltaX_circle *= -1;
		if(sIn.y > eIn.y ) deltaY_circle *= -1;
		Point s = new Point(sIn.x + deltaX_circle, sIn.y + deltaY_circle);

		double alphaE = deltaX / hypo;
		
		deltaX_circle = (int) (Math.sin(alphaE) * (NODE_GFX_SIZE/2)); 
		deltaY_circle = (int) (Math.cos(alphaE) * (NODE_GFX_SIZE/2)); 
		if(eIn.x > sIn.x) deltaX_circle *= -1;
		if(eIn.y > sIn.y ) deltaY_circle *= -1;
		Point e = new Point(eIn.x + deltaX_circle, eIn.y + deltaY_circle);
		
		double f = EDGE_BOW
				/ Math.sqrt((s.x - e.x) * (s.x - e.x) + (s.y - e.y)
						* (s.y - e.y));

		// bow point to visualize directed edges
		int x = (s.x + e.x) / 2 - (int) ((s.y - e.y) * f);
		int y = (s.y + e.y) / 2 + (int) ((s.x - e.x) * f);
		Point center = new Point(x, y);

		// arrow
		int xArray[] = new int[3];
		int yArray[] = new int[3];

		xArray[0] = e.x;
		yArray[0] = e.y;

		f = ARROW_SIZE
				/ Math.sqrt((center.x - e.x) * (center.x - e.x)
						+ (center.y - e.y) * (center.y - e.y));

		xArray[1] = e.x + (int) ((center.x - e.x) * f * 2)
				- (int) ((center.y - e.y) * f / 2);
		yArray[1] = e.y + (int) ((center.y - e.y) * f * 2)
				+ (int) ((center.x - e.x) * f / 2);

		xArray[2] = e.x + (int) ((center.x - e.x) * f * 2)
				+ (int) ((center.y - e.y) * f / 2);
		yArray[2] = e.y + (int) ((center.y - e.y) * f * 2)
				- (int) ((center.x - e.x) * f / 2);

		// draw		
		g.setColor(COLORS[color]);
		drawLine(g, s, center, color);
		drawLine(g, center, e, color);

		g.fillPolygon(new Polygon(xArray, yArray, 3));

		drawEdgeLabel(g, center, label, color);
	}

	/**
	 * draw a line. The line will be drawn bold if the color is not the default
	 * color
	 */
	private void drawLine(Graphics g, Point s, Point e, int color) {

		g.setColor(new Color(0, 0, 0));
		g.drawLine(s.x, s.y + 1, e.x, e.y + 1);
		g.drawLine(s.x - 1, s.y, e.x - 1, e.y);
		g.drawLine(s.x + 1, s.y, e.x + 1, e.y);
		g.drawLine(s.x, s.y - 1, e.x, e.y - 1);

	}

	/** draw the edge label */
	private void drawEdgeLabel(Graphics g, Point center, String label, int color) {


		int wX = center.x;
		int wY = center.y;

		wX -= 4;
		wY += 4;

		g.setFont(EDGE_FONT);

		if (label.length() >= 2){
			g.setColor(new Color(0, 0, 0));
			g.fillRect(wX -4, wY - 13, 44, 26);
			g.setColor(COLORS[color]);
			g.fillRect(wX - 2, wY - 11, 40, 22);
		}else{
			g.setColor(new Color(0, 0, 0));
			g.fillRect(wX -4, wY - 13, 36, 26);
			g.setColor(COLORS[color]);
			g.fillRect(wX - 2, wY - 11, 32, 22);
		}
		if(color == 2){
			g.setColor(new Color(255, 255, 255));
		}else{
			g.setColor(LABEL_COLOR);
		}
		g.drawString(label, wX + 5, wY + 10);
		
		
	}

	/** draw a node with a label */
	private void drawNode(Graphics g, Point c, String label, int color) {
		
		int x = c.x - (int) (NODE_GFX_SIZE / 2);
		int y = c.y	- (int) (NODE_GFX_SIZE / 2);
		int width = (int) NODE_GFX_SIZE;
		int height = (int) NODE_GFX_SIZE;
		g.setColor(new Color(0, 0, 0));
		g.fillOval(x, y, width , height);
		g.setColor(COLORS[color]);
		g.fillOval(x+4, y+4, width -8, height -8);

		if(color == 2){
			g.setColor(new Color(255, 255, 255));
		}else{
			g.setColor(LABEL_COLOR);
		}
		
		g.setFont(NODE_FONT);
		g.drawString(label, c.x - (int) (NODE_GFX_SIZE / 2) + 10, c.y + (int) (NODE_GFX_SIZE / 4));
	}

	/**
	 * This method is called each time the window is refreshed. The content of
	 * the buffer is copied on the screen.
	 * 
	 * @see #drawGraphImage()
	 */
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		if (imgMain != null)
			g.drawImage(imgMain, 0, 0, null);

	}
	
	/**
	 * repaints the graph
	 */
	public void update(){
		this.drawGraphImage();
	}

	/** deletes all nodes and edges */
	public void clear(){
		this.edges.clear();
		this.nodes.clear();
	}


}

