import java.util.ArrayList;
import java.util.LinkedList;

/**
 * The class <code>Node</code> implements a node.
 * 
 * @author Marc Alexa - modified by MPGI2 - Tutors
 * @version 1.2
 */
public class CellNode extends Node {

	int i; //row index of cell
	int j; //column index cell
	
	public double distanceToGoalEstimate;
	

	//class function to compute the node id from indices
	static int MAXROWS=10000;
	public static int computeID(int i, int j) {
		return i + j * MAXROWS; //try to assign a unique name
	}
	
	/**
	 * Constructor that creates specified node.
	 * 
	 * @param name
	 *            the drawn value in visualization
	 */
	public CellNode(int name) {		
		super(name);		
		//neighbourcellnodes = new LinkedList<CellNode>();		
		this.i = -1;
		this.j = -1;
		this.distanceToGoalEstimate = Double.NaN;
	}

	public CellNode(int i, int j) {
		super(-1);
		if (i >= MAXROWS) throw new RuntimeException("row index is too large! (bigger than "+ MAXROWS + ")");
		this.id = computeID(i,j);
		//neighbourcellnodes = new LinkedList<CellNode>();
		this.i = i;
		this.j = j;
		this.distanceToGoalEstimate = Double.NaN;
	}
	
	public int getRow() {
		return i;
	}
	
	public int getColumn() {
		return j;
	}


	@Override
	public String toString(){
		return "Node("+i+","+j+"]";
//		return "Node(position=["+i+","+j+"] edges="+edges+")";
	}

}

