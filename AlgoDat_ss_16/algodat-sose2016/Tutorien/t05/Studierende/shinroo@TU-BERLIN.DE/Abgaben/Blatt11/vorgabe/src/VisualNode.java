import java.awt.Point;

/**
 * A node object. The object contains the position and the label of the
 * node.
 * 
 * @author Joerg Schneider <komm@cs.tu-berlin.de>
 */
public class VisualNode extends Node{
	Point position;
	/**
	 * Creates a new node
	 */
	public VisualNode(int label) {
		super(label);
	}
	
	public int getLabel(){
		return getID();
	}
	
	@Override
	public int hashCode() {
		return getID();
	}
	
	@Override
	public boolean equals(Object o) {
		return super.equals(o);
	}
}

