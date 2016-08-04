import javax.swing.JFrame;

/**
 * This class displays a window on the screen.
 * It implements the comparable interface so that we can compare
 * objects of this class with shapes.
 */
public class Fenster extends JFrame implements Comparable<Shape> {

	/**
	 * Creates a Fenster with dimension 800 by 600
	 * and the title "Test"
	 */
	public Fenster() {
		this.setSize(800, 600);
		this.setTitle("Test");
		this.setVisible(true);
	}

	/**
	 * Compares the Fenster with Shape objects based
	 * on surface area.
	 */
	@Override
	public int compareTo(Shape o) {
		double otherArea = o.calculateArea();
		// TODO: Don't use hard coded dimensions
		double myArea = 800 * 600;
		double tol = 0.1;

		if (otherArea < myArea - tol) {
			return -1;
		} else if (otherArea > myArea + tol) {
			return 1;
		} else {
			return 0;
		}
	}

}
