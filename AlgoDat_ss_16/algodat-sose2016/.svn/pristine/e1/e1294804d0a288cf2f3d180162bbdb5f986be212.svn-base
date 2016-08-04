import static org.junit.Assert.* ;
import org.junit.* ;


/**
 * Test-Class for Shape
 * 
 * 
 * @author damien
 *
 */
public class ShapeTest {

	@Test
	public void testSmaller() {
		HexagonShape r1 = new HexagonShape(1);
		RectangleShape r2 = new RectangleShape(.5,1);
		assertTrue("compareTo gives incorrect output for smaller input", r1.compareTo(r2) < 0);
	}

	@Test
	public void testEqual() {
		RectangleShape r1 = new RectangleShape(1,1);
		RectangleShape r2 = new RectangleShape(.5,1);
		assertTrue("compareTo gives incorrect output for same-sized input", r1.compareTo(r2) == 0);
	}
	
	@Test
	public void testLarger() {
		RectangleShape r1 = new RectangleShape(1,1);
		RectangleShape r2 = new RectangleShape(.5,1);
		assertTrue("compareTo gives incorrect output for larger input", r1.compareTo(r2) > 0);

	}
}



