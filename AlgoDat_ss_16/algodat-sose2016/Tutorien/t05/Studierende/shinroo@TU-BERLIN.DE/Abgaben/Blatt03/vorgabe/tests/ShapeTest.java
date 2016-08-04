import static org.junit.Assert.* ;

import org.junit.Test;


/**
 * Test-Class for Shape
 * 
 * 
 * @author damien
 *
 */
public class ShapeTest {
	
	@Test()
	public void testSmaller() {
		RectangleShape r1 = new RectangleShape(1,1);
		RectangleShape r2 = new RectangleShape(.5,1);
		try {
			assertTrue("compareTo gives incorrect output for smaller input", r1.compareTo(r2) < 0);
		} catch (NullPointerException e) {
			fail("smaller input to compareTo provokes exception");
		}
	}

	@Test()
	public void testEqual() {
		RectangleShape r1 = new RectangleShape(1,1);
		RectangleShape r3 = new RectangleShape(1,1);
		try {
			assertTrue("compareTo gives incorrect output for same-sized input", r1.compareTo(r3) == 0);
		} catch (NullPointerException e) {
			fail("same-sized input to compareTo provokes exception");
		}
	}
	
	@Test()
	public void testLarger() {
		RectangleShape r1 = new RectangleShape(1,1);
		RectangleShape r4 = new RectangleShape(2,1);
		try {
			assertTrue("compareTo gives incorrect output for larger input", r1.compareTo(r4) > 0);
		} catch (NullPointerException e) {
			fail("larger input to compareTo provokes exception");
		}
	}
}