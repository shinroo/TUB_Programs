import org.junit.* ;
import static org.junit.Assert.* ;

class DataType {
	public int number;
	
	@Override
	public String toString() {
		return "<"+number+">";
	}

	@Override
	public boolean equals(Object a) {
		if (a instanceof DataType)
			return ((DataType)a).number == this.number;
		return false;
	}
}

/**
 * Bogus class that triggers an exception when somebody tries 
 * to convert it to a string
 *
 *
**/
class NotImplementedDataType {
	@Override
	public String toString() {
		throw new RuntimeException("I am evil.");
	}
}

/**
 * Test-Class for Pair
 * 
 * @author AlgoDat
 *
 */
public class PairTest {

	@Test()
	public void testPair() {
		DataType a = new DataType();
		DataType b = new DataType();
		a.number = 1;
		b.number = 20;

		Pair<DataType> pair = new Pair<DataType>(a, b);
		try {
			assertEquals("Constructor or getFirst() fails", a, pair.getFirst());
		} catch (NullPointerException e) {
			assertFalse("getFirst() provokes a NullPointerException", true);
		}
	}
	

	/************************************************************************/
	/************************************************************************/
	/************************************************************************/
	
	@Test()
	public void testPairExceptionHandling() {
		NotImplementedDataType  a1 = new NotImplementedDataType();
		NotImplementedDataType  a2 = new NotImplementedDataType();
		
		Pair<NotImplementedDataType> pair = new Pair<NotImplementedDataType>(a1, a2);
		
		String s = "";
		try {
			s = pair.toString();
		} catch (Exception e) {
			assertFalse("toString() does not handle the Exception: " + e.getMessage(), true);
		}
		// parse the string using RegEx
		String corr_s = "<not implemented>, <not implemented>";
		assertEquals("Array exception does not return failure.",corr_s, s);	}
}




