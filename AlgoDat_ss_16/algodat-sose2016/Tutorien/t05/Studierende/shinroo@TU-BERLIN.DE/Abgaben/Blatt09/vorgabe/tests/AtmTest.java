import java.util.List;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;


public class AtmTest
{
	Atm atm;

	@Before 
	public void setup() {
		atm = new Atm(); //instantiates an Atm once before starting the tests
	}
	

	@Test
	public void testAtmNoMoney(){
		List<Integer> notes = atm.getDivision(0);
		int [] expected = {0,0,0,0,0,0};
		for (int i=0; i < expected.length; i++) {
			Assert.assertEquals(expected[i], notes.get(i).intValue());
		}		
	}

	@Test
	public void testAtm1(){
		List<Integer> notes = atm.getDivision(285);
		int [] expected = {1,0,1,1,1,1};
		for (int i=0; i < expected.length; i++) {
			Assert.assertEquals(expected[i], notes.get(i).intValue());
		}		
	}
	
	@Test
	public void testAtmLargeAmount(){
		List<Integer> notes = atm.getDivision(1000000);
		int [] expected = {5000,0,0,0,0,0};
		for (int i=0; i < expected.length; i++) {
			Assert.assertEquals(expected[i], notes.get(i).intValue());
		}		
	}
	
 }

