import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;

import org.junit.Before;
import org.junit.Test;


public class ArrayCheckTest {
	// NOTTODO DIESE ZEILE BITTE NICHT VERAENDERN!!
	// TODO Fuehrt eure Tests auf diesem ArrayCheck-Objekt aus!
	// Ansonsten kann eure Abgabe moeglicherweise nicht
	// gewertet werden.
	public ArrayCheck ArrayCheck = new ArrayCheck();
	
	@Test(timeout = 1000)
	public void testAllDivisibleBy() {
		// TODO Schreibt verschiedene Testfaelle, die unterschiedliche Arrays
		// anlegen und an testAllDivisibleBy uebergeben.
		// Testet auch randfaelle wie z.B. leere arrays.
		
		//Variable declarations
		ArrayList<Integer> dummyArray = new ArrayList<Integer>();
		int dummyInt;
		
		//Test for empty array, expect false
		//System.out.println("Checking for empty array...");
		dummyArray.clear();
		dummyInt = 10;
		assertFalse("Empty array test failed.",ArrayCheck.allDivisibleBy(dummyArray, dummyInt));
		
		//Test for 0 divisor, expect false
		//System.out.println("Checking for 0 divisor...");
		for (int i = 0; i < 10; i++) {
			dummyArray.add(i);
		}
		dummyInt = 0;
		assertFalse("0 divisor test failed.",ArrayCheck.allDivisibleBy(dummyArray, dummyInt));
		
		// dummyArray 10 elements, dummyInt 1, expect true
		//System.out.println("Checking array with 10 elements, divisor 1...");
		dummyArray.clear();
		for (int i = 0; i < 10; i++) {
			dummyArray.add(i);
		}
		dummyInt = 1;
		assertTrue("Test failed.",ArrayCheck.allDivisibleBy(dummyArray, dummyInt));
		
		// dummyArray 20 elements, dummyInt 3, expect true
		//System.out.println("Checking array with 20 elements, divisor 3...");
		dummyArray.clear();
		for (int i = 0; i < 20; i++) {
			dummyArray.add(i*3);
		}
		dummyInt = 3;
		assertTrue("Test failed.",ArrayCheck.allDivisibleBy(dummyArray, dummyInt));
		
		// dummyArray 30 elements, dummyInt 2, expect false
		//System.out.println("Checking array with 30 elements, divisor 3...");
		dummyArray.clear();
		for (int i = 0; i < 30; i++) {
			dummyArray.add(i);
		}
		dummyInt = 2;
		assertFalse("Test failed.",ArrayCheck.allDivisibleBy(dummyArray, dummyInt));
		
		// dummyArray 20000 elements, dummyInt 10, expect false
		//System.out.println("Checking array with 20000 elements, divisor 10...");
		dummyArray.clear();
		for (int i = 0; i < 20000; i++) {
			dummyArray.add(i);
		}
		dummyInt = 10;
		assertFalse("Test failed.",ArrayCheck.allDivisibleBy(dummyArray, dummyInt));
				
		//Test for null array, expect false
		//System.out.println("Checking for null array...");
		dummyArray = null;
		dummyInt = 10;
		assertFalse("Null array test failed.",ArrayCheck.allDivisibleBy(dummyArray, dummyInt));
		
	}   

	@Test(timeout = 1000)
	public void testIsFibonacci() {
		// TODO Schreibt verschiedene Testfaelle, fuer testIsFibonacci.
		
		//Variable declarations
		ArrayList<Integer> dummyArray = new ArrayList<Integer>();
		
		//Test for empty array, expect false
		//System.out.println("Checking for empty array...");
		dummyArray.clear();
		assertFalse("Empty array test failed.",ArrayCheck.isFibonacci(dummyArray));
		
		//Test for array of length 2, expect false
		//System.out.println("Checking with array of length 2...");
		dummyArray.add(0);
		dummyArray.add(1);
		assertFalse("Small array test failed.",ArrayCheck.isFibonacci(dummyArray));
		
		//Test with various non fibonacci sequences, expect false
		
		//System.out.println("Testing with sequence 1,1,2,7...");
		dummyArray.clear();
		dummyArray.add(1);
		dummyArray.add(1);
		dummyArray.add(2);
		dummyArray.add(7);
		assertFalse("Test with sequence 1,1,2,7 failed.",ArrayCheck.isFibonacci(dummyArray));
		
		//System.out.println("Testing with sequence 0,0,1,1...");
		dummyArray.clear();
		dummyArray.add(0);
		dummyArray.add(0);
		dummyArray.add(1);
		dummyArray.add(1);
		assertFalse("Test with sequence 0,0,1,1 failed.",ArrayCheck.isFibonacci(dummyArray));
		
		//System.out.println("Testing with sequence 1,1,2,3,5,8,14...");
		dummyArray.clear();
		dummyArray.add(1);
		dummyArray.add(1);
		dummyArray.add(2);
		dummyArray.add(3);
		dummyArray.add(5);
		dummyArray.add(8);
		dummyArray.add(14);
		assertFalse("Test with sequence 1,1,2,3,5,8,14 failed.",ArrayCheck.isFibonacci(dummyArray));
		
		//System.out.println("Testing with sequence 1,1,3...");
		dummyArray.clear();
		dummyArray.add(1);
		dummyArray.add(1);
		dummyArray.add(3);
		assertFalse("Test with sequence 1,1,3 failed.",ArrayCheck.isFibonacci(dummyArray));
		
		//System.out.println("Testing with sequence 13,21,35...");
		dummyArray.clear();
		dummyArray.add(13);
		dummyArray.add(21);
		dummyArray.add(35);
		assertFalse("Test with sequence 13,21,35 failed.",ArrayCheck.isFibonacci(dummyArray));

		//System.out.println("Testing with 0 to 99...");
		dummyArray.clear();
		for (int i = 0; i < 100; i++) {
			dummyArray.add(i);
		}
		assertFalse("Test with 0 to 99 failed.",ArrayCheck.isFibonacci(dummyArray));
		
		//System.out.println("Testing with multiples of 3...");
		dummyArray.clear();
		for (int i = 0; i < 100; i++) {
			dummyArray.add(i * 3);
		}
		assertFalse("Test with multiples of 3 failed.",ArrayCheck.isFibonacci(dummyArray));
		
		//Test with various fibonacci sequences, expect true
		
		//System.out.println("Testing with sequence 1,1,2...");
		dummyArray.clear();
		dummyArray.add(1);
		dummyArray.add(1);
		dummyArray.add(2);
		assertTrue("Test with sequence 1,1,2 failed.",ArrayCheck.isFibonacci(dummyArray));
		
		//System.out.println("Testing with sequence 0,1,1...");
		dummyArray.clear();
		dummyArray.add(0);
		dummyArray.add(1);
		dummyArray.add(1);
		assertTrue("Test with sequence 0,1,1 failed.",ArrayCheck.isFibonacci(dummyArray));

		//System.out.println("Testing with sequence 1,2,3...");
		dummyArray.clear();
		dummyArray.add(1);
		dummyArray.add(2);
		dummyArray.add(3);
		assertTrue("Test with sequence 1,2,3 failed.",ArrayCheck.isFibonacci(dummyArray));
		
		//System.out.println("Testing with sequence 3,5,8...");
		dummyArray.clear();
		dummyArray.add(3);
		dummyArray.add(5);
		dummyArray.add(8);
		assertTrue("Test with sequence 3,5,8 failed.",ArrayCheck.isFibonacci(dummyArray));
		
		//System.out.println("Testing with sequence 13,21,34...");
		dummyArray.clear();
		dummyArray.add(13);
		dummyArray.add(21);
		dummyArray.add(34);
		assertTrue("Test with sequence 13,21,34 failed.",ArrayCheck.isFibonacci(dummyArray));
		
		//System.out.println("Testing with sequence 0,1,1,2,3,5,8,13,21,34...");
		dummyArray.clear();
		dummyArray.add(0);
		dummyArray.add(1);
		dummyArray.add(1);
		dummyArray.add(2);
		dummyArray.add(3);
		dummyArray.add(5);
		dummyArray.add(8);
		dummyArray.add(13);
		dummyArray.add(21);
		dummyArray.add(34);
		assertTrue("Test with sequence 0,1,1,2,3,5,8,13,21,34 failed.",ArrayCheck.isFibonacci(dummyArray));
		
		//Test for null array, expect false
		//System.out.println("Checking for null array...");
		//dummyArray = null;
		//assertFalse("Null array test failed.",ArrayCheck.isFibonacci(dummyArray));
		
	}


}

