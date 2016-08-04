import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;


public class MyHashMapUnitTest {

	private MyHashMap hashmap;
	private Student s1, s2, s3, s4, s5, s6, s7, s8, s9;
	
	@Before
	public void setUp() throws Exception {
		hashmap = new MyHashMap(7);
		s1 = new Student(1, "Max", "Mustermann", "Informatik");
		s2 = new Student(2, "Annika", "Mustermann", "Mathematik");
		s3 = new Student(3, "Alex", "Mustermann", "Biologie");
		s4 = new Student(4, "Julie", "Mustermann", "TI");
		s5 = new Student(5, "Leila", "Mustermann", "Informatik");
		s6 = new Student(6, "Arne", "Mustermann", "Jura");
		s7 = new Student(7, "Bella", "Mustermann", "Physik");
		s8 = new Student(8, "Linus", "Mustermann", "Psychologie");
		s9 = new Student(9, "Albert", "Mustermann", "Informatik");
	}

	/**
	 * Adds an element to the hash table. Checks if it is in the array
	 */
	@Test(timeout=1000)
	public void testSimpleAdd() {
		hashmap.add(s1);
		Student[] array = hashmap.getArray();
		boolean success = false;
		for (int i = 0; i < array.length; i++) {
			Student s = array[i];
			if (s1.equals(s)){
				success = true;
				break;
			}
		}
		assertTrue("add() failed. Element not found in array", success);
	}

	/**
	 * Remove an element from a 
	 */
	@Test(timeout=1000)
	public void testRemove() {
		Student[] array = new Student[7];
		array[5] = s2;  // This is the correct hash position of this student
		hashmap.setArray(array);
		hashmap.remove(s2);
		// Check if the element has been removed
		boolean success = true;
		array = hashmap.getArray();
		for (int i = 0; i < array.length; i++) {
			Student s = array[i];
			if (s2.equals(s)){
				success = false;
				break;
			}
		}
		assertTrue("remove() failed. Element was not removed from hashtable", success);
	}

	@Test(timeout=1000)
	public void testContains() {
		Student[] array = new Student[7];
		array[5] = s2;  // This is the correct hash position of this student
		hashmap.setArray(array);
		assertTrue("contains() failed. Element was not found in the hashtable", hashmap.contains(s2));
	}
	
	/**
	 * Test getNumberStudentsWithHashvalue
	 */
	@Test(timeout=1000)
	public void testGetNumberStudentsWithHashvalue() {
		Student[] array = new Student[7];
		array[5] = s2;  // This is the correct hash position of this student
		array[6] = s4;
		array[0] = s5;
		array[3] = s1;
		array[4] = s3;
		hashmap.setArray(array);
		assertEquals("getNumberStudentsWithHashvalue() failed. Wrong number", 3, hashmap.getNumberStudentsWithHashvalue(5));
	}
	
	
	@Test(timeout=1000)
	public void testTrivialResize() {
		Student[] array = new Student[7];
		array[5] = s2;  // This is the correct hash position of this student
		array[6] = s4;
		array[0] = s5;
		array[3] = s1;
		array[4] = s3;
		hashmap.setArray(array);
		hashmap.resize();
		int newsize = hashmap.getArray().length;
		assertEquals("resize() failed; length of new hash table is not the double of the old one", 14, newsize);
	}
	
	
	
	/* The rest of these tests are not implemented. We include them 
	 * to give you an idea on the parts of your code that we will test for the correction.
	 * Note, that these is not the final list of tests that we will use
	 * We do recommend that you implement these tests. 
	 * But, YOU DO NOT HAVE TO IMPLEMENT THEM!
	 */
	
	/**
	 * Tests resize: Checks that each element is saved in the correct position after a resize operation
	 */
	@Test(timeout=1000)
	public void testResize(){
		fail("Not yes implemented");
	}
	
	/**
	 * Test remove an element saved in the wrong position of the hashtable.
	 * This test will set the array of MyHashTable with an array containing one element on the wrong index
	 * We then try removing these elements. This should not be possible.
	 */
	@Test(timeout=1000)
	public void testWrongRemove() {
		fail("Not yet implemented");
	}
	

	/**
	 * Add element to the hash table twice. Checks if an exception is thrown.
	 */
	@Test(timeout=1000)
	public void testAddElementTwice() {
		fail("Not yet implemented");
	}
	
	/**
	 * Add elements to the hashtable until it is full. Check if an exception is thrown
	 */
	@Test(timeout=1000)
	public void testAddUntilFull() {
		fail("Not yet implemented");
	}
	
	/**
	 * Add an element, then remove it. Check what contains return.
	 */
	@Test(timeout=1000)
	public void testAddRemoveContains() {
		fail("Not yet implemented");
	}
	
	/**
	 * Remove an element twice. Check for the exception.
	 */
	@Test(timeout=1000)
	public void testRemoveElementTwice() {
		fail("Not yet implemented");
	}
	
	
	

}


