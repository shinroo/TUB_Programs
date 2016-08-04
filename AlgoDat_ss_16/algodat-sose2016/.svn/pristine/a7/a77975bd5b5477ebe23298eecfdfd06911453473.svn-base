/**
 * 
 * @author Algorithm and Datastructures Team SS2016
 * @version 1.0  
 * 
 */
import java.lang.RuntimeException;
public class MyHashMap {
	
	/**
	 * Use this array to store the values
	 * DO NOT MODIFY OR REMOVE THIS Attribute. Our tests rely on it.
	 */
	Student[] array;
	
	/**
	 * Tries inserting a Student into the hash map.
	 * Throws RunTimeException, if the student is already in the table or the table is full.
	 */
	public void add(Student s){
		//check if student is already in table
		if (contains(s)){
			throw new RuntimeException("Student is already in the table!");
		}
		else {
			//check if table is full
			boolean full = true;
			for (Student curr: array){
				if (curr == null){
					full = false;
					break;
				}
			}
			if (full){
				throw new RuntimeException("Hash table is full!");
			}
			else {
				//table is not full and student is not in table
				//determine where to copy student
				int pos = hashFunction(s);
				if (array[pos] == null){
					array[pos] = s;
				}
				else {
					pos = (pos + 1) % array.length;
					while (array[pos] != null){
						pos = (pos + 1) % array.length;
					}
					array[pos] = s;
				}
			}
		}
	}
	
	/**
	 * Try removing a Student from the hash table.
	 * You should use the same implementation for remove discussed in the tutorial.
	 * You should NOT use the lazy deletion strategy (adding a special flag key indicating a deleted key)
	 * See https://en.wikipedia.org/wiki/Linear_probing#Deletion for more details about the algorithm!
	 * Throw RunTimeException if the hash table contained the Student
	 */
	public void remove(Student s){
		if (contains(s)){
			//find index of student
			int initialHash = hashFunction(s);
			int hashStart = 0;
			if (array[initialHash].equals(s)){
				//student found at initial hash
				hashStart = initialHash;
				Student temp = null;
				array[initialHash] = null;
				for (hashStart = (hashStart++) % array.length; array[hashStart] != null; hashStart = (hashStart++) % array.length){
					temp = array[hashStart];
					array[hashStart] = null;
					add(temp);
				}
			}
			else {
				//look for student in other possible hash locations
				boolean error = true;
				for (int i = (initialHash+1) % array.length; i != initialHash; i = (i++) % array.length){
					if (array[i].equals(s)){
						error = false;
						array[i] = null;
						hashStart = i;
						break;
					}
				}
				if (error){
					throw new RuntimeException("Student couldn't be removed from the table");
				} else {
					//re-add all remaining elements
					Student temp = null;
					for (hashStart = (hashStart++) % array.length; array[hashStart] != null; hashStart = (hashStart++) % array.length){
						temp = array[hashStart];
						array[hashStart] = null;
						add(temp);
					}
				}
			}
		} else {
			throw new RuntimeException("Student not in hash table!");
		}
		
	}
	
	
	/**
	 * Returns true, if the hash table contains the given Student
	 */
	public boolean contains(Student s){
		boolean test = false;
		for (Student curr: array){
			if ((curr != null) && (curr.equals(s))){
				test = true;
				break;
			}
		}
		return test;
	}
	
	/**
	 * @param h Hashvalue to search for
	 * @return Number of Student in the hash table that have the hashvalue h
	 */
	public int getNumberStudentsWithHashvalue(int h){
		int n = 0;
		//TODO: Your Code here.
		for (Student curr: array){
			if ((curr != null) && (hashFunction(curr) == h)){
				n++;
			}
		}
		return n;
	}
	
	/**
	 * Doubles the size of the hash table. Recomputes the position of all elements using the
	 * new function.
	 */
	public void resize(){
		int size = array.length;
		Student[] array2;
		Student[] temp = new Student[array.length];
		temp = array;
		array2 = new Student[(size)*2];
		//initialise new array
		for (int i = 0; i < array2.length; i++){
			array2[i] = null;
		}
		
		setArray(array2);
		//insert into new array
		for (int j = 0; j < temp.length; j++) {
			if (temp[j] != null){
				add(temp[j]);
			}
		}
	}

	/**
	 * This method return the array in which the strings are stored.
	 * DO NOT MODIFY OR REMOVE THIS METHOD. Our tests rely on it.
	 */
	public Student[] getArray(){
		return array;
	}
	
	/**
	 * DO NOT MODIFY OR REMOVE THIS METHOD. Our tests rely on it.
	 */
	public void setArray(Student[] array){
		this.array = array;
	}

	/**
	 * Runs the hash function for Student s (dependent on the size of the hash table)
	 * DO NOT MODIFY OR REMOVE THIS METHOD. Our tests rely on it.
	 * @param s Student
	 * @return the hash value for a student. (The position where it would be saved given no collisions)
	 */
	public int hashFunction(Student s){
		int hashvalue = Math.abs(s.hashCode()) % array.length;
		return hashvalue;
	}
	
	/**
	 * Constructor to initialize the hash with a given capacity
	 * DO NOT MODIFY OR REMOVE THIS METHOD. Our tests rely on it.
	 */
	public MyHashMap(int capacity){
		array = new Student[capacity];
	}

}

