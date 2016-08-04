import java.util.ArrayList;
import java.util.ListIterator;


/**
 * This class implements three methods. These test an array on a few
 * characteristics.
 *
 * @author AlgoDat-Tutoren
 *
 */
public class ArrayCheck {
	/**
	 * Tests all elements of the given array,
     * if they are divisible by the given divisor.
     *
     * @param arr
     *            array to be tested
     * @param divisor
     * 				number by which all elements of the given array should be divisible
     * @return true if all elements are divisible by divisor
     */
    public boolean allDivisibleBy(ArrayList<Integer> arr, int divisor) {
        //check for null array
    	if (arr == null) {
    		return false;
    	}
    	
    	//check for 0 divisor
    	if (divisor == 0) {
    		return false;
    	}
    	
    	//check for empty array or 0 size array
    	if ((arr.isEmpty()) || (arr.size() == 0)) {
    		return false;
    	}
    	
    	//check for divisibility by 1
    	if (divisor == 1){
    		return true;
    	}
    	
    	//check divisibility
    	int check;
    	ListIterator<Integer> i = arr.listIterator();
    	while (i.hasNext()) {
    		check = i.next();
    		if ( check % divisor != 0 ) {
    			if (check != 0) {
    				return false;
    			}
    		}
    	}
        return true;
    }

    /**
     * Tests if the given array is a part of
     * the fibonacci sequence.
     *
     * @param arr
     *            array to be tested
     * @return true if the elements are part of
     *         the fibonacci sequence
     */
    public boolean isFibonacci(ArrayList<Integer> arr) {
        
    	if ((arr.isEmpty()) || (arr == null)) {
    		return false;
    	}
        
    	if (arr.size() < 3) {
    		return false;
    	}
    	
    	int fA = 0;
    	int fB = 0;
    	int temp = 0;
    	
    	ListIterator<Integer> fIt = arr.listIterator();
    	
    	//check for sequence starting 0,1,1...
    	if (arr.get(0) == 0) {
    		if (arr.get(1) != 1) {
    			return false;
    		} else {
    			fA = 0;
    			fB = 1;
    			
    			fIt.next();
    			fIt.next();
    			
    			while(fIt.hasNext()){
    				temp = fB;
    				fB += fA;
    				fA = temp;
    				
    				if (fIt.next() != fB){
    					return false;
    				}
    			}
    			return true;
    		}
    	}
    	
    	//check for sequences starting 1,1,2... or 1,2,3...
    	if (arr.get(0) == 1) {
    		//112
    		if (arr.get(1) == 1) {
    			fA = 1;
    			fB = 1;
    			
    			fIt.next();
    			fIt.next();
    			
    			while(fIt.hasNext()){
    				temp = fB;
    				fB += fA;
    				fA = temp;
    				
    				if (fIt.next() != fB){
    					return false;
    				}
    			}
    			return true;
    		}
    		//123
    		else if (arr.get(1) == 2) {
    			fA = 1;
    			fB = 2;
    			
    			fIt.next();
    			fIt.next();
    			
    			while(fIt.hasNext()){
    				temp = fB;
    				fB += fA;
    				fA = temp;
    				
    				if (fIt.next() != fB){
    					return false;
    				}
    			}
    			return true;
    		}
    		else {
    			return false;
    		}
    	}
    	
    	/*
    	 * if the program gets to here, check if the first value of the array is
    	 * a fibonacci value, calculate the previous fibonacci number and then check
    	 * the remaining values of the array, if the while loop is passed, return true
    	 */
    	
    	fA = 0;
    	fB = 1;
    	
    	while (fB < arr.get(0)) {
    		temp = fB;
			fB += fA;
			fA = temp;
    	}
    	
    	if (fB != arr.get(0)) {
    		return false;
    	}
    	
    	if (arr.get(1) != (fA + fB)) {
    		return false;
    	}
    	
    	fIt.next();
    	
    	while(fIt.hasNext()){
			temp = fB;
			fB += fA;
			fA = temp;
			
			if (fIt.next() != fB){
				return false;
			}
		}
    	
    	return true;
    }

}

