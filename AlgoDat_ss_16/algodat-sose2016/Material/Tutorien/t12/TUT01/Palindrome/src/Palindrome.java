/**
 * This class checks palindromes
 * @author RBO
 *
 */
public class Palindrome {
	/**
	 * Returns true for input strings that are palindromes.
	 * A palindrome is a sentence that reads the same forwards or backwards.
	 * 
	 * @param s the input string
	 * @return true for palindromes 
	 */
	public static boolean isPalindrome(String s){

		int i = 0;
		int j = s.length() - 1;
		do
		{			
			if(s.charAt(i) != s.charAt(j)){
				return false;
			}
			
			i++;
			j--;
		}while(i<j);
 
		return true;
	}

}









//Solution for empty strings
//if(s==null||s.length()==0) return false;

//Solution capitalization
//s = s.toLowerCase();
//s = s.replaceAll("[^a-z]", "");
