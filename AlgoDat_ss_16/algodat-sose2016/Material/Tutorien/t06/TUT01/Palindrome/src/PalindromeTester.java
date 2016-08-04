import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class PalindromeTester {

	@Test
	public void simpleTest() {
		assertEquals("Simple test 1", true, Palindrome.isPalindrome("uhu"));
		assertEquals("Simple test 2", false, Palindrome.isPalindrome("anba"));
		assertEquals("Simple test 3", true, Palindrome.isPalindrome("lagerregal"));
	}
}


















//@Test
//public void borderCaseTest() {
//	assertEquals("border case test 1", true, Palindrome.isPalindrome(""));
//}
//
//@Test
//public void capitalizationTest() {
//	assertEquals("capitalization test 1", true, Palindrome.isPalindrome("Anna"));
//	assertEquals("capitalization test 2", true, Palindrome.isPalindrome("A man, a plan, a canal: Panama"));		
//}
