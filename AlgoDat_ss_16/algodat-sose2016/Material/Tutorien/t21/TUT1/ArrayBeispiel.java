import java.util.ArrayList;
import java.util.ListIterator;
/**
 * @author Marcin
 *
 */
public class ArrayBeispiel {
	public static void main (String[] args) {
		ArrayList<Integer> a = new ArrayList<Integer>();
		a.add(10);
		a.add(20);
		a.add(32);
		a.add(2);
		
		System.out.println("Value of no. 1+3 is " + (a.get(0) + a.get(2)) + " test");
		System.out.println(a.size());
		
	}
}
