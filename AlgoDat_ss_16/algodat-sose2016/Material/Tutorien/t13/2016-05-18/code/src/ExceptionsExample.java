import java.util.ArrayList;
import java.util.ListIterator;

public class ExceptionsExample {

	/**
	 * This is a method from exercise 1 but it throws different kinds of
	 * Exceptions this time depending on the the input
	 *
	 * @param arr
	 *            array to be tested
	 * @param divisor
	 *            number by which all elements of the given array should be
	 *            divisible
	 * @return true if all elements are divisible by divisor
	 * @throws EmptyArrayException,
	 *             ArithmeticException
	 * @throws Exception
	 *             throws ArithmeticException for zero divisor,
	 *             EmptyArrayException for empty Arrays Exception for null array
	 */
	public static boolean allDivisibleBy(ArrayList<Integer> arr, int divisor)
			throws EmptyArrayException, ArithmeticException {

		// This line will throw a NullpointerException
		int s = arr.size();

		if (divisor == 0) {
			// This is an exception type provided by java
			throw new ArithmeticException("Division by Zero");
		}

		if (arr.size() == 0) {
			// This is our own self-made exception type
			throw new EmptyArrayException("The array is empty");
		}

		ListIterator<Integer> i = arr.listIterator();
		while (i.hasNext()) {
			if ((i.next() % divisor) != 0)
				return false;
		}

		return true;
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// ArrayList<Integer> arr = new ArrayList<Integer>();
		ArrayList<Integer> arr = null;

		try {
			allDivisibleBy(arr, 1);
		} catch (ArithmeticException e) {
			System.out.println("division by zero");
			e.printStackTrace();
		} catch (EmptyArrayException e) {
			System.out.println("empty array");
			e.printStackTrace();
		} catch (NullPointerException e) {
			System.out.println("array is null");
			e.printStackTrace();
		} catch (Exception e) {
		}

		System.out.println("bla");

	}

}
