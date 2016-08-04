/**
 * Code for playing with SimpleList.
 */
public class SimpleListTutorial {

	public static void main(String[] args) {
		// int list
		System.out.println("int list");
		int[] values = { 1, 2, 3, 4 };
		SimpleList list = new SimpleList(values);
		for (int i = 0; i < list.size(); ++i) {
			int value = list.get(i);
			System.out.println("index=" + i + ", value=" + value);
		}

		// Object list
		// Intention: We can use all types of objects without creating new
		// list classes. If we want to use methods of the subclasses, we
		// have to cast the objects.
		System.out.println("\nObject list");
		Object[] values2 = { "a", "bb", "ccc" };
		ObjectSimpleList list2 = new ObjectSimpleList(values2);
		for (int i = 0; i < list2.size(); ++i) {
			String value = (String) list2.get(i); // cast!
			System.out.println("index=" + i + ", value=" + value +
					", string length=" + value.length());
		}

		// We can mix the types, but that is probably not useful.
		System.out.println("\nObject list 2");
		Object[] values3 = { 42.42, "dddd", -3 };
		ObjectSimpleList list3 = new ObjectSimpleList(values3);
		for (int i = 0; i < list3.size(); ++i) {
			Object value = list3.get(i);
			if (value instanceof String) {
				// print the value and length of strings
				String stringValue = (String) value;
				System.out.println("index=" + i + ", value=" + stringValue +
						", string length=" + stringValue.length());
			} else {
				// print the value of other objects
				System.out.println("index=" + i + ", value=" + value);
			}
		}
		


		// We can use the generic list for all classes without casting.
		System.out.println("\nGeneric list");
		Integer[] values4 = { 1, 2, 3 }; // Integer is important here!
		GenericSimpleList<Integer> list4 = new GenericSimpleList<Integer>(values4);
		for (int i = 0; i < list4.size(); ++i) {
			int value = list4.get(i);
			System.out.println("index=" + i + ", value=" + value);
		}
		
		System.out.println("\nGeneric list 2");
		String[] values5 = { "a", "bb", "ccc" };
		GenericSimpleList<String> list5 = new GenericSimpleList<String>(values5);
		for (int i = 0; i < list5.size(); ++i) {
			String value = list5.get(i);
			System.out.println("index=" + i + ", value=" + value +
					", string length=" + value.length());
		}
	}

}
