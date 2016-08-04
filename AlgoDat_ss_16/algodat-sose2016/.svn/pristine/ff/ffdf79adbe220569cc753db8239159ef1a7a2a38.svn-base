
public class SimpleListTutorial {

	public static void main(String[] args) {
		// int[] array = { 1, 2, 3, 4 };
		// SimpleList list = new SimpleList(array);
		System.out.println("list:");
		SimpleList<Integer> list = new SimpleList<>(new Integer[] { 1, 2, 3, 4 });
		for (int i = 0; i < list.size(); ++i) {
			System.out.println("index=" + i + ", value=" + list.get(i));
		}

		System.out.println("\nlist2:");
		SimpleList<String> list2 = new SimpleList<>(new String[] { "aaa", "b", "cc", "dddd" });
		for (int i = 0; i < list2.size(); ++i) {
			String value = list2.get(i);
			System.out.println("index=" + i + ", value=" + value + ", length=" + value.length());
		}
	}

}
