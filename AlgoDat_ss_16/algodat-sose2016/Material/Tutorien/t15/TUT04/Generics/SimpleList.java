/**
 * Simple linked list
 */
public class SimpleList {

	/**
	 * first element
	 */
	private SimpleListElement head;

	/**
	 * last element
	 */
	private SimpleListElement tail;

	/**
	 * Create an empty list
	 */
	public SimpleList() {
	}

	/**
	 * Create a list with values from an array.
	 * 
	 * @param values
	 *            the values to insert
	 */
	public SimpleList(int[] values) {
		for (int value : values) {
			add(value);
		}
	}

	/**
	 * Add a value to the list.
	 * 
	 * @param value
	 *            the value to append
	 */
	public void add(int value) {
		SimpleListElement element = new SimpleListElement(value);
		if (head == null) {
			head = tail = element;
		} else {
			tail.next = element;
			tail = element;
		}
	}

	/**
	 * Get the value at index.
	 * 
	 * @param index
	 *            index of element
	 * @return the value of the element
	 */
	public int get(int index) {
		SimpleListElement current = head;
		for (int i = 0; i < index && current != null; ++i) {
			current = current.next;
		}
		return current.value;
	}

	/**
	 * Calculate the size.
	 * 
	 * @return the size of list
	 */
	public int size() {
		int count = 0;
		SimpleListElement current = head;
		while (current != null) {
			++count;
			current = current.next;
		}
		return count;
	}

	/**
	 * Element of SimpleList
	 */
	private class SimpleListElement {

		/**
		 * value of element
		 */
		private int value;

		/**
		 * next element
		 */
		private SimpleListElement next;

		/**
		 * Create an element of SimpleList.
		 * 
		 * @param value
		 *            value of element
		 */
		public SimpleListElement(int value) {
			this.value = value;
		}

	}

}
