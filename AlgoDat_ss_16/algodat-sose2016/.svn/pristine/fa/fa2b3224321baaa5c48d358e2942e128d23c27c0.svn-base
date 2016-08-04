/**
 * Simple linked list
 */
public class SimpleList<T> {

	/**
	 * first element
	 */
	private SimpleListElement<T> head;

	/**
	 * last element
	 */
	private SimpleListElement<T> tail;

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
	public SimpleList(T[] values) {
		for (T value : values) {
			add(value);
		}
	}

	/**
	 * Add a value to the list.
	 * 
	 * @param value
	 *            the value to append
	 */
	public void add(T value) {
		SimpleListElement<T> element = new SimpleListElement<>(value);
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
	public T get(int index) {
		SimpleListElement<T> current = head;
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
		SimpleListElement<T> current = head;
		while (current != null) {
			++count;
			current = current.next;
		}
		return count;
	}

	/**
	 * Element of SimpleList
	 */
	private class SimpleListElement<T2> {

		/**
		 * value of element
		 */
		private T2 value;

		/**
		 * next element
		 */
		private SimpleListElement<T2> next;

		/**
		 * Create an element of SimpleList.
		 * 
		 * @param value
		 *            value of element
		 */
		public SimpleListElement(T2 value) {
			this.value = value;
		}

	}

}
