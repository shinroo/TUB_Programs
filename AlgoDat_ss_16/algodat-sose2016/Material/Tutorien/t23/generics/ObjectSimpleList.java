
/**
 * Simple linked list
 */
public class ObjectSimpleList {

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
	public ObjectSimpleList() {
	}

	/**
	 * Create a list with values from an array.
	 * 
	 * @param values
	 *            the values to insert
	 */
	public ObjectSimpleList(Object[] values) {
		for (Object value : values) {
			add(value);
		}
	}

	/**
	 * Add a value to the list.
	 * 
	 * @param value
	 *            the value to append
	 */
	public void add(Object value) {
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
	public Object get(int index) {
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

	// Da SimpleListElement nur in der Klasse SimpleList benutzt werden kann,
	// wäre es einfacher an dieser Stelle zu implementieren. Alternativ habe ich
	// eine gleichnamige Klasse mit Gettern und Settern beigefügt.
	/**
	 * Element of SimpleList
	 */
	private class SimpleListElement {

		/**
		 * value of element
		 */
		private Object value;

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
		public SimpleListElement(Object value) {
			this.value = value;
		}

	}

}
