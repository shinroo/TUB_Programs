/**
 * Element of SimpleList
 */
public class SimpleListElement {

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

	/**
	 * Get the value of the element.
	 * 
	 * @return the value
	 */
	public int getValue() {
		return value;
	}

	/**
	 * Set the value of the element.
	 * 
	 * @param value
	 *            the new value
	 */
	public void setValue(int value) {
		this.value = value;
	}

	/**
	 * Get the next element.
	 * 
	 * @return next element
	 */
	public SimpleListElement getNext() {
		return next;
	}

	/**
	 * Set the next element.
	 * 
	 * @param next
	 *            the new next element
	 */
	public void setNext(SimpleListElement next) {
		this.next = next;
	}

}
