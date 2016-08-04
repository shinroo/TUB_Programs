
/**
 * This class represents a pair of objects of a generic type t
 */

public class Pair<T> {
	
	/** The first object */
	T first = null;
	/** The second object */
	T second = null;
	
	 /**
     * Creates a new pair of objects
     * @param first, the first object
     * @param second, the second object
     */
	Pair(T first, T second) {
		// TODO
		this.first = first;
		this.second = second;
	}

	 /**
     * Returns the first object
     */
	public T getFirst() {
		// TODO
		return this.first;
	}

	 /**
     * Returns the second object
     */
	public T getSecond() {
		// TODO
		return this.second;
	}

	 /**
     * Sets the first object
     */
	public void setFirst(T dt) {
		// TODO
		this.first = dt;
	}

	/**
     * Sets the second object
     */
	public void setSecond(T dt) {
		// TODO
		this.second = dt;
	}

	/**
     * Swaps the objects in the pair
     */
	public void swap() {
		// TODO
		T temp = this.first;
		this.first = this.second;
		this.second = temp;
	}

	/**
     * Attempts to return the objects as strings
     * if a RuntimeException is caught, then it returns the string
     * "<not implemented>, <not implemented>"
     */
	@Override
	public String toString() {
		// TODO
		try {
			return "\"" + first.toString() + ", " + second.toString() + "\"";
		} catch (RuntimeException e) {
			return "<not implemented>, <not implemented>";
		}
	}
}

