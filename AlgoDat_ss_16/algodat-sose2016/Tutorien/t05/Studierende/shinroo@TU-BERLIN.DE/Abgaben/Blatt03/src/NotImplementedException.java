/**
 * This class is used on not implemented methods/tasks.
 */
public class NotImplementedException extends RuntimeException {
	private static final long serialVersionUID = 1L;
	private final String method;
	
	public NotImplementedException(String method) {
		this.method = method;
	}
	
	@Override
	public String toString() {
		return "Method "+method+" should be implemented!";
	}
}

