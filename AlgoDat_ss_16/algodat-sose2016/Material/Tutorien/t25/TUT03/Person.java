
public class Person implements HasName{
	private String myName;
	
	@Override
	public String getName() {
		return pre + myName;
	}
}
