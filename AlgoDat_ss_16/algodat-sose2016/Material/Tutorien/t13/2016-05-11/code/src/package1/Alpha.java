package package1;

public class Alpha {

	public int publicField;
	protected int protectedField;
	int packagePrivateField;
	private int privateField;
	
	public void method() {
		Alpha a = new Alpha();
		System.out.println(a.publicField);
		System.out.println(a.packagePrivateField);
		System.out.println(a.privateField);
		System.out.println(a.protectedField);
	}
	
}
