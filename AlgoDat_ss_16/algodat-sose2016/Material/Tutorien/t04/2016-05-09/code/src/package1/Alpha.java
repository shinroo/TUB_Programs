package package1;

public class Alpha {

	public int publicField;
	protected int protectedField;
	int packagePrivateField;
	private int privateField; // Warning: unused field

	public void publicMethod() {}
	protected void protectedMethod() {}
	void packagePrivateMethod() {}
	private void privateMethod() {} // Warning: unused method

	// Wir werden gewarnt, da die privaten Member nur innerhalb der Klasse benutzt werden können,
	// wir sie aber nicht verwenden.

}
