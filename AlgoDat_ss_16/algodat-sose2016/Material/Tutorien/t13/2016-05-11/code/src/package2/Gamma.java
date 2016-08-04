package package2;
import package1.Alpha;

public class Gamma {

	public void method() {
		Alpha a = new Alpha();
		System.out.println(a.publicField);
		System.out.println(a.packagePrivateField);
		System.out.println(a.privateField);
		System.out.println(a.protectedField);
	}
	
}
