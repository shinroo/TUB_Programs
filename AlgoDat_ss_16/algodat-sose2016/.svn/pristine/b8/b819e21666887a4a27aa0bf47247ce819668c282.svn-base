package package2;

import package1.Alpha;

public class Gamma {

	void method() {
		Alpha a = new Alpha();
		System.out.println(a.publicField);
		System.out.println(a.protectedField); // Fehler: nicht sichtbar
		System.out.println(a.packagePrivateField); // Fehler: nicht sichtbar
		System.out.println(a.privateField); // Fehler: nicht sichtbar
		a.publicMethod();
		a.protectedMethod(); // Fehler: nicht sichtbar
		a.packagePrivateMethod(); // Fehler: nicht sichtbar
		a.privateMethod(); // Fehler: nicht sichtbar
	}

}
