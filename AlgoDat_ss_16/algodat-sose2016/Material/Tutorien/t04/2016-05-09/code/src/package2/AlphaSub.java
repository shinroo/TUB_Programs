package package2;

import package1.Alpha;

public class AlphaSub extends Alpha {

	void method() {
		//Alpha a = new Alpha(); // würde a statt this genutzt werden, wäre das Verhalten wie bei Gamma
		System.out.println(this.publicField);
		System.out.println(this.protectedField);
		System.out.println(this.packagePrivateField); // Fehler: nicht sichtbar
		System.out.println(this.privateField); // Fehler: nicht sichtbar
		this.publicMethod();
		this.protectedMethod();
		this.packagePrivateMethod(); // Fehler: nicht sichtbar
		this.privateMethod(); // Fehler: nicht sichtbar
	}

}
