/**
 * Implementation of the UML example regarding inheritance.
 * Class that defines a person by its name and age.
 * @author Wen-Long
 *
 */
public class Person {
	public String name;
	public int age;
	
	/**
	 * Construct a Person object by the given values.
	 * @param name	Name of the person
	 * @param age	Age of the person
	 */
	public Person(String name, int age) {
		this.name = name;
		this.age = age;
	}
}
