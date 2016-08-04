/**
 * Implementation of the UML example regarding inheritance.
 * Class that defines a student by its name, age and student number (Matrikelnummer).
 * @author Wen-Long
 *
 */
public class Student extends Person{
	public int studentNumber;
	
	/**
	 * Constructs a Student object by the given values.
	 * @param name	Name of the student
	 * @param age	Age of the student
	 * @param studentNumber	Student number of the student
	 */
	public Student(String name, int age, int studentNumber) {
		super(name, age);
		this.studentNumber = studentNumber;
	}
	
}
