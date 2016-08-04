import java.util.ArrayList;
import java.util.ListIterator;

public class ArrayTut {

	/**
	 * This method calculates the average from given examgrades.
	 * 
	 * @param grades
	 *            grades from the result of the exam
	 * @return the average grade
	 */
	public static double calcExamAverage(ArrayList<Double> grades) {

		double total = 0.0;
		
        ListIterator<Double> i = grades.listIterator();
        while(i.hasNext()) {
        	total += i.next();
        }

		return total / grades.size();
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		int[] arr = { 1, 2, 3 };
		ArrayList<Double> testGrades = new ArrayList<Double>();
		testGrades.add(2.0);
		testGrades.add(1.3);
		testGrades.add(2.7);
		testGrades.add(4.0);
		testGrades.add(1.0);

		System.out.println("Avg. grade: " + calcExamAverage(testGrades));

		System.out.println("Sum: " + (testGrades.get(1) + testGrades.get(3)));
	}
}
