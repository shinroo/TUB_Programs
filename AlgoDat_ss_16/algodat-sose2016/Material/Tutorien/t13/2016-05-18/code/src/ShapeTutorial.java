import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ShapeTutorial {

	public static void main(String[] args) {
		RectangleShape r1 = new RectangleShape(5, 1);
		RectangleShape r2 = new RectangleShape(2, 6.1);
		RectangleShape r3 = new RectangleShape(5.01, 0.996);
		HexagonShape h1 = new HexagonShape(10.7);
		List<Shape> l = new ArrayList<>();
		l.add(r1);
		l.add(r2);
		l.add(r3);
		l.add(h1);
		
		System.out.println(r1.compareTo(r2));
		System.out.println(r2.compareTo(r1));
		System.out.println(r1.compareTo(r3));
		System.out.println(r1.compareTo(h1));
		
		Collections.sort(l);
		
		System.out.println(l);
	}
	
}
