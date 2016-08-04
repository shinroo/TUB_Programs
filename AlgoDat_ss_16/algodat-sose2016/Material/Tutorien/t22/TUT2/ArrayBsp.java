import java.util.ArrayList;
import java.util.Iterator;


public class ArrayBsp {

	public static void main(String[] args) {
		//ArrayList
		
		ArrayList<String> arrayListObj = new ArrayList<String>();
		arrayListObj.add("Element 1");
		arrayListObj.add("Element 2");
		
		for (int i = 0; i < 1000; i++) {
			arrayListObj.add("For Element " + i);
		}
		
//		for (int i = 0; i < arrayListObj.size(); i++) {
//			arrayListObj.add("For Element " + i);
//		}
		
		Iterator<String> it = arrayListObj.iterator();
		
		int num = 0;
		
		while (it.hasNext()) {
			num++;
			System.out.println(num + ". " + it.next());
		}
		
		//Array
		
		int a = 3;
		
		String[] arrayObj = new String[a];
		arrayObj[0] = "Etwas 1";
		arrayObj[1] = "Etwas 2";
		arrayObj[2] = "Etwas 3";
		
		for (int i = 0; i < arrayObj.length; i++) {
			System.out.println(arrayObj[i]);
		}
		
		Picture p = new Picture(300, 300);
		
	}

}
