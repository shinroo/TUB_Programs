import java.util.ArrayList;
import java.util.Iterator;


public class ArrayBsp {

	public static void main(String[] args) {
		
		//ArrayList
		ArrayList<String> arrayListObj = new ArrayList<String>();
		arrayListObj.add("Etwas 1");
		arrayListObj.add("Etwas 2");
		arrayListObj.add("Etwas 3");
		
		for (int i = 0; i < 1000; i++) {
			arrayListObj.add("Etwas" + i);
			System.out.println(i + " Etwas hinzugefügt");
		}
		
		for (int i = 0; i < 1000; i++) {
			System.out.println(i + " Etwas = " + arrayListObj.get(i));
		}
		
		Iterator it = arrayListObj.iterator();
		int num = 1;
		while(it.hasNext()){
			System.out.println(num + " Etwas = " + it.next());
			num++;
		}
		
		//Arrays
		String[] arrayObj = new String[3];
		arrayObj[0] = "Element 1";
		arrayObj[1] = "Element 2";
		arrayObj[2] = "Element 3";
		
		System.out.println("----------------------------------------------");
		
		for(int i = 0; i < arrayObj.length; i++) {
			System.out.println(i + "-tes Element = " + arrayObj[i]);
		}
		
		Picture p = new Picture (200, 200);

	}

}
