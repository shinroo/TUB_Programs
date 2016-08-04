import java.util.ArrayList;
import java.util.Iterator;


public class BlaKeks {

	//Can seen everywhere
	public int a;		
	//just in this class
	private int b;
	//in this class and every childclass
	protected int c;
	//in this class and every other class in this package
	int e;
	
	static int d;
	
	
	
	public static void main(String args[]){
		//ArrayList
		
		ArrayList<String> arrayListObj = new ArrayList<String>();
		arrayListObj.add("Element_1");
		arrayListObj.add("Element_2");
		arrayListObj.add("Element_3");
		arrayListObj.add(1, "Hose");
		System.out.println(arrayListObj);
		Iterator it = arrayListObj.iterator();
		while(it.hasNext())
			System.out.println(it.next() + " ");
		
		//Array
		String[] arrayObj = new String[3];
		arrayObj[0] = "Element_1";
		arrayObj[1] = "Element_2";
		arrayObj[2] = "Element_3";
		System.out.println(arrayObj);
		for(int i=0;i<arrayObj.length;i++){
			System.out.println(arrayObj[i]);
		}
		
	}
}
