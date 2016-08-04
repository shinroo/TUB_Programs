/**
 * Code for playing with SimpleList.
 */
public class SimpleListTutorial {

	public static void main(String[] args) {
		System.out.println("SimpleList example:");
		int[] values1 = {1,2,3};
		SimpleList list1 = new SimpleList(values1);
		for(int i = 0; i < list1.size(); i++){
			int value = list1.get(i);
			System.out.println("list at "+i+": "+value);
		}
		System.out.println("");
		System.out.println("ObjectSimpleList example:");
		Object[] values2 = {"s8","s9","s10"};
		ObjectSimpleList list2 = new ObjectSimpleList(values2);
		for(int i = 0; i < list2.size(); i++) {
			String value = (String) list2.get(i);
			System.out.println("list at " + i + ": " + value+" ("+value.length()+")");
		}
		System.out.println("");
		System.out.println("GenericsSimpleList example:");
		String[] values3 = {"g1","g2","g3"};
		GenericsSimpleList<String> list3 = new GenericsSimpleList<String>(values3);
		for(int i = 0; i < list3.size(); i++) {
			String value = list3.get(i);
			System.out.println("list at " + i + ": " + value+" ("+value.length()+")");		}
	}
}
