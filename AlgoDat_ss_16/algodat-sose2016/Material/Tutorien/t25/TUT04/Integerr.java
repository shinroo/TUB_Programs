
public class Integerr {
	private int number;
	
	public Integerr(int number){
		this.number = number;
	}
	
	public static void main(String[] args) {
		Integerr one = new Integerr(1000);
		Integerr two = new Integerr(1000);
		System.out.println(one.hashCode() + "_vs_" + two.hashCode());
		
	}

}
