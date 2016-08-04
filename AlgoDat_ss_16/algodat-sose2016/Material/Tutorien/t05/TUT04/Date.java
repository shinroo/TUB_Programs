
public class Date {
	public int day;
	public int month;
	public int year;
	
	@Override
	public int hashCode(){
		int hashCode = 0;
		hashCode += day * 1;
		hashCode += month * 32;
		hashCode += year * 16 * 32;
		return hashCode;
	}
	
	public static void main(String[] args) {
		// TODO Auto-generated method stub

	}

}





