

public class BlaKeks<T> {

	T data;
	
	BlaKeks(){
		this.data = null;
	}
	BlaKeks(T input){
		this.data = input;
	}
	
	public T getData(){
		return this.data;
	}
	public void setData(T input){
		this.data = input;
	}
	
	public static void main(String args[]){
		BlaKeks<String> bkString = new BlaKeks<String>();
		bkString.setData("Hose");
		System.out.println(bkString.getData());
		
		BlaKeks<Integer> bkInteger = new BlaKeks<Integer>();
		bkInteger.setData(782138);
		System.out.println(bkInteger.getData());
	}
}
