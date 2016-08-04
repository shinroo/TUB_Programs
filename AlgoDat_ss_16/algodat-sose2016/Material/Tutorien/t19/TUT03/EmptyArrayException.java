	/**
	 * You can write your own exceptions types by extending the Java class Exception
	 */
	public class EmptyArrayException extends Exception {
		 
		
		public EmptyArrayException(String msg){
		      super(msg);
		 }
		
		/**
		 * This Exception implements a special function
	     */
		public String specialMethod(){
			return "You should fill the array";
		}
	}

