
/**
 * Speech Recognition application
 * 
 * The application expects two arguments: 
 * The database txt file and the Wav file query.
 * 
 * You can either set these files directly in this class
 * or pass them via the command line. The fo
 *
 */
public class SpeechRecognition {
	
	/**
	 * Main method 
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		try {
			
			// Open query WavFile
			System.out.println("=======");
			System.out.println("Open query file");
		
			Database database = new Database("./data/database_small.txt"); //default database

			String queryFilename = "./data/da1.wav"; //default file to match			
			if (args.length > 1) {
				System.out.println("Malformed arguments.\n  Format:   java SpeechRecognition <file.wav>");
				return;
			} else if (args.length == 1) {				
				queryFilename =  args[0].trim();
			}
			ISignal query = new SignalFromWavFile(queryFilename);
			
			System.out.println("=======");
			System.out.println("Querying");
			// Perform query
			Pair< String, IMatcher> result = database.lookup(query);
			
			
			// Display result
			String name = result.getLeft();
			double distance = result.getRight().getDistance();
			System.out.println(String.format("detected %s (distance: %.2f)", name, distance));
			
		} catch (Exception e) {
			System.out.println("Error: " + e.getMessage());
			e.printStackTrace(System.err);
		}
	}
}

