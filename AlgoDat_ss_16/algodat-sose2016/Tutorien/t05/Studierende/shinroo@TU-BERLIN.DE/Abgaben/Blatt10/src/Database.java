
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

public class Database {

	/**
	 * A mapping from the annotation to the WavFile
	 */
	List<Pair<String, ISignal>> database;
	

	public Database(String filename) throws IOException {		
		this.loadFromFile(filename);
	}
	
	/**
	 * Reads in a database file and stores all the Signals
	 * with the according labels 
	 * @param filename
	 * @throws IOException
	 * @return Number of entries read
	 */
	protected int loadFromFile(String filename) throws IOException {
		database = new LinkedList<Pair<String,ISignal>>();
		
		// opens a file
		// if this file was not found this method will exit
		// and an IOException will be thrown to the calling method
		BufferedReader br = new BufferedReader(new FileReader(filename));

		try {
			// the BufferedReader allows to read a line of
			// the file by calling the method readLine()
			// if the end of the file is arrived, readLine()
			// will return null
	
			String line = br.readLine();
			int lineNr = 0;
			while (line != null) {
				line = line.trim();
				if (line.length() > 0 && line.charAt(0) == '#') {
					// ignore comments
					line = br.readLine();
					lineNr++;
					continue;
				}
				
				String[] lineAsArray = line.split(" ");
				if (lineAsArray.length != 2) {
					throw new IOException("Error in line " + lineNr + ": expected two entries, got " + lineAsArray.length);
				}
				
				String path = lineAsArray[1];
				ISignal signal = new SignalFromWavFile(path);
				System.out.println("-----");
				database.add(new Pair<String,ISignal>(lineAsArray[0], signal));
				
				line = br.readLine();
				lineNr++;
			}
		} finally { //makes sure the file gets closed properly even in the event of an Exception
			br.close();
		}
		return database.size();
				
	}
	
	
	/**
	 * Common Method to compare two signals.
	 *
	 * @param two signals to compare
	 * @return Matcher object containing the result 
	 * 
	 * Use this method in lookup()!
	 * 
	 */
	protected IMatcher compare(ISignal x, ISignal y) {		
		IMatcher m = new Matcher(x, y);
		m.compute();
		return m;		
	}
	
	
	/**
	 * Returns the WavFile and the annotation with the shortest
	 * distance to the queried file 
	 * 
	 * @param query
	 * @return name of the match and the Matcher containing detailed results (e.g. distance, mapping path)
	 * 
	 *		the matcher has the query as left signal, and the database reference as the right signal
	 */
	public Pair< String, IMatcher> lookup(ISignal query) {

		if(query == null)return null;
		if (database == null) return null;
		if (database.size() == 0)  return null;

		/* IMPORTANT: Use the method compare() to create Matcher objects, 
		 * and do not create them directly via "new Matcher()"!
		*/
		
		double min = Double.MAX_VALUE;
		Matcher compare = (Matcher) compare(query,query);
		Matcher retMatch = (Matcher) compare(query,query);
		String name = "";
		
		for (Pair pair: database){
			ISignal compSig = (ISignal) pair.getRight();
			compare = (Matcher) compare(query,compSig);
			if (compare.getDistance() < min){
				min = compare.getDistance();
				name = (String) pair.getLeft();
				retMatch = compare;
			}
		}
		
		return new Pair<String, IMatcher>(name, retMatch);
	}
	
}

