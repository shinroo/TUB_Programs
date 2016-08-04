import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;

public class GraphIO{


/**
	 * loads a Graph from given file
	 * 
	 * @param filename
	 *            URL to file
	 * @return a MyGraph stored in file
	 * @throws IOException
	 */
public static Network loadGraph(String filename) throws IOException {
	// creates a new graph
	Network m = new Network();
	loadGraph(m, filename);
	return m;
}

	/**
	 * loads a Graph from given file
	 * 
	 * @param filename
	 *            URL to file
	 * @param m
	 * 			INetwork object to load network into 
	 * @throws IOException
	 */	
public static void loadGraph(INetwork m, String filename) throws IOException {

		// opens a file
		// if this file was not found this method will exit
		// and an IOException will be thrown to the calling method
		BufferedReader br = new BufferedReader(new FileReader(filename));

		// the BufferedReader allows to read a line of
		// the file by calling the method readLine()
		// if the end of the file is arrived, readLine()
		// will return null

		// get the number of nodes, by reading the first line
		// and convert the String into an int-value
		int cNodes = new Integer(br.readLine()).intValue();

		// add Nodes to the graph
		for (int i = 0; i < cNodes; i++) {
			m.addNode();
		}

		// add edges to the graph
		int currNode = 0;
		String line = br.readLine();
		while (line != null) {
			String[] lineAsArray = line.split(" ");
			if (lineAsArray.length < cNodes) {
				String msg = String.format("There are fewer columns than expected in the edge array in line %d: %d instead of %d",currNode, lineAsArray.length, cNodes);
				br.close();
				throw new java.io.IOException(msg);
			}
			for (int i = 0; i < cNodes; i++) {
				// cast value to int
				int weight = new Integer(lineAsArray[i]).intValue();
				// add edge if weight != 0
				if (weight != 0)
					m.addEdge(currNode, i, weight);
			}
			line = br.readLine();
			currNode++;
		}

		// closes the buffer
		br.close();

	}

	/**
	 * saves a Graph to given file
	 * 
	 * @param file url to file
	 * @param g the graph, has to be saved
	 * @throws IOException
	 */
	public static void saveGraph(INetwork g, String file) throws IOException{
			//open a new Stream to the File

			PrintWriter bufW = new PrintWriter(file);
			
			//get all nodes of the graph
			List<Node> nodes = g.getNodes();
			int nrNodes=nodes.size();
			
			//print number of Nodes as first value into File
			bufW.print(nrNodes);
			bufW.println();
			
			// now printing the adjacency matrix of the graph into File
			for(Node start:nodes){
				
				for(Node end:nodes){
						
						bufW.print(start.getEdgeTo(end).getCapacity());
						if(end.getID()<nrNodes-1)
							bufW.print(" ");
					
				}
				bufW.println();
			}
			
			//write all values in the Stream into the File if it wasn't done until yet
			bufW.flush();
			
			//close the Stream
			bufW.close();

	}

}

