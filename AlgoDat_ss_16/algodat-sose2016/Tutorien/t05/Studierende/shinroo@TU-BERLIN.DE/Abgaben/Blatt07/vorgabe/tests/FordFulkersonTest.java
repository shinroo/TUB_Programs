import java.io.IOException;
import java.util.LinkedList;

import org.junit.Assert;
import org.junit.Test;

public class FordFulkersonTest
{
	
	private Network g1;
	private Network g2;
	private Network g3;
	
	@Test
	public void testFordFulkersonGraphFF1(){
		try {
			g1 = GraphIO.loadGraph("tests/testgraphen/graphFF_1.txt");
		} catch (IOException e) {
			e.printStackTrace();
		}
		Assert.assertEquals("Error Calculating Max Flow on Graph: graphFF_1 . ", 8, g1.fordFulkerson());
	}
	
	@Test
    public void testFordFulkersonGraphFFSimple(){
        try {
            g2 = GraphIO.loadGraph("tests/testgraphen/graphFF_simple.txt");
        } catch (IOException e) {
           
            e.printStackTrace();
           
        }
        Assert.assertEquals("Error Calculating Max Flow on Graph: graphFF_simple . ", 1200, g2.fordFulkerson());
    }
 }

