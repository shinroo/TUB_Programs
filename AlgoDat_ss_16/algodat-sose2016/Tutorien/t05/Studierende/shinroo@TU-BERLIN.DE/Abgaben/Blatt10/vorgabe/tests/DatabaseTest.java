import java.io.File;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;


public class DatabaseTest
{
	Database db;
	
	@Before 
	public void setup() throws IOException {		
		db = new Database("./data/database_small.txt");
	}

	@Test
	public void testDa2() throws IOException {		
		ISignal signal = new SignalFromWavFile("./data/da2.wav"); //is in the database, so it should return distance 0.0
		Pair< String, IMatcher> query = db.lookup(signal);
		Assert.assertEquals("da", query.getLeft() );
		Assert.assertTrue( java.lang.Math.abs(query.getRight().getDistance()) < 1e-4);
	}

	@Test
	public void testFast2() throws IOException {		
		ISignal signal = new SignalFromWavFile("./data/fast2.wav"); //is in the database, so it should return distance 0.0
		Pair< String, IMatcher> query = db.lookup(signal);
		Assert.assertEquals("fast", query.getLeft() );
		Assert.assertTrue( java.lang.Math.abs(query.getRight().getDistance()) < 1e-4);		
	}

	@Test
	public void testDa1() throws IOException {		
		ISignal signal = new SignalFromWavFile("./data/da1.wav");
		Pair< String, IMatcher> query = db.lookup(signal);
		Assert.assertEquals("da", query.getLeft() );
		double distance = java.lang.Math.abs(query.getRight().getDistance());
		Assert.assertTrue( (176. < distance) && (distance < 177.) );		
	}

	
 }

