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


public class MatcherTest
{


	/**
	 * Helper function to print matrices in a nice way
	 * @param m 2D matrix of double values
	 */
	public static void prettyprintMatrix(double[][] m){
		for (double[] row: m) {
			for (double element : row) {
				if (element == Double.POSITIVE_INFINITY) {					
					System.out.print(" inf, ");
				} else {					
					System.out.print(String.format("%.2f, ", element));
				}
			}
			System.out.println();				
		}
	}
	
	/**
	 * Function to write out a signal into a .wav file
	 * 
	 * 
	 * @param signal signal to save
	 * @param filename where to save it
	 * @throws IOException
	 */
	public static void writeToWav(ISignal signal, String filename) throws IOException {
		File file = new File(filename);
		WavFile w = WavFile.newWavFile(file, 1, signal.getNumFrames(), 16, signal.getSampleRate());
		w.writeFrames(signal.getBuffer(), signal.getNumFrames());
		w.close();
		System.out.println("Mapping files written!");
	}

	
	
	//pairs of signals for testing
	Pair<ISignal, ISignal> da;
	Pair<ISignal, ISignal> dort;
	Pair<ISignal, ISignal> fast;
	Pair<ISignal, ISignal> hallo;
	
	@Before 
	public void setup() throws IOException {
		//load all wav files for testing
		
		da = new Pair<ISignal, ISignal>( 
				new SignalFromWavFile("./data/da1.wav"), 
				new SignalFromWavFile("./data/da2.wav")
		);
		dort = new Pair<ISignal, ISignal>( 
				new SignalFromWavFile("./data/dort1.wav"), 
				new SignalFromWavFile("./data/dort2.wav")
		);
		fast = new Pair<ISignal, ISignal>( 
				new SignalFromWavFile("./data/fast1.wav"), 
				new SignalFromWavFile("./data/fast2.wav")
		);
		hallo = new Pair<ISignal, ISignal>( 
				new SignalFromWavFile("./data/hallo1.wav"), 
				new SignalFromWavFile("./data/hallo2.wav")
		);
		
	}
	
	

	

	@Test
	public void testCompareTestExample(){
		double[] buffer1 = {0.0, 1.0, 3.0, 3.0, 0.0}; 
		double[] buffer2 = {0.0, 3.0, 3.0, 1.0, 0.0}; 
		ISignal s1 = new SignalFromBuffer(buffer1, 44100); 
		ISignal s2 = new SignalFromBuffer(buffer2, 44100); 

		Matcher matcher = new Matcher(s1, s2); 
		matcher.compute();

		//visualize AccDistance matrix:
		prettyprintMatrix(matcher.accumulatedDistance);
		
		
		double inf = Double.POSITIVE_INFINITY;
		//test the AccDistance matrix:
		double[][] expectedAccMatrix = {
				{0.0 , inf, inf, inf, inf,inf}, 
				{inf , 0.0, 3.0, 6.0, 7.0, 7.0}, 
				{inf , 1.0, 2.0, 4.0, 4.0, 5.0}, 
				{inf , 4.0, 1.0, 1.0, 3.0, 6.0}, 
				{inf , 7.0, 1.0, 1.0, 3.0, 6.0}, 
				{inf , 7.0, 4.0, 4.0, 2.0, 2.0}, 				
		};
		Assert.assertArrayEquals(expectedAccMatrix, matcher.accumulatedDistance);

		//Test the path:
		int[][] expectedPath = {{0,0},{1,0},{2,1},{3,2},{4,3},{4,4}};
		Assert.assertArrayEquals(expectedPath, matcher.getMappingPathAsArray());

		//Test the distance:		
		double deviation = matcher.getDistance() - 2.0;
		Assert.assertTrue( java.lang.Math.abs(deviation) < 1e-4);
		
	}

	@Test
	public void testCompareTestStretchedRamp(){
		double[] buffer1 = {0.0, 0.25, 0.5, 0.75, 1.0}; 
		double[] buffer2 = {0.0, 1.0, 1.0, 1.0, 1.0}; 
		ISignal s1 = new SignalFromBuffer(buffer1, 44100); 
		ISignal s2 = new SignalFromBuffer(buffer2, 44100); 

		Matcher matcher = new Matcher(s1, s2); 
		matcher.compute();

		prettyprintMatrix(matcher.accumulatedDistance);
		
		double inf = Double.POSITIVE_INFINITY;
		double[][] expected = {
				{0.00,  inf,  inf,  inf,  inf,  inf  }, 
				{ inf, 0.00, 1.00, 2.00, 3.00, 4.00  },
				{ inf, 0.25, 0.75, 1.50, 2.25, 3.00  }, 
				{ inf, 0.75, 0.75, 1.25, 1.75, 2.25  },
				{ inf, 1.50, 1.00, 1.00, 1.25, 1.50  },
				{ inf, 2.50, 1.00, 1.00, 1.00, 1.00  } 				
		};
		Assert.assertArrayEquals(expected, matcher.accumulatedDistance);
		double deviation = matcher.getDistance() - 1.0;
		Assert.assertTrue( java.lang.Math.abs(deviation) < 1e-4);
		
		System.out.println(matcher.getMappingPath());
	}
	
	
	@Test
	public void testCompareAllSignalsWithThemself(){
		ISignal[] signalList = {
				da.getLeft(),
				da.getRight(),
				dort.getLeft(),
				dort.getRight(),
				fast.getLeft(),
				fast.getRight(),
				hallo.getLeft(),
				hallo.getRight(),
		};
		
		for (ISignal s : signalList) {
			Matcher matcher = new Matcher(s,s); 
			matcher.compute();		
			double distance = matcher.getDistance();
			Assert.assertTrue(distance >= 0.0);
			Assert.assertTrue(distance < 0.001);
		}

	}
		
	

	@Test
	public void testCompareDa() throws IOException {
		Pair<ISignal, ISignal> pair = da;
		
		Matcher matcher = new Matcher(pair.getLeft(),pair.getRight()); 
		matcher.compute();		
		
		double deviation = matcher.getDistance() - 176.828;
		Assert.assertTrue( java.lang.Math.abs(deviation) < 1e-1);
		
		//warpes both signals and saves the result:
		Pair<ISignal, ISignal> warpedsignals = matcher.warpSignals();
		writeToWav(warpedsignals.getLeft(), "./da1_warpedto_da2.wav");
		writeToWav(warpedsignals.getRight(), "./da2_warpedto_da1.wav");
	}

	
	@Test
	public void testCompareDa2Hallo() throws IOException {
		
		Matcher matcher = new Matcher(da.getLeft(),hallo.getLeft()); 
		matcher.compute();		
		
		double deviation = matcher.getDistance() - 278.242;
		Assert.assertTrue( java.lang.Math.abs(deviation) < 1e-1);
		
		Assert.assertEquals(6331, matcher.getMappingPath().size());
		
		//warpes both signals and saves the result:
		Pair<ISignal, ISignal> warpedsignals = matcher.warpSignals();
		writeToWav(warpedsignals.getLeft(), "./da1_warpedto_hallo1.wav");
		writeToWav(warpedsignals.getRight(),  "./hallo1_warpedto_da1.wav");
	}

	@Test
	public void testCompareDa2Fast() throws IOException {
		
		Matcher matcher = new Matcher(da.getLeft(),fast.getLeft()); 
		matcher.compute();		
		
		double deviation = matcher.getDistance() - 216.001;
		Assert.assertTrue( java.lang.Math.abs(deviation) < 1e-1);
		
		Assert.assertEquals(6705, matcher.getMappingPath().size());
		
		//warpes both signals and saves the result:
		Pair<ISignal, ISignal> warpedsignals = matcher.warpSignals();
		writeToWav(warpedsignals.getLeft(), "./da1_warpedto_fast1.wav");
		writeToWav(warpedsignals.getRight(),  "./hallo1_warpedto_fast1.wav");
	}

	
 }

