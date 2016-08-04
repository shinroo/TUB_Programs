import java.io.File;
import java.io.IOException;
import java.lang.Exception;
/**
 * SignalFromWavFile for creating signals from .wav files
 * 
 * @author AlgoDat team
 * 
 */
public class SignalFromWavFile implements ISignal {

	private double[] buffer;
	private int length;
	private long sampleRate;
	final int MAX_BUFFERLENGTH=30000;
	private int wavResolutionBits;

	public SignalFromWavFile(String filename) throws IOException {
	
		File file = new File(filename);
		WavFile wavfile = WavFile.openWavFile(file);
		
		long length = wavfile.getNumFrames();
		if (length > MAX_BUFFERLENGTH) {
			System.out.println("wav file is longer than maximum internal buffer, truncating it.");
			length = MAX_BUFFERLENGTH;
		}			
		this.length = (int)length;
		//read in max number of frames into the buffer
		this.buffer = new double[this.length];
		wavfile.readFrames(this.buffer, this.length);
		this.wavResolutionBits = wavfile.getValidBits();
		wavfile.close();
		
		this.sampleRate = wavfile.getSampleRate();
		
	}


	public int getNumFrames()  {
		return buffer.length;
	}

	public long getSampleRate() {
		return this.sampleRate;
	}

	public int getValidBits() {
		return this.wavResolutionBits; 
	}
	
	public double[] getBuffer()  {
		return this.buffer;		
	}


	public void trimTo(int length)  {
		if (length  > this.buffer.length ) length = this.buffer.length;   
		this.length = length;
	}
	
	
	public double getFrame(int index) {
		return buffer[index];
	}

	public void setFrame(int index, double value) {
		buffer[index] = value;
	}


}


