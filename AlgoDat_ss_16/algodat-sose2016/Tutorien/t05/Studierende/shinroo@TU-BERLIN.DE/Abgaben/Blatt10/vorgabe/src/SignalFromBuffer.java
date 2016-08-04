import java.io.File;
import java.io.IOException;
import java.lang.Exception;

/**
 * SignalBuffer for testing WavMatch
 * 
 * Store signals in memory buffers without having to deal with wave files
 * 
 * @author AlgoDat team
 * 
 */
public class SignalFromBuffer implements ISignal {

	double[] buffer;
	int length;
	long sampleRate;

	public SignalFromBuffer(double[] buffer, long sampleRate) {
		this.buffer = buffer;
		this.length = buffer.length;
		if (sampleRate > Integer.MAX_VALUE) {throw new RuntimeException("sample rate too high.");}
		this.sampleRate = (int) sampleRate;
	}

	
	
	public SignalFromBuffer(int framecount, long sampleRate) {
		this.buffer = new double[framecount];
		for (int i=0; i < this.buffer.length; i++) {
			this.buffer[i] = 0.0;
		}
		this.sampleRate = sampleRate;
	}

	public double getFrame(int index) {
		return buffer[index];
	}

	public void setFrame(int index, double value) {
		buffer[index] = value;
	}


	public int getNumFrames()  {
		return length;
	}

	public void trimTo(int length)  {
		if (length  > this.buffer.length ) length = this.buffer.length;   
		this.length = length;
	}


	public long getSampleRate() {
		return this.sampleRate;
	}


	public double[] getBuffer()  {
		return this.buffer;		
	}


}


