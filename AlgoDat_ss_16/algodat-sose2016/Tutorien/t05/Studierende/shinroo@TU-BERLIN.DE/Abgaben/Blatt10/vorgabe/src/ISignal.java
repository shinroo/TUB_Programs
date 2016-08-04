/**
 * This interface provides methods to handle a time series signal
 * 
 * @author AlgoDat team
 */
 
public interface ISignal {

		
    /**
     * @return the number of frames this signal has
     */
	public int getNumFrames();
	
	/**
	 * 
	 * @return the number of frames per second for this signal
	 */
	public long getSampleRate();
	
	/**
	 *  get the signal value at frame index i 
	 */
	public double getFrame(int index);

	/**
	 *  set the signal value at frame index i 
	 */
	public void setFrame(int index, double value);

	/**
	 *  Trim the signal to a certain number of frames by dropping them 
	 */
	public void trimTo(int length);
	
	/**
	 * return an array with the data
	 * @return
	 */	
	public double[] getBuffer();

}


