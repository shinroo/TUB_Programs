/**
 * This interface is to represent Enterprises. 
 * This interface extends the interface <code>Comparable</code>. So it should be possible to
 * compare Enterprises with each other.
 * @see java.lang.Comparable
 *  
 * @author AlgoDat-Tutoren
 * @version 1.1
 */
public interface Enterprise extends Comparable<Enterprise> {
    /**
     * @return returns the name of the Enterprise.
     */
    public String getName();

    /**
     * @return returns the number of employees
     */
    public int getWorkerCount();

    /**
     * This method will add a new worker to the Enterprise.
     * @param firstName first name of the worker 
     * @param lastName last name of the worker
     */
    public void addWorker(String firstName, String lastName);
}

