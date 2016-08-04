import java.util.ArrayList;
/**
 * A steel factory (an implementation of an enterprise)
 * represented by its name and list of employees.
 * @author AlgoDat team
 *
 */
public class SteelFactory implements Enterprise {
  
    /** Name of the factory */
    private String name;
    /** List of workers */
    private ArrayList<Worker> workers;

    /**
     * Creates a new steel factory with a given name
     * @param name Name of the factory
     */
    public SteelFactory(String name) {
        this.name = name;//TODO
        workers = new ArrayList<Worker>();
    }

    /**
     * Creates a new temporary worker and then adds the worker to the arraylist
     * @param first name of worker
     * @param last name of worker
     */
    @Override
    public void addWorker(String firstName, String lastName) {
      Worker temp = new Worker(firstName, lastName);
      workers.add(temp);//TODO
    }

    /**
     * Returns the name of the enterprise
     */
    @Override
    public String getName() {
      return this.name;//TODO
    }

    /**
     * Returns the difference in number of workers between the current enterprise and
     * another enterprise "o"
     * @param Enterprise o the enterprise to be compared to
     */
    @Override
    public int compareTo(Enterprise o) {
      return this.getWorkerCount() - o.getWorkerCount();//TODO
    }

    /**
     * Returns the number of workers in the enterprise
     */
    @Override
    public int getWorkerCount() {
      return this.workers.size();//TODO
    }
}

