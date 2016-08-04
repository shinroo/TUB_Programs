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
        throw new NotImplementedException("SteelFactory");//TODO
    }

    @Override
    public void addWorker(String firstName, String lastName) {
      throw new NotImplementedException("addWorker");//TODO
    }

    @Override
    public String getName() {
      throw new NotImplementedException("getName");//TODO
    }

    @Override
    public int compareTo(Enterprise o) {
      throw new NotImplementedException("compareTo");//TODO
    }

    @Override
    public int getWorkerCount() {
      throw new NotImplementedException("getWorkerCount");//TODO
    }
}

