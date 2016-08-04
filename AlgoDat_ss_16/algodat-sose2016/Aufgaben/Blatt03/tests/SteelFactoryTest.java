import org.junit.* ;


/**
 * Test-Class for the SteelFactory
 * @author AlgoDat
 *
 */
public class SteelFactoryTest {

  @Test()
  public void testAddWorker() {
	  SteelFactory enterprise = new SteelFactory("Whatever");

	  Assert.assertEquals("Worker list not empty after initialization", 0, enterprise.getWorkerCount());
	  enterprise.addWorker("Test", "Case");
	  Assert.assertEquals("Worker list.size not 1 after adding one worker ", 1, enterprise.getWorkerCount());
	  enterprise.addWorker("Nut", "Job");
	  Assert.assertEquals("Worker list.size not 2 after adding one worker ", 2, enterprise.getWorkerCount());	  
  }

  @Test()
  public void testCompareFactories() {
	  Enterprise enterprise = new SteelFactory("Werk 1");
	  enterprise.addWorker("Test1", "Case1");
	  Enterprise enterprise2 = new SteelFactory("Stammwerk");
	  
	  Assert.assertEquals("Failure for first enetrprise larger than second", 1, enterprise.compareTo(enterprise2));	  
	  enterprise2.addWorker("Test2", "Case2");
	  Assert.assertEquals("Failure for first enterprise same as second", 0, enterprise.compareTo(enterprise2));	  

  }
}




