import org.junit.Test;
import org.junit.Assert;

public class UnionFindTest {

  @Test
  public void testUnionFindAddElements(){
	  
	UnionFindSet<Integer> ufd = new UnionFindSet<Integer>();
    
	//Create a partition from 1
	ufd.add(1);
	Assert.assertEquals("Element 1 in first partition  does not have itself as the representative",(Integer)1,ufd.getRepresentative(1));
	
	//Create a partition from 2 
	ufd.add(2);
	Assert.assertEquals("Element 2 in second partition does not have itself as representant",(Integer)2,ufd.getRepresentative(2));
  }
  
  @Test
  public void testUnionFindMaintainPartition(){
	  
	UnionFindSet<Integer> ufd = new UnionFindSet<Integer>();
    
	ufd.add(1);
	ufd.add(2);
	ufd.add(3);
	ufd.add(10);
	
	//do one merge
	ufd.union(1, 2);
	Assert.assertNotEquals("The elements 1 and 4 are NOT different partitions (have the same representative)",ufd.getRepresentative(1),ufd.getRepresentative(4));	
	Assert.assertNotEquals("The elements 2 and 3 are NOT different partitions (have the same representative)",ufd.getRepresentative(2),ufd.getRepresentative(3));	
  }


  @Test
  public void testUnionFindUnionOnElements(){
	  
	UnionFindSet<Integer> ufd = new UnionFindSet<Integer>();
    
	ufd.add(1);
	ufd.add(2);
	ufd.add(3);
	ufd.add(4);
	ufd.add(5);
	
	//merge all
	ufd.union(1, 2);
	ufd.union(1, 3);
	ufd.union(4, 3);
	ufd.union(4, 5);
	
	Assert.assertEquals("The elements 1 and 3 are not in the same partition",ufd.getRepresentative(1),ufd.getRepresentative(3));	
	Assert.assertEquals("The elements 1 and 4 are not in the same partition",ufd.getRepresentative(1),ufd.getRepresentative(4));	
	Assert.assertEquals("The elements 1 and 5 are not in the same partition",ufd.getRepresentative(1),ufd.getRepresentative(5));	
  }
  
  
  
  @Test
  public void testUnionFindUnionOnRepresentatives(){
	  
	UnionFindSet<Integer> ufd = new UnionFindSet<Integer>();
    
	ufd.add(1000);
	ufd.add(1);
	ufd.add(100);
	ufd.add(10);
	
	//merge all
	ufd.union(1, 10);
	ufd.union(1, 100);
	ufd.union(1, 1000);
	
	Assert.assertEquals("The elements 1 and 10 are not in the same partition",ufd.getRepresentative(1),ufd.getRepresentative(10));	
	Assert.assertEquals("The elements 1 and 100 are not in the same partition",ufd.getRepresentative(1),ufd.getRepresentative(100));	
	Assert.assertEquals("The elements 1 and 1000 are not in the same partition",ufd.getRepresentative(1),ufd.getRepresentative(1000));	
  }
  
}

