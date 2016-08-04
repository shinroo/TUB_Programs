import java.util.Comparator;
import java.lang.Math;

/**
 * This class implements a heuristic based on the "Manhatten" distance metric (L1 Norm)
 * 
 **/
public class HeuristicManhattan implements Comparator<CellNode> {
	
		/**
		 * The goal node which the heuristic operates on:
		 */
		CellNode goal;
		
		/**
		 * 
		 * @param goal
		 * 		the target/goal node the heuristic should be computed with
		 */
		public HeuristicManhattan(CellNode goal) {
			this.goal = goal;
		}
		
		/**
		 * Computes an estimate of the remaining distance from node n to the goal node and
		 * updates the node attribute distanceRemainingEstimate
		 * 
		 *  This class implements the L1 norm ("Manhattan" distance)
		 *  as the permissible heuristic
		 *  
		 * @param n
		 * 		node to estimate the remaining distance from
		 */
		public void estimateDistanceToGoal(CellNode n) {
        	//Your implementation here

        	//Hint: you only need to estimate once for each node.
			
			n.distanceToGoalEstimate = Math.abs(n.i - goal.i) + Math.abs(n.j - goal.j); 
        }

    	/*
    	* compares two nodes based on the distance heuristic
    	* Computes the function cost(n) = d(start, n)+h(n, goal), 
    	* where d(start, n) is the distance from the start to n \
    	* and h(n, goal) is an estimate of the distance from n to the goal based on the Manhattan distance
    	*  
    	*  
    	* @see java.util.Comparator#compare(java.lang.Object, java.lang.Object)
    	*/		
        public int compare(CellNode n1, CellNode n2) {
        	//exception handling
        	if (n1 == null || n2 == null){
        		throw new NullPointerException("One or more of the passed nodes is null!");
        	}
        	
        	if (!(n1 instanceof CellNode) || !(n2 instanceof CellNode)){
        		throw new RuntimeException("One or more of the passed arguments is of the wrong type!");
        	}
            
        	int compare = 0;
        	
        	estimateDistanceToGoal(n1);
        	estimateDistanceToGoal(n2);
        	
        	//calculate cost functions
        	double cost1 = n1.distance + n1.distanceToGoalEstimate;
        	double cost2 = n2.distance + n2.distanceToGoalEstimate;
        	
        	//compare results
        	if (cost1 < cost2){
        		compare = -1;
        	} else if (cost1 > cost2){
        		compare = 1;
        	} else if (cost1 == cost2){
        		compare = 0;
        	}
        	
            return compare;
        }		
}

