import java.util.Comparator;

/**
 * This class implements a heuristic based on the "Euclidean" distance metric (L2 Norm)
 * 
 **/
 public class HeuristicEuclidean implements Comparator<CellNode> {
	
	    /*
	    This is the goal node the heuristic uses to estimate the remaining distance
	    */
		CellNode goal;
		
		public HeuristicEuclidean(CellNode goal) {
			this.goal = goal;
		}
		
		/**
		 * Computes an estimate of the remaining distance from node n to the goal node and
		 * updates the node attribute distanceRemainingEstimate
		 * 
		 *  This class implements the L2 norm (euclidean distance)
		 *  as the permissible heuristic
		 *  
		 * @param n
		 * 		node to estimate the remaining distance from
		 */
		public void estimateDistanceToGoal(CellNode n) {
        	//Your implementation here
			n.distanceToGoalEstimate = 0.0; //Setting this to 0.0 effectively turns A* into Dijkstra
        	//Hint: you only need to compute the estimate once for each node.
        	 
			n.distanceToGoalEstimate = Math.sqrt((n.i - goal.i)*(n.i - goal.i) + (n.j - goal.j)*(n.j - goal.j)); 
			
        	return;
		}
		
		
		
		/*
		 * compares two nodes based on an estimate of the path length to goal
		 * Computes the function cost(n) = d(start, n)+h(n, goal), 
		 * where d(start, n) is the distance from the start to n \
		 * and h(n, goal) is an estimate of the distance from n to the goal based on the Euclidean distance.
		 *  
		 * @see java.util.Comparator#compare(java.lang.Object, java.lang.Object)
		 */		
        public int compare(CellNode n1, CellNode n2) {
        	//Your implementation here
        	
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

