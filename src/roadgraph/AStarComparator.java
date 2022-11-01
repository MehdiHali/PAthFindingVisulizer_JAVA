package roadgraph;

import java.util.Comparator;

public class AStarComparator implements Comparator<Vertex>{
	
	
	@Override
	public int compare(Vertex v1, Vertex v2) {
		// f(n) = g(n) + h(n)
		double fv1 = v1.getDistance() + v1.getDistToGoal();
		double fv2 = v2.getDistance() + v2.getDistToGoal();
			System.out.println("v1 distTOGoal: "+v1.getDistToGoal());
			System.out.println("v2 distTOGoal: "+v2.getDistToGoal());
			System.out.println(v1.getLocation()+" heuristic "+fv1);
			System.out.println(v2.getLocation()+" heuristic "+fv2);
		if( fv1 < fv2 ) {
			System.out.println("v1 is prior");
			return -1;
		}
		if(fv1 > fv2) {
			System.out.println("v2 is prior");
			return 1;
		}
		else return 0;
	}

}
