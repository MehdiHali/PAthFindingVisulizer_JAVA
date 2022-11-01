package roadgraph;
import java.util.Set;
import java.util.Comparator;
import java.util.HashSet;
import geography.GeographicPoint;;

public class Vertex{
	private GeographicPoint location;
	// distance from the start
	private double distance;
	// distance from the goal
	private double distToGoal;
	
	public Vertex() {};
	public Vertex(GeographicPoint location, double distToGoal){
		this.location = location;
		this.distance = Integer.MAX_VALUE;
		this.distToGoal = distToGoal;
	}

	public Vertex(GeographicPoint location){
		this.location = location;
		this.distance = Integer.MAX_VALUE;
		this.distToGoal = 0;
	}
	
	public GeographicPoint getLocation() {
		return this.location;
	}
	public double getDistance() {
		return this.distance;
	}
	public void setLocation(GeographicPoint location) {
		this.location = location;
	}
	public void setDistance(double distance) {
		this.distance = distance;
	}
	public double getDistToGoal() {
		return distToGoal;
	}

	public static void main(String[] args) {
		// TODO Auto-generated method stub

	}



}
