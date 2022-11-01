package roadgraph;
import geography.GeographicPoint;

public class Edge {
	private GeographicPoint to;
	private String roadName;
	private String roadType;
	private double length;
	
	public Edge(GeographicPoint to, String roadName, String roadType, double length) {
		this.length=length;
		this.roadName=roadName;
		this.to=to;
	}
	
	public GeographicPoint getEnd() {
		return to;
	}
	public void setEnd(GeographicPoint to) {
		this.to = to;
	}
	public String getRoadName() {
		return roadName;
	}
	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}
	public String getRoadType() {
		return roadType;
	}
	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}
	public double getLength() {
		return length;
	}
	public void setLength(double length) {
		this.length = length;
	}

	public static void main(String[] args) {
		// TODO Auto-generated method stub

	}

}
