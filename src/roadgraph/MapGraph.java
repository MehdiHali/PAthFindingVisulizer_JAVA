/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import roadgraph.Edge;
import java.util.Queue;
import java.awt.datatransfer.SystemFlavorMap;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.HashMap;
import java.util.Set;
import java.util.Stack;
import java.util.HashSet;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private int numVertices;
	private int numEdges;
	private Map<GeographicPoint,List<Edge>> adjList;
	private Set<Vertex> verticesSet = new HashSet<Vertex>();
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		numVertices = 0;
		numEdges = 0;
		adjList = new HashMap<GeographicPoint,List<Edge>>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return adjList.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}

	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if(getVertices().contains(location) || location == null) {
			System.out.println("location either already exists in the graph or empty");
			return false;
		}
		adjList.put(location, new ArrayList<Edge>());
		numVertices++;
		System.out.println("Vertex "+location+" been added successfully");
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
		String roadType, double length) throws IllegalArgumentException {
		if(!getVertices().contains(from) || !getVertices().contains(to)) {
			System.out.println("from or to point does not exist in the graph");
			throw new IllegalArgumentException();
		}
		
		Edge newEdge = new Edge(to,roadName,roadType,length);
		adjList.get(from).add(newEdge);
		numEdges++;
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        
        return bfs(start,goal,temp);
		}
	
	/**
	 * returns the neighbors of a point
	 * @param point
	 * @return
	 */
	private List<GeographicPoint> getNeibors(GeographicPoint point){
		if(point==null || !adjList.containsKey(point)) {
			System.out.println("this point does not exist in the graph");
			return null;
		}
		List<GeographicPoint> neighbors = new ArrayList<GeographicPoint>();
		for(Edge e : adjList.get(point)){
			neighbors.add(e.getEnd());
		}
		return neighbors;
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		Queue<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Map<GeographicPoint,GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();
		System.out.println("DONE initializing BFS");
		
		// the actual BFS search
		boolean found = bfsSearch(start,goal,queue,visited,parentMap,nodeSearched);
		
		// constructing path
		List<GeographicPoint> path = null;
		// we only construct the path if the goal is found
		if(found) path = constructPath(start,goal,parentMap);
		else System.out.println("GOAL NOT FOUND !");

		return path;
	}
	

	
	private boolean bfsSearch(GeographicPoint start, GeographicPoint goal, Queue<GeographicPoint> queue, Set<GeographicPoint> visited,
							Map<GeographicPoint,GeographicPoint> parentMap,Consumer<GeographicPoint> nodeSearched) {

		if(start == null || goal == null || !this.getVertices().contains(start) || !this.getVertices().contains(goal))
		{
			System.out.println("start or goal either null or does not exists in the graph");
			return false;
		}

		// Initialization
		queue.add(start);
		visited.add(start);
		parentMap.put(start, goal);
		boolean found = false;

		// pushed the start node and start searching
		while(!queue.isEmpty() && !found) {
			GeographicPoint curr = queue.poll();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(curr);
			System.out.println("current "+curr);
			if(curr.equals(goal)) {
				// if the curr is the goal then we found it
				found = true;
			}else
				for(GeographicPoint p : getNeibors(curr))
					// for each neighbor if it is not visited we process it
					if(!visited.contains(p)) {
						System.out.println("pushing "+p);
						queue.add(p);
						visited.add(p);
						parentMap.put(p, curr);
						}

			System.out.println("pushed the neighbors");
			System.out.println("goal is not in the neighbors");
		}
		return found;
	}
	
	
		private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal,Map<GeographicPoint,GeographicPoint> parentMap){
		// we will store the path in a list
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		
		/**
		 * @debug 
		for(GeographicPoint p : parentMap.keySet())
			System.out.println(p+" parent is "+parentMap.get(p));
			System.out.println("constructing the path"); 
		*/

		GeographicPoint curr = goal;
		// we start adding from the goal till the start
		path.add(curr);
		do {
			GeographicPoint parent =  parentMap.get(curr);
			curr = parent;
			System.out.println("adding "+curr+" to the path");
			((LinkedList<GeographicPoint>) path).addFirst(curr);
		}while(!curr.equals(start));

		/**
		 * @debug
		 * System.out.println("DONE : constructed the path");
		 */
		return path;
	}
	
	public static String printPath(List<GeographicPoint> path) {
		String pathStr ="";
		if(path ==null) {
			pathStr = "PATH IS EMPTY";
		}
		else 
		for(GeographicPoint p : path) {
			pathStr += " ---->["+p.x+","+p.y+"]";
		}
		System.out.println("Path: "+pathStr);
		return pathStr;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		Queue<Vertex> PQ = new PriorityQueue<Vertex>(this.getNumVertices(),new DijkstraComparator());
		List<GeographicPoint> shortestPath = null;
		Map<GeographicPoint,GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Map<GeographicPoint,Vertex> distMap = new HashMap<GeographicPoint,Vertex>();
		
		boolean found = DijkstraSearch(start, goal,PQ,parentMap, visited, distMap, nodeSearched);

		// construct the path
		GeographicPoint curr = goal;
		System.out.println("parent map "+parentMap);
		if(found)
		{
		shortestPath =  new LinkedList<GeographicPoint>();	
			do{
				// Type casting
			((LinkedList<GeographicPoint>)(shortestPath)).addFirst(curr);
			System.out.println("parent of "+curr+" is "+parentMap.get(curr));
			curr = parentMap.get(curr);
		}while(curr != null);
		}

		
		return shortestPath;
	}
	
	private boolean DijkstraSearch(GeographicPoint start, GeographicPoint goal,Queue<Vertex> PQ,Map<GeographicPoint,GeographicPoint> parentMap,Set<GeographicPoint> visited,Map<GeographicPoint,Vertex> distMap, Consumer<GeographicPoint> nodeSearched) {
		// initializsing the distMap with dist of INFINITY
		for(GeographicPoint p : this.getVertices())
			distMap.put(p, new Vertex(p));

		// start dist to 0
		distMap.get(start).setDistance(0);
		// queue start
		PQ.add(distMap.get(start));
		parentMap.put(start, null);
		
		System.out.println("DONE initializing Dijkstra");
		boolean found = false;
		int count = 0;
		
		while(!PQ.isEmpty() && !found) {
			Vertex curr = PQ.poll();
			System.out.println("curr is "+curr.getLocation());
			if(!visited.contains(curr.getLocation())) {
				System.out.println("curr is not visited");

				// Hook for visualization.  See writeup.
				nodeSearched.accept(curr.getLocation());

				visited.add(curr.getLocation());
				count++;
				if(curr.getLocation().equals(goal))
				{
					System.out.println("Goal is FOUND !!!");
					found = true;
				}
				else 
				{
						// for each of curr edges
						for(Edge e : adjList.get(curr.getLocation()))
						{
							if(!visited.contains(e.getEnd())) {
								System.out.println(e.getEnd() + " is not visited");
								double currDist = curr.getDistance() + e.getLength();
								Vertex eVertex = distMap.get(e.getEnd());
								double eDist = eVertex.getDistance();
		 						// if the curr distance is less than old then update the vertex
								if(currDist < eDist)
								{
									System.out.println("updating "+eVertex.getLocation()+": "+eDist+" to "+currDist+"= "+curr.getDistance()+"+"+e.getLength());
									eVertex.setDistance(currDist);
									parentMap.put(e.getEnd(), curr.getLocation());
									// if the current vertex is not in the queue then add it
									if(!PQ.contains(eVertex))
									{
										System.out.println("adding "+eVertex.getLocation()+" to PQ");
										PQ.add(eVertex);
									}else System.out.println("vertex already in queue");
								}
							}else System.out.println(e.getEnd() + " is visited");
						}
				}
				
			}
		}

		System.out.println("DIJKSTRAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
		System.out.println("Dijkstra visited "+count);
		
		return found;
	}
	
	public Vertex getVertex(GeographicPoint location) {
		for(Vertex v : verticesSet) {
			if(v.getLocation().equals(location)) return v;
		}
		System.out.println("location does not exist in the vertices set");
		return null;
	}


	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		Queue<Vertex> PQ = new PriorityQueue<Vertex>(this.getNumVertices(),new AStarComparator());
		List<GeographicPoint> shortestPath = null;
		Map<GeographicPoint,GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Map<GeographicPoint,Vertex> distMap = new HashMap<GeographicPoint,Vertex>();
		
		boolean found = aStarSearchSearch(start,goal,PQ,parentMap, visited,distMap, nodeSearched);
		// construct the path
		GeographicPoint curr = goal;
		System.out.println("parent map "+parentMap);
		if(found) {
			shortestPath =  new LinkedList<GeographicPoint>();
			do{
			((LinkedList<GeographicPoint>)(shortestPath)).addFirst(curr);
			System.out.println("parent of "+curr+" is "+parentMap.get(curr));
			curr = parentMap.get(curr);
		}while(curr != null);
		}

		
		return shortestPath;
	}
	
	private boolean aStarSearchSearch(GeographicPoint start, GeographicPoint goal, Queue<Vertex> PQ, Map<GeographicPoint,GeographicPoint> parentMap, Set<GeographicPoint> visited, Map<GeographicPoint, Vertex> distMap, Consumer<GeographicPoint> nodeSearched ) {

		// initializsing the distMap with dist of INFINITY
		for(GeographicPoint p : this.getVertices()) {
			double distToGoal = Math.sqrt( Math.pow((goal.x-p.getX())*1000,2) + Math.pow( (goal.y- p.getY())*1000,2));
			distMap.put(p, new Vertex(p,distToGoal));
		}

		// start dist to 0
		distMap.get(start).setDistance(0);

		// queue start
		PQ.add(distMap.get(start));
		parentMap.put(start, null);
		
		// debug
		System.out.println("DONE initializing Dijkstra");
		boolean found = false;
		int count = 0;
		while(!PQ.isEmpty() && !found) {
			Vertex curr = PQ.poll();
			System.out.println("curr is "+curr.getLocation());
			if(!visited.contains(curr.getLocation())) {
				// debug
				System.out.println("curr is not visited");

				// Hook for visualization.  See writeup.
				nodeSearched.accept(curr.getLocation());

				visited.add(curr.getLocation());
				count++;
				if(curr.getLocation().equals(goal))
				{
					// debug
					System.out.println("Goal is FOUND !!!");
					found = true;
				}
				else 
				{
						// for each of curr edges
						for(Edge e : adjList.get(curr.getLocation()))
						{
							if(!visited.contains(e.getEnd())) {
								System.out.println(e.getEnd() + " is not visited");
								double currDist = curr.getDistance() + e.getLength();
								Vertex eVertex = distMap.get(e.getEnd());
								double eDist = eVertex.getDistance();
//								if the curr distance is less than old then update the vertex
								if(currDist < eDist)
								{
									// debug
									System.out.println("updating "+eVertex.getLocation()+": "+eDist+" to "+currDist+"= "+curr.getDistance()+"+"+e.getLength());
									eVertex.setDistance(currDist);
									parentMap.put(e.getEnd(), curr.getLocation());
			//						if the current vertex is not in the queue then add it
									if(!PQ.contains(eVertex))
									{
										System.out.println("adding "+eVertex.getLocation()+" to PQ");
										PQ.add(eVertex);
									}else System.out.println("vertex already in queue");
								}
							}else System.out.println(e.getEnd() + " is visited");
						}
				}
				
			}
		}

		System.out.println("ASTARRAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
		System.out.println("A* Visited "+count+" vertices");
		
		return found;
	}

	public static void printGraph(MapGraph graph) {
		
		for(GeographicPoint p : graph.adjList.keySet()) {
			System.out.println(p +"----->"+graph.adjList.get(p)+"\n");
		}
	}
	
	
	public static void main(String[] args)
	{
		/*
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		printGraph(firstMap);
		
		// You can use this method for testing.  
		GeographicPoint start = new GeographicPoint(1.0,1.0);
		GeographicPoint goal = new GeographicPoint(8.0, -1.0);
//		GeographicPoint start = new GeographicPoint(32.866743, -117.2136249);
//		GeographicPoint goal = new GeographicPoint(32.866421, -117.2164968);
		System.out.println("Finding a path from "+start+" to "+ goal+"\n");
		List<GeographicPoint> path = firstMap.bfs(start, goal);
		printPath(path);
		*/
		
		
		
//		System.out.print("Making a new map...");
//		MapGraph secondMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/maps/utc.map", secondMap);
//		System.out.println("DONE.");
//		printGraph(firstMap);
//		
//		// You can use this method for testing.  
//		GeographicPoint utcStart = new GeographicPoint(1.0,1.0);
//		GeographicPoint utcGoal = new GeographicPoint(8.0,-1.0);
//		System.out.println("Finding a path from "+start+" to "+ goal+"\n");
//		List<GeographicPoint> utcpath = firstMap.bfs(start, goal);
//		printPath(path);
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		System.out.println("Dijkstra PATH: "+testroute);
		System.out.println("A* PATH: "+testroute2);
		
		
		System.out.println("============================");
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		System.out.println("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
		System.out.println("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
		System.out.println("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
		System.out.println("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
		System.out.println("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
		System.out.println("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
//	
//		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
