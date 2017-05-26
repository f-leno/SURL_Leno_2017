package RSGridWorld;

/* This class builds the agent space in a specifically predefined way. That is, no effort is made to see what features are
 * part of the agent-space, the agent-space is predefined (in this case, the ClassObject obstacles and all its instances)
 *
 * For use with the RSGridWorld domain build on top of the BURLAP framework
 *
 * Author Maxwell J Svetlik
*/

import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.core.State;
import burlap.oomdp.core.ObjectInstance;
import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.oomdp.core.TerminalFunction;
import burlap.behavior.singleagent.QValue;
import burlap.behavior.singleagent.Policy;
import burlap.oomdp.core.AbstractGroundedAction;
import burlap.behavior.singleagent.*;
import burlap.behavior.singleagent.learning.*;


import burlap.behavior.singleagent.learning.tdmethods.QLearning;


import java.util.List;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.HashSet;

 import burlap.behavior.singleagent.planning.QComputablePlanner.QComputablePlannerHelper;


public class RSGridWorldAgentSpaceBuilder{
	
	QComputablePlanner planner;
	Policy pi;
	LearningPair lp;
	String obstacleName;

	boolean relativeCoord = true;
	boolean distance 	  = false;
	public RSGridWorldAgentSpaceBuilder(LearningPair lp, String obstacleName){
		this.lp = lp;
		this.obstacleName = obstacleName;
	}
	public RSGridWorldAgentSpaceBuilder(LearningPair lp, String obstacleName, boolean distance){
		this(lp, obstacleName);
		this.distance = !distance;
	}

	public double distance(int sx, int sy, int tx, int ty){
		//double xdis = Math.pow(tx - sx, 2);
		//double ydis =  Math.pow(ty-sy,2);
		//double temp = Math.sqrt( xdis+ydis);
		//System.out.println("Xdis " + xdis + " Ydis " + ydis);
		return tx-sx + ty-sy;
	}

	/*
	 * Returns a convoluted multimap of the obstacle name, and the <coordinate tuple, state-value> pair.
	 */
	public HashMap<HashSet<Coordinates>, Double> getASTuple(){
		HashMap<HashSet<Coordinates>, Double> tuples = new HashMap<HashSet<Coordinates>, Double>();
		String xa = RSGridWorld.ATTX;
		String ya = RSGridWorld.ATTY;
		String lt = RSGridWorld.ATTLOCTYPE;
		String na = RSGridWorld.ATTNAME;
		EpisodeAnalysis ea = (EpisodeAnalysis)lp.ep;
		LearningAgent la = (LearningAgent)lp.la;
		for(int s = 1; s < ea.numTimeSteps() - 1; s++){
			State st = ea.getState(s);
			String action_name = ea.getAction(s).actionName();
			//System.out.println("Got action " + action_name + " in state s");
			ObjectInstance a = st.getObjectsOfClass(RSGridWorld.CLASSAGENT).get(0);
			ObjectInstance locs = st.getObjectsOfClass(RSGridWorld.CLASSLOCATION).get(0);
			List<ObjectInstance> obst = st.getObjectsOfClass(RSGridWorld.CLASSOBSTACLE);

			int ax = a.getIntValForAttribute(xa);
			int ay = a.getIntValForAttribute(ya);


			if(relativeCoord){
				double stateValue = ((QLearning)la).getQ(st,ea.getAction(s)).q;//ea.getReward(s+1);
				HashSet<Coordinates> positions = new HashSet<Coordinates>();
				/* 
				 * In learning the goal, we make the assumption that there is only one. This could be extended however.
				 */
				if(obstacleName.equals(RSGridWorld.OBSTACLEGOAL)){
					int ox = locs.getIntValForAttribute(xa);
					int oy = locs.getIntValForAttribute(ya);
					String target = RSGridWorld.OBSTACLEGOAL;
					HashMap<Coordinates, Double> workaround = new HashMap<Coordinates, Double>();


					Coordinates relTuple = null;
					if(distance){
						double dist = distance(ax, ay, ox, oy);
						relTuple = new Coordinates(dist,dist, action_name,RSGridWorld.OBSTACLEGOAL);
						if(RSGridWorld.DEBUG){
							System.out.println("ax: " + ax + " ay " + ay + " ox " + ox + " oy " + oy);
							System.out.println("Distance: " + dist);
							System.out.println("Goaaat " + relTuple.x + " " + relTuple.y + " with v: " + relTuple.actionName + " with v(s)" + stateValue + " ob " + relTuple.obstacleName);
						}
					}
					else{
						relTuple = new Coordinates(ax-ox,ay-oy, action_name,RSGridWorld.OBSTACLEGOAL);
						if(RSGridWorld.DEBUG)
							System.out.println("Got " + relTuple.x + " " + relTuple.y + " with v: " + relTuple.actionName + " with v(s)" + stateValue);
					}

					positions.add(relTuple);
					//if(!tuples.containsKey(positions)){
						tuples.put(positions, stateValue);
					//}
						double min = Double.MIN_VALUE;
						for(Coordinates t : workaround.keySet()){
							if(workaround.get(t) > min){
								min = workaround.get(t);
							}
						}
						min = 100;
						if(min != Double.MIN_VALUE){
							HashSet<Coordinates> t = new HashSet<Coordinates>();
							Coordinates n = new Coordinates(0,0,"north",target);
							t.add(n);
							HashSet<Coordinates> t1 = new HashSet<Coordinates>();
							Coordinates g = new Coordinates(0,0,"south",target);
							t1.add(g);
							HashSet<Coordinates> t2 = new HashSet<Coordinates>();
							Coordinates e = new Coordinates(0,0,"east",target);
							t2.add(e);
							HashSet<Coordinates> t3 = new HashSet<Coordinates>();
							Coordinates w = new Coordinates(0,0,"west",target);
							t3.add(w);
							tuples.put(t,min);
							tuples.put(t1,min);
							tuples.put(t2,min);
							tuples.put(t3,min);

						}
				}
				/*
				* If not learning the goal, we build a tuple of all objects and assign a value to them
				*/
				else{
					Coordinates relTuple = new Coordinates(0,0,"north");
					String target = "";
					HashMap<Coordinates, Double> workaround = new HashMap<Coordinates, Double>();
					for(ObjectInstance l : obst){
						if(!l.getStringValForAttribute(na).equals(RSGridWorld.OBSTACLEMUSIC)){
							int ox = l.getIntValForAttribute(xa);
							int oy = l.getIntValForAttribute(ya);

							if(distance){
								double dist = distance(ax, ay, ox, oy);
								relTuple = new Coordinates(dist,dist, action_name,l.getStringValForAttribute(na));
							}
							else
								relTuple = new Coordinates(ax-ox,ay-oy, action_name,l.getStringValForAttribute(na));


							positions.add(relTuple);
							/*if(relTuple.x == 0 && relTuple.y == 1 && relTuple.actionName.equals("south") || 
								relTuple.x == 0 && relTuple.y == -1 && relTuple.actionName.equals("north") || 
								relTuple.x == 1 && relTuple.y == 0 && relTuple.actionName.equals("west") || 
								relTuple.x == -1 && relTuple.y == 0 && relTuple.actionName.equals("east")){
								workaround.put(relTuple,stateValue);
							target = l.getStringValForAttribute(na);
							}*/
							//System.out.println("Got " + relTuple.x + " " + relTuple.y + " with v: " + relTuple.actionName + " with v(s)" + stateValue);
						}
					}
					//if(!tuples.containsKey(positions)){
						tuples.put(positions, stateValue);
						//System.out.println("Got " + relTuple.x + " " + relTuple.y + " with v: " + relTuple.actionName + " with v(s)" + stateValue + " ob: " + relTuple.obstacleName);

					//}
						/*double min = Double.MIN_VALUE;
						for(Coordinates t : workaround.keySet()){
							if(workaround.get(t) > min){
								min = workaround.get(t);
							}
						}*/
						if(true){//if(min != Double.MIN_VALUE){
							HashSet<Coordinates> t = new HashSet<Coordinates>();
							Coordinates n = new Coordinates(0,0,"north",RSGridWorld.OBSTACLEPIT);
							t.add(n);
							HashSet<Coordinates> t1 = new HashSet<Coordinates>();
							Coordinates g = new Coordinates(0,0,"south",RSGridWorld.OBSTACLEPIT);
							t1.add(g);
							HashSet<Coordinates> t2 = new HashSet<Coordinates>();
							Coordinates e = new Coordinates(0,0,"east",RSGridWorld.OBSTACLEPIT);
							t2.add(e);
							HashSet<Coordinates> t3 = new HashSet<Coordinates>();
							Coordinates w = new Coordinates(0,0,"west",RSGridWorld.OBSTACLEPIT);
							t3.add(w);
							//tuples.put(t,-2500.0);
							//tuples.put(t1,-2500.0);
							//tuples.put(t2,-2500.0);
							//tuples.put(t3,-2500.0);
						}
				}		


			}
			/*
			 * If not doing relative coordinates to objects, you just assign the absolute position to a value
			 */
			else{
				double stateValue = ((QLearning)la).getQ(st,ea.getAction(s)).q;
				Coordinates relTuple = new Coordinates(ax,ay, action_name);
				HashSet<Coordinates> positions = new HashSet<Coordinates>();
				positions.add(relTuple);
				//System.out.println("Got " + relTuple.x + " " + relTuple.y + " with v: " + relTuple.actionName + " with v(s)" + stateValue);

				if(!tuples.containsKey(positions))
					tuples.put(positions, stateValue);
			}
		}
	return tuples;

}
	//Iterate through states, get value
    public double getValue(State s, AbstractGroundedAction a){
        return planner.getQ(s,a).q;
    }
}
