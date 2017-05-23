package RSGridWorld;

import burlap.oomdp.core.State;
import burlap.behavior.singleagent.shaping.potential.PotentialFunction;
import burlap.oomdp.core.ObjectInstance;
import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.behavior.singleagent.QValue;

import java.util.Map;
import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;
import java.util.Set;
import java.util.HashSet;
import java.util.Iterator;


public class RSGridWorldPotentialFunction implements PotentialFunction{

	public HashMap<HashSet<Coordinates>, Double> av;
	public String obstacleName;
	Policy p;
	double last_pot;
	boolean defined;
	Map<HashSet<Coordinates>, Double> last_potential;
	public boolean stateaction = true;
	public QComputablePlanner			planner;

	public RSGridWorldPotentialFunction(Map<HashSet<Coordinates>, Double> agentValueMap, String obstacleName, Policy p){
		av = new HashMap<HashSet<Coordinates>, Double>();
		for(HashSet<Coordinates> c : agentValueMap.keySet()){
			HashSet<Coordinates> g = new HashSet<Coordinates>(c);
			av.put(g, new Double(agentValueMap.get(c)));
		}
		this.obstacleName = obstacleName;
		this.p = p;
		defined = false;
		last_pot = 0.0;
		last_potential = new HashMap<HashSet<Coordinates>, Double>();
	}
	public void setStateAction(boolean input){
		this.stateaction = input;
	}
	public void setPlanner(QComputablePlanner planner){
		this.planner = planner;
	}
	public boolean getStateAction(){return stateaction;}
	public void reset(){
		last_pot = 0.0;
		defined = false;
		last_potential = new HashMap<HashSet<Coordinates>, Double>();	
	}
	public void setPolicy(Policy p){
		this.p = p;
	}
	public AbstractGroundedAction getAction(State s){
		return p.getAction(s);
	}
	public double distance(int sx, int sy, int tx, int ty){
		return tx - sx + ty-sy;
	}
	public double potentialValue(State s){
		ObjectInstance a = s.getObjectsOfClass(RSGridWorld.CLASSAGENT).get(0);
		List<ObjectInstance> obst = s.getObjectsOfClass(RSGridWorld.CLASSOBSTACLE);
		double ep = ((BiasedPolicy)p).getEpsilon();
		//AbstractGroundedAction aga = ((BiasedPolicy)p).getBiasedAction(s);
		ObjectInstance locs = s.getObjectsOfClass(RSGridWorld.CLASSLOCATION).get(0);

		int ax = a.getIntValForAttribute("x");
		int ay = a.getIntValForAttribute("y");
		int ox = locs.getIntValForAttribute("x");
		int oy = locs.getIntValForAttribute("y");
		double dist = distance(ax, ay, ox, ox);
		//System.out.println("ax: " + ax + " ay " + ay + " ox " + ox + " oy " + oy);
		Coordinates xdis = new Coordinates(ox-ax,ox-ax, "dummy",RSGridWorld.OBSTACLEGOAL);
		Coordinates ydis = new Coordinates(oy-ay,oy-ay, "dummy",RSGridWorld.OBSTACLEGOAL);

		HashSet<Coordinates> temp = new HashSet<Coordinates>();
		temp.add(xdis);
		double maxX = Double.MIN_VALUE;
		double maxY = Double.MIN_VALUE;
		double maxXY = Double.MIN_VALUE;

		boolean skipy = false;
		if(oy == 0)
			skipy = true;

		for(HashSet<Coordinates> cs : av.keySet()){
			for(Coordinates c : cs){
				if(c.x == xdis.x && av.get(cs) > maxX)
					maxX = av.get(cs);
				if(c.x == ydis.x && av.get(cs) > maxY)
					maxY = av.get(cs);
				//else if(c.x == (ydis.x + xdis.x) && av.get(cs) > maxXY)
				//	maxXY = av.get(cs);
			}
		}
		double out = 0.;
		int divisor = 0;
		if(maxX > Double.MIN_VALUE){
			//System.out.println("Returningx: " + maxX);
			out += maxX;
			divisor++;
		}

		if(maxY > Double.MIN_VALUE && !skipy){
			//System.out.println("Returningy: " + maxY);
			out += maxY;
			divisor++;
		}

		if(maxXY > Double.MIN_VALUE && !skipy){
			//System.out.println("Returningy: " + maxY);
			//out += maxXY;
		}
		if(divisor > 0)
			return out/divisor;
		return out;
		/*
		List<QValue> qValues = this.planner.getQs(s);
		double max = Double.MIN_VALUE;
		for(QValue q : qValues){
			if(q.q > max)
				max = q.q;
		}
		if(max == Double.MIN_VALUE)
			return 0.;
		return max;
		*/
	}
	public double potentialValue(State s, AbstractGroundedAction aga){
			return ((BiasedPolicy)p).biasedValue(s,aga);
	}
}