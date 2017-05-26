package RSGridWorld;

import java.util.List;

import burlap.oomdp.auxiliary.StateParser;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.ObjectInstance;
import burlap.oomdp.core.State;

/**
 * Simplified state parser for grid world states. Format:<br/>
 * ax ay, l1x l1y l1t, l2x l2y lt2, ..., lnx lny lnt 
 * <br/>
 * where ax and ay is the agent x and y position and lix liy is the ith location objects x and y position and lit is the type of the ith locaiton object.
 * 
 * 
 * @author James MacGlashan
 *
 * Based off of burlap's GridWorldStateParser, this modified version for the RSGridWorld allows state parsing of the obstacle classObject.
 * Unfortunately this has required that some experiment variables (which obstacles, and how many) be public variables in the RSGridWorld class.
 *
 */
public class RSGridWorldStateParser implements StateParser {

	protected Domain				domain;
	
	
	public RSGridWorldStateParser(int width, int height){
		//Invoking the class statics in the constructor may not work as expected or at all.
		//depending on how the stateparser is called, may be worth it to pass it into the constructor
		RSGridWorld generator = new RSGridWorld(width, height, RSGridWorld.OBSTACLECLASSNAMES, RSGridWorld.NUMOBSTACLES);
		this.domain = generator.generateDomain();
	}
	
	public RSGridWorldStateParser(Domain domain){
		this.domain = domain;
	}
	
	@Override
	public String stateToString(State s) {
		
		StringBuffer sbuf = new StringBuffer(256);
		
		ObjectInstance a = s.getObjectsOfClass(RSGridWorld.CLASSAGENT).get(0);
		List<ObjectInstance> locs = s.getObjectsOfClass(RSGridWorld.CLASSLOCATION);
		List<ObjectInstance> obst = s.getObjectsOfClass(RSGridWorld.CLASSOBSTACLE);

		String xa = RSGridWorld.ATTX;
		String ya = RSGridWorld.ATTY;
		String lt = RSGridWorld.ATTLOCTYPE;
		String na = RSGridWorld.ATTNAME;
		
		sbuf.append(a.getIntValForAttribute(xa)).append(" ").append(a.getIntValForAttribute(ya));
		for(ObjectInstance l : obst){
			sbuf.append(", ").append(l.getIntValForAttribute(xa)).append(" ").append(l.getIntValForAttribute(ya)).append(" ").append(l.getStringValForAttribute(na));
		}

		for(ObjectInstance l : locs){
			sbuf.append(", ").append(l.getIntValForAttribute(xa)).append(" ").append(l.getIntValForAttribute(ya)).append(" ").append(l.getIntValForAttribute(lt));
		}
		
		return sbuf.toString();
	}

	@Override
	public State stringToState(String str) {
		
		String [] obcomps = str.split(", ");
		
		String [] acomps = obcomps[0].split(" ");
		int ax = Integer.parseInt(acomps[0]);
		int ay = Integer.parseInt(acomps[1]);
		int correctedObstacleLength = RSGridWorld.NUMOBSTACLES * RSGridWorld.OBSTACLECLASSNAMES.length;
		
		int nl = obcomps.length - 1 - correctedObstacleLength;
		
		State s = RSGridWorld.getOneAgentNLocationNObstacleState(domain, nl, RSGridWorld.NUMOBSTACLES);
		RSGridWorld.setAgent(s, ax, ay);
		
		//the first RSGridWorld.NUMOBSTACLES - 1 will be obstacles, since states are written as follows: agent, obstacles, locations
		for(int i = 1; i < obcomps.length; i++){
			String [] lcomps = obcomps[i].split(" ");
			int lx = Integer.parseInt(lcomps[0]);
			int ly = Integer.parseInt(lcomps[1]);
			
			if( i < correctedObstacleLength + 1){
				RSGridWorld.setObstacle(s, i-1, lx, ly, lcomps[2]);
			} else {
				if(lcomps.length < 3){
					RSGridWorld.setLocation(s, i-1 - correctedObstacleLength, lx, ly);
				}
				else{
					int lt = Integer.parseInt(lcomps[2]);
						RSGridWorld.setLocation(s, i-1 - correctedObstacleLength, lx, ly, lt);
				}
			}
		}
		
		return s;
	}

}