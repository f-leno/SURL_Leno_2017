package RSGridWorld;

import java.util.HashSet;
import java.util.Set;
import java.util.List;

import burlap.oomdp.core.ObjectInstance;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;

/**
 * This class is used for setting a terminal function for GridWorlds that is based on the location of the agent in the world.
 * An alternative approch would be to use location objects and the atLocation propositional function to specify terminal states,
 * but this approach may be simpler to define or when terminal states are defined independent of location objects. 
 * <p/>
 * The set of terminal positions is defined either in the constructor or with subsequent calls to the {@link #markAsTerminalPosition(int, int)}
 * method. Terminal positions may also be removed with the {@link #unmarkTerminalPosition(int, int)} and {@link #unmarkAllTerminalPositions()}
 * methods.
 * @author James MacGlashan
 *
 */
public class RSGridWorldTerminalFunction implements TerminalFunction {

	/**
	 * The set of positions marked as terminal positions.
	 */
	protected Set<IntPair> terminalPositions = new HashSet<RSGridWorldTerminalFunction.IntPair>();
	protected State init;
	protected boolean terminate;
	/**
	 * Initializes without any terminal positions specified.
	 */
	public RSGridWorldTerminalFunction(State s){
		this.terminate = true;
		this.init = s;
		List<ObjectInstance> obst = s.getObjectsOfClass(RSGridWorld.CLASSOBSTACLE);
		for(ObjectInstance l : obst){
				if(l.getStringValForAttribute(RSGridWorld.ATTNAME).equals(RSGridWorld.OBSTACLEPIT)){
						//if(terminate){
							//dom.clearLocations();
							if(RSGridWorld.DEBUG)
								System.out.println("Terminal at " + l.getIntValForAttribute(RSGridWorld.ATTX) + " " + l.getIntValForAttribute(RSGridWorld.ATTY));
							terminalPositions.add(new IntPair(l.getIntValForAttribute(RSGridWorld.ATTX),l.getIntValForAttribute(RSGridWorld.ATTY)));
						//}
				}
		}
		ObjectInstance locs = s.getObjectsOfClass(RSGridWorld.CLASSLOCATION).get(0);
		terminalPositions.add(new IntPair(locs.getIntValForAttribute(RSGridWorld.ATTX),locs.getIntValForAttribute(RSGridWorld.ATTY)));
	}
	public RSGridWorldTerminalFunction(State s, boolean terminate){
		this(s);
		this.terminate = terminate;
	}
	
	/**
	 * Initializes with a terminal position at the specified agent x and y locaiton.
	 * @param x the x location of the agent
	 * @param y the y location of the agent
	 */
	public RSGridWorldTerminalFunction(int x, int y){
		this.terminalPositions.add(new IntPair(x, y));
	}
	
	/**
	 * Initializes with a list of terminal positions specified by a sequence of {@link IntPair} objects.
	 * @param terminalPositions the agent positions that are terminal states, specified with {@link IntPair} objects.
	 */
	public RSGridWorldTerminalFunction(IntPair...terminalPositions){
		for(IntPair i : terminalPositions){
			this.terminalPositions.add(i);
		}
	}
	
	/**
	 * Marks a position as a terminal position for the agent.
	 * @param x the x location of the agent.
	 * @param y the y location of the agent.
	 */
	public void markAsTerminalPosition(int x, int y){
		this.terminalPositions.add(new IntPair(x, y));
	}
	
	/**
	 * Unmarks an agent position as a terminal position.
	 * @param x the x location of the agent.
	 * @param y the y location of the agent.
	 */
	public void unmarkTerminalPosition(int x, int y){
		this.terminalPositions.remove(new IntPair(x, y));
	}
	
	/**
	 * Unmarks all agent positions as terminal positions. That is, no position will be considered a terminal position.
	 */
	public void unmarkAllTerminalPositions(){
		this.terminalPositions.clear();
	}
	
	/**
	 * Returns true if a position is marked as a terminal position; false otherwise.
	 * @param x the x location of the agent.
	 * @param y the y location of the agent.
	 * @return true if a position is marked as a terminal position; false otherwise.
	 */
	public boolean isTerminalPosition(int x, int y){
		return this.terminalPositions.contains(new IntPair(x, y));
	}
	
	@Override
	public boolean isTerminal(State s) {
		ObjectInstance agent = s.getFirstObjectOfClass(RSGridWorld.CLASSAGENT);
		int x = agent.getIntValForAttribute(RSGridWorld.ATTX);
		int y = agent.getIntValForAttribute(RSGridWorld.ATTY);
		ObjectInstance loc = s.getFirstObjectOfClass(RSGridWorld.CLASSLOCATION);
		int lx = loc.getIntValForAttribute(RSGridWorld.ATTX);
		int ly = loc.getIntValForAttribute(RSGridWorld.ATTY);


		IntPair ag = new IntPair(x,y);
		if(terminalPositions.contains(ag)){
			//System.out.println("IN PIT");
			return true;
		}
		boolean atLoc = (lx == x && ly == y);
		//if(atLoc && terminate)
		//	dom.clearLocations();
		return atLoc && terminate;
	}
	
	
	/**
	 * A pair class for two ints.
	 * @author James MacGlashan
	 *
	 */
	public class IntPair{
		public int x;
		public int y;
		public IntPair(int x, int y){
			this.x = x;
			this.y = y;
		}
		
		@Override
		public int hashCode(){
			return this.x + 31*this.y;
		}
		
		@Override
		public boolean equals(Object other){
			if(!(other instanceof IntPair)){
				return false;
			}
			
			IntPair o = (IntPair)other;
			return this.x == o.x && this.y == o.y;
		}
	}

}