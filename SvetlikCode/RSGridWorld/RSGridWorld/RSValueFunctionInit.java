package RSGridWorld;

import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.core.State;
import burlap.behavior.singleagent.ValueFunctionInitialization;
	/**
	 * A {@link ValueFunctionInitialization} implementation that always returns a constant value.
	 * @author James MacGlashan
	 *
	 */
	public class RSValueFunctionInit implements ValueFunctionInitialization{

		/**
		 * The constant value to return for all initializations.
		 */
		public double value = 0;
		public BiasedPolicy p;
		
		/**
		 * Will cause this object to return 0 for all initialization values.
		 */
		public RSValueFunctionInit(BiasedPolicy p){
			this.p = p;
		}
		
		
		/**
		 * Will cause this object to return <code>value</code> for all initialization values.
		 * @param value the value to return for all initializations.
		 */
		public RSValueFunctionInit(double value){
			this.value = value;
		}
		
		@Override
		public double value(State s) {
			System.out.println("Polling");
			return value;
		}

		@Override
		public double qValue(State s, AbstractGroundedAction a) {
			value = p.biasedValue(s,a);
			System.out.println("Initializing value: " + value);
			return value;
		}
		
		
		
		
	}
	
