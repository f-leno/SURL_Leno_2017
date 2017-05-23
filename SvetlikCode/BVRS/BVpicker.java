import RSGridWorld.*;
import java.awt.Color;
import java.util.List;
import java.util.*;

import burlap.behavior.singleagent.*;
import burlap.domain.singleagent.gridworld.*;
import burlap.oomdp.core.*;
import burlap.oomdp.singleagent.*;
import burlap.oomdp.singleagent.common.*;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.oomdp.auxiliary.StateParser;
//visualization
import burlap.oomdp.visualizer.Visualizer;
import burlap.behavior.singleagent.EpisodeSequenceVisualizer;
//plotting visuals
import burlap.oomdp.auxiliary.StateGenerator;
import burlap.oomdp.auxiliary.common.ConstantStateGenerator;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
//planning & learning
import burlap.behavior.singleagent.learning.*;
import burlap.behavior.singleagent.learning.tdmethods.*;
import burlap.behavior.singleagent.planning.*;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.deterministic.*;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.behavior.singleagent.Policy.ActionProb;
import burlap.oomdp.core.State;
import burlap.oomdp.core.Attribute;
import burlap.oomdp.core.Attribute.AttributeType;


//graphs and plotting
import burlap.oomdp.auxiliary.StateGenerator;
import burlap.oomdp.auxiliary.common.ConstantStateGenerator;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;


public class BVpicker{

	
	BVRS                 		bvrs;
    Domain                      domain;
    StateParser                 sp;
    RewardFunction              rf;
    TerminalFunction            tf;
    StateConditionTest          goalCondition;
    State                       initialState;
    DiscreteStateHashFactory    hashingFactory;


	public static void main(String[] args) {
		String[] obstacles = {RSGridWorld.OBSTACLEFIRE, RSGridWorld.OBSTACLEPIT};
        RewardShape problem = new RewardShape(obstacles, 2, 10, 10);
        BVpicker example = new BVpicker(problem);
        example.experimenterAndPlotter("test");
        //example = new RewardShape(obstacles, 1, 2, 2, .99, example.getStates(), example.getPlanner());
        //example.SarsaLearningExample(outputPath, 1, 1);
        //example.visualize(outputPath);
        //example.experimenterAndPlotter("expDataRSRF0");
        //example = new RewardShape(obstacles, 5, 5, 5, .99, example.getStates(), example.getPlanner());
        //example.experimenterAndPlotter("expDataRSRF1");
        //example = new RewardShape(obstacles, 50, 20, 20, .99, example.getStates(), example.getPlanner());
        //example.experimenterAndPlotter("expDataRSRF2");
        //example.ValueIterationExample(outputPath);
	}
	
	public BVpicker(RewardShape rs){

        //create the domain
        bvrs = new BVRS(rs);

        domain = bvrs.generateDomain();
        //create the state parser
        sp = new RSGridWorldStateParser(domain);

        rf = new BVRSRewardFunction(rs.SarsaLearningExample(100), bvrs);
        tf = new SinglePFTF(domain.getPropFunction("ChoseTermActionPF"));
        goalCondition = new TFGoalCondition(tf);

        //set up the initial state of the task
        initialState = bvrs.getInitState(domain);
        bvrs.setTermAction(initialState, false);

        //set up the state hashing system
        hashingFactory = new DiscreteStateHashFactory();
	}


	    /* This method creates a graph, or several, based on some class defined metrics. This compares Qlearning and sarsaLamda on
     * the same domain, reward function, and terminal function
     */
    public void experimenterAndPlotter(String fileName){

        //custom reward function for more interesting results
        //final RewardFunction rf = new GoalBasedRF(this.goalCondition, 5., -0.1);
        /**
         * Create factories for Q-learning agent and SARSA agent to compare
         */

        LearningAgentFactory qLearningFactory = new LearningAgentFactory() {

            @Override
            public String getAgentName() {
                return "Q-learning";
            }

            @Override
            public LearningAgent generateAgent() {
                return new QLearning(domain, rf, tf, 0.99, hashingFactory, 0.3, 0.1);
            }
        };

        LearningAgentFactory sarsaLearningFactory = new LearningAgentFactory() {

            @Override
            public String getAgentName() {
                return "SARSA";
            }

            @Override
            public LearningAgent generateAgent() {
                return new SarsaLam(domain, rf, tf, 0.99, hashingFactory, 0.0, 0.1, 0.9);
            }
        };

        StateGenerator sg = new ConstantStateGenerator(this.initialState);

        LearningAlgorithmExperimenter exp = new LearningAlgorithmExperimenter((SADomain)this.domain,
            rf, sg, 1, 1000, qLearningFactory);

        //BoundedLearningAlgorithmExperimenter exp = new BoundedLearningAlgorithmExperimenter((SADomain)this.domain,
        //    rf, sg, 2, 1000, 200, qLearningFactory);
        exp.toggleTrialLengthInterpretation(true);
        exp.setUpPlottingConfiguration(500, 250, 1, 1000,
        TrialMode.MOSTRECENTANDAVERAGE,
        PerformanceMetric.CUMULATIVESTEPSPEREPISODE,
        PerformanceMetric.AVERAGEEPISODEREWARD);

        exp.startExperiment();

        exp.writeStepAndEpisodeDataToCSV(fileName);

    }
}