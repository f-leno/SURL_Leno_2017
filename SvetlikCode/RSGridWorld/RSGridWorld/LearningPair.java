package RSGridWorld;
import burlap.behavior.singleagent.*;
import burlap.behavior.singleagent.learning.*;

public class LearningPair<EpisodeAnalysis,LearningAgent> {
    public EpisodeAnalysis ep;
    public LearningAgent la;

    public LearningPair(EpisodeAnalysis a, LearningAgent b) {
        this.ep = a;
        this.la = b;
    }
    public LearningPair(){}
    public void setAgent(LearningAgent agent){
    	this.la = agent;
    }
};