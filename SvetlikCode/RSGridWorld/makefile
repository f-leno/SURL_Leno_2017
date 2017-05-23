JFLAGS = -cp
CLASSPATH = ../lib/burlap.jar:.
JC = javac
.SUFFIXES: .java .class
.java.class:
	$(JC) $(JFLAGS) $(CLASSPATH) $*.java -Xlint:none

CLASSES = \
	RSGridWorld/TreeNode.java \
	RewardShape.java \
	RSGridWorld/RSGridWorld.java \
	RSGridWorld/RSGridWorldRewardFunction.java \
	RSGridWorld/RSGridWorldStateParser.java \
	RSGridWorld/RSGridWorldVisualizer.java \
	RSGridWorld/RSGridWorldAgentSpaceBuilder.java \
	RSGridWorld/RSGridWorldPotentialFunction.java \
	RSGridWorld/RSPotentialShapedRF.java \
	ThreadedExperiment.java \
	RSGridWorld/MergeAndParseCSV.java \
	RSGridWorld/BoundedLearningAlgorithmExperimenter.java \
	RSGridWorld/RSGridWorldTerminalFunction.java \
	RSGridWorld/DomainToFile.java \
	RSGridWorld/Coordinates.java \
	RSGridWorld/BiasedPolicy.java \
	RSGridWorld/RSGreedyQPolicy.java \
	RSGridWorld/LearningPair.java \
	RSGridWorld/RSGridWorldLearningRate.java \
	RSGridWorld/RSValueFunctionInit.java \
	RSGridWorld/DataAnalysis.java \
	RSGridWorld/CompareTask.java \
	RSGridWorld/Tuple.java

MISTAKE = \
	MistakeDriven.java

default: classes

mistake: $(MISTAKE:.java=.class)

classes: $(CLASSES:.java=.class)

clean:
	$(RM) *.class
	$(RM) RSGridWorld/*.class
	
cleanexp:
	$(RM) expData*.csv
