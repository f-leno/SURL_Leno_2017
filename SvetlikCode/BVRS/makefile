JFLAGS = -cp
CLASSPATH = ../lib/burlap.jar:../RSGridWorld/:.
JC = javac
.SUFFIXES: .java .class
.java.class:
	$(JC) $(JFLAGS) $(CLASSPATH) $*.java

CLASSES = \
	BVRS.java \
	BVRSRewardFunction.java \
	BVpicker.java

default: classes

classes: $(CLASSES:.java=.class)

clean:
	$(RM) *.class
