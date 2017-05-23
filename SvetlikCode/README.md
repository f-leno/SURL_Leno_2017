# burlapcustomdomains
An unofficial fork of burlap. This repository hosts domains build on top of an existing BURLAP jar in an effort to maximize portability by containing all dependencies.

Current domains:
'RSGridWorld' : based on class GridWorldDomain, but with edits to all itself and all auxillery classes (RewardFunction, Visualizers, etc) to handle different types of ClassObjects of a predefined ClassInstance number.

Please consult the README file in the specific domain directory for instructions for building / compiling that domain.

Dependencies:
All BURLAP dependencies are contained inside the lib/burlap.jar. This repository is built off of BURLAP and its dependencies, thus this jar is the only necessary library component. It may be updated with the latest jar from the burlap website if desired (http://burlap.cs.brown.edu/) but be sure to download the precompiled jar WITH dependencies included.

For the author's repo and the most up-to-date code, visit https://github.com/jmacglashan/burlap
