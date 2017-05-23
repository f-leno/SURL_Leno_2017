import RSGridWorld.*;

/*
 * Mistake driven kicker class using the RSGridWorld domain. With many convinience methods already defined in the
 * RewardShape class, this class will utilize RewardShape objects.
 *
 * As the need arises, the RewardShape class can be remade here and be cut down, with any needed modifications.
 */

public class MistakeDriven {

     public static void main(String[] args) {
        //fill with desired obstacle types. supported are: rock, fire, pit
        //
        String [] obstacles = { RSGridWorld.OBSTACLEPIT};
        //define the number of obstacles. there will be numObstacles of each obstacle type
        int numObstacles = 5;
        String outputPath = "data/"; //directory to record results

        /*
         * VISUAL EXAMPLES
        */
        
        //Invoking RewardShape like this generates a random obstacle task, as defined by the variables set above.
        //obstalces = Obstacle types, num Obstacles = the number of EACH type of obstacle (ie total obstacles = obstacles.length * numObstacles)
        RewardShape example = new RewardShape(obstacles, numObstacles, 5, 5);

        //This example writes states to file, so they can be visualzied. The folling line visualizes all states that can be scrolled through
        //SarsaLearningExample(where states defined, number episodes, maxSteps)
        //example.SarsaLearningExample(outputPath, 2, 20);
        //example.visualize(outputPath);

        //This function runs many trials and plots various metrics as definied in the method
        //there are a few versions, but basically runs n episodes, x maxSteps, or a combination of either Qlearning or sarsalam
        //example.experimenterAndPlotter("expDataRF");


        //This example runs VIteration and displays a heatmap of the value function
        //example.ValueIterationExample(outputPath);

        /* 
         * Generate Domains and visualize a specified one
         */
        //generateDomainsToFile(obstacles);
        //example.SarsaLearningExample(outputPath, 2, 50);
        //example.visualize(outputPath);

        //example = new RewardShape("domains/nstep/target2.dat");
        //example.SarsaLearningExample(outputPath, 1, 1);
        //example.visualize(outputPath);

        //example = new RewardShape("domains/nstep/firepit1.dat");
        //example.SarsaLearningExample(outputPath, 1, 1);
        //example.visualize(outputPath);

        //example = new RewardShape("domains/nstep/fire2.dat");
        //example.SarsaLearningExample(outputPath, 1, 1);
        //example.visualize(outputPath);
    }


}