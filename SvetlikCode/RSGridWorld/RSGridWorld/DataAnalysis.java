 package RSGridWorld;
 import java.io.*;
 import java.util.HashMap;

public class DataAnalysis{

    protected HashMap<Integer, Double> mean;
    protected HashMap<Integer, Double> var;
    protected String path;
    protected double area = 0.0;
    protected int trainingOffset = 0;
    public DataAnalysis(String path, int trainingOffset){
        mean = new HashMap<Integer, Double>();
        this.path = path;
        this.trainingOffset = trainingOffset;
    }
    public void parseData(){
        BufferedReader br = null;
        String line = "";
        int count=1;

        try{
            br = new BufferedReader(new FileReader(path));
            try {
                line = br.readLine(); //clear the first line, containing headers etc
                int lastTrial = 0;
                while ((line = br.readLine()) != null) {

                    String[] country = line.split(",");
                    int curTrial = Integer.parseInt(country[1]);
                    if(lastTrial != curTrial){
                        count++;
                        lastTrial = curTrial;
                    }
                    int step=Integer.parseInt(country[2])/10;
                    double value = Double.parseDouble(country[3]);
                    if(!mean.containsKey(step)){
                        mean.put(step,value);
                    }
                    else{
                        count++;
                        double old = mean.get(step);
                        mean.put(step, value+old);
                    }
                }
            } catch (NumberFormatException | IOException e) {
                System.out.println("Some shit happened during data analysis");
                e.printStackTrace();
            }

        } catch (FileNotFoundException e) {
            e.printStackTrace();
            System.out.println("File doesn't exist.");
        }

        System.out.println(count);
        for(Integer e : mean.keySet()){
            double newVal = mean.get(e)/count;
            if(e >= trainingOffset){
                area += newVal;
            }
            mean.put(e, newVal);
        }
    }
    public HashMap<Integer, Double> getMean(){
        return mean;
    }
    public double getArea(){
        return area;
    }
}
