package RSGridWorld;

/* Used to combine experiment output files using the multithreaded method
 * Needs the trial numbers to replace the dummy fill values the experiment ran
 *
 * 'baseName' should contain the relative path to the files eg: data/csvfilex.csv
 * !!**This has the explicit assumption that each trial is in a different csv file.**!!
 *
 * Author Maxwell J Svetlik
 */
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.FileWriter;
 
public class MergeAndParseCSV {
	String baseName;
	int numTrials;
	int numEpisodes;

	public MergeAndParseCSV(String baseName, int numTrials, int numEpisodes){
		this.baseName = baseName;
		this.numTrials = numTrials;
		this.numEpisodes = numEpisodes;
	}
 
  public String run() {
 	String mergedFileName = baseName + "Merged.csv";
	BufferedReader br = null;
	String line = "";
	String delim = ",";
 	FileWriter writer;
	try {
		writer = new FileWriter(mergedFileName);
 		for(int i = 0; i < numTrials; i++){
 			br = new BufferedReader(new FileReader(baseName+i+".csv"));
			line = br.readLine();
			if(i == 0){
				String[] temp = {line};
				addDelimiterAndWrite(writer,temp, delim);
			}
			for(int j = 0; j < numEpisodes * 2; j++){
				line = br.readLine();
				String[] splits = line.split(delim);
				splits[1] = Integer.toString(i);
				addDelimiterAndWrite(writer, splits, delim);
			}
		}
		writer.flush();
	 	writer.close();
	} catch (FileNotFoundException e) {
		e.printStackTrace();
	} catch (IOException e) {
		e.printStackTrace();
	} finally {
			if (br != null) {
				try {
					br.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		System.out.println("Done merging independant trials");
		return mergedFileName;
	}

	public void addDelimiterAndWrite(FileWriter bw, String[] line, String delim){
		try{
			for(int i = 0; i < line.length; i++){
				bw.append(line[i]);
				if(i != line.length -1)
					bw.append(delim);
			}
			bw.append("\n");
		}catch (IOException e) {
			e.printStackTrace();
		}
	}

}