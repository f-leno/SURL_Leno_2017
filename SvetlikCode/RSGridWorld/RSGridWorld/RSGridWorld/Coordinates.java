package RSGridWorld;

public class Coordinates {
    public double x;
    public double y;
    public String actionName;
    public String obstacleName;
    public Coordinates(double ix, double iy){
        x = ix;
        y = iy;
    }

    public Coordinates(double ix, double iy, String actionName){
        x = ix;
        y = iy;
        this.actionName = actionName;
    }

    public Coordinates(double ix, double iy, String actionName, String obstacleName){
        x = ix;
        y = iy;
        this.actionName = actionName;
        this.obstacleName = obstacleName;
    }
    public Coordinates(double ix, double iy, String obstacleName, boolean dummy){
        x = ix;
        y = iy;
        this.obstacleName = obstacleName;
    }
    public String getObstacleName(){
        return obstacleName;
    }
    public void setActionName(){
        this.actionName = null;
    }

    @Override
	public boolean equals(Object o) {
	    if (o == null) {
	        return false;
	    }
	    if (!(o instanceof Coordinates)) {
	        return false;
	    }
        if(obstacleName != null && actionName != null){
            return (x == ((Coordinates) o).x && y == ((Coordinates) o).y && actionName.equals(((Coordinates) o).actionName) && obstacleName.equals(((Coordinates)o).obstacleName));
        }
        if(obstacleName != null && actionName == null)
            return (x == ((Coordinates) o).x && y == ((Coordinates) o).y  && obstacleName.equals(((Coordinates)o).obstacleName));
	    if(actionName != null)
			return (x == ((Coordinates) o).x && y == ((Coordinates) o).y && actionName.equals(((Coordinates) o).actionName));
		return (x == ((Coordinates) o).x && y == ((Coordinates) o).y);
	}


   @Override
    public int hashCode() {
        int signx = x > 0 ? 1 : 0;
        int signy = y > 0 ? 1 : 0;
        String hash;
        if(actionName != null && obstacleName != null)
            hash = "" + signx + signy + Math.abs(x) + Math.abs(y) + actionName + obstacleName;
        else if(obstacleName != null && actionName == null)
            hash = "" + signx + signy + Math.abs(x) + Math.abs(y) + obstacleName;
        else if(actionName != null){
            hash = "" + signx + signy + Math.abs(x) + Math.abs(y) + actionName;
        }
        else
            hash = "" + signx + signy + Math.abs(x) + Math.abs(y);
        return hash.hashCode();
    }

    @Override
    public String toString(){
        String ret;
        if(actionName != null && obstacleName != null)
            ret = "x:" + x + " y:" + y + " action: " + actionName + "obstacle: " + obstacleName;
        else if(obstacleName != null && actionName == null)
            ret = "x:" + x + " y:" + y + " obstacle: " + obstacleName;
        else if(actionName != null)
            ret = "x:" + x + " y:" + y + " action: " + actionName;
        else
            ret = "x:" + x + "y: " + y;
        return ret;
    }

}
