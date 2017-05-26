package RSGridWorld;


import java.util.Comparator;

public class Tuple<X extends Comparable<X>, Y extends Comparable<Y>, Z extends Comparable<Z> > { 
    public final X x; 
    public final Y y;
    public final Z z;
    public Tuple(X x, Y y, Z z) { 
        this.x = x; 
        this.y = y;
        this.z = z;
    }

    public X getX(){
        return x;
    }

    public Y getY(){
        return y;
    }

    public Z getZ(){
        return z;
    }
    @Override
    public String toString() {
        return "(" + x + "," + y + "," + z + ")";
    }

    @Override
    public boolean equals(Object other) {
        if (other == null) {
            return false;
        }
        if (other == this) {
            return true;
        }
        if (!(other instanceof Tuple)){
            return false;
        }
        Tuple<X,Y,Z> other_ = (Tuple<X,Y,Z>) other;
        return other_.x == this.x && other_.y == this.y && other_.z == this.z;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((x == null) ? 0 : x.hashCode());
        result = prime * result + ((y == null) ? 0 : y.hashCode());
        result = prime * result + ((z == null) ? 0 : z.hashCode());
        return result;
    }
}
