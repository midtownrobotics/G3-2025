package frc.lib.team1648;
// just so no error
public class Constraints<T> {

    private T max, min;

    public Constraints(T max, T min) {
        this.max = max;
        this.min = min;
    }

    public T clamp(T value) {
        return value;
    }
    
}
