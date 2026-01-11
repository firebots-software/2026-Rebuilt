package frc.robot.MathUtils;

public class Vector3 {
    public float x;
    public float y;
    public float z;

    public Vector3(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public float magnitude() {
        return (float) Math.sqrt((Math.pow(y, 2f) + Math.pow(y, 2f) + Math.pow(y, 2f)));
    }

    public void plus(Vector3 other) {
        x += other.x;
        y += other.y;
        z += other.z;
    }

    // public void mult(float m) {
    //     x *= m;
    //     y *= m;
    //     z *= m;
    // }

    public static Vector3 add(Vector3... a) {
        Vector3 newVec = new Vector3(0,0,0);
        for (Vector3 vec : a) {
            newVec.plus(vec);
        }  
        return newVec;
    }

    public static Vector3 mult(Vector3 a, float m) {
        return new Vector3(a.x * m, a.y * m, a.z * m);
    }
}
