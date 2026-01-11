package frc.robot.MathUtils;

public class Polynomials {
    public static float newtonRaphson(float startVal, int iterations, float precisionOfValidity, float... coefficients) {
        float val = startVal;
        float newVal = 0;
        for (int i = 0; i < iterations; i++) {
            newVal = val - (polynomialEquation(val, coefficients) / polynomialEquation(val, coefficientsOfDerivative(coefficients)));

            float temp = val;
            val = newVal;
            newVal = temp;
        }
        if (Math.abs(polynomialEquation(newVal, coefficients)) < precisionOfValidity) return newVal;
        return -1000000000000000000000000f;
    }

    public static float polynomialEquation(float x, float... coefficients) {
        float sum = 0;
        for (int i = 0; i < coefficients.length; i++) {
            sum += coefficients[coefficients.length - 1 - i] * Math.pow(x, i);
        }
        return sum;
    }

    public static float[] coefficientsOfDerivative(float... coefficientsOfRegular) {
        float[] newCoeffs = new float[coefficientsOfRegular.length - 1];
        for (int i = 0; i < newCoeffs.length; i++) {
            newCoeffs[i] = (newCoeffs.length - i) * coefficientsOfRegular[i];
        }
        return newCoeffs;
    }
}
