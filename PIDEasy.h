#ifndef PIDEASY_H
#define PIDEASY_H

class PID {
  public:
    PID(float kp, float ki, float kd);

    float compute(float error, unsigned long dt);
    void reset();
    void setWindUP(float min, float max);
    void setConstrain(int min, int max);
    void setSmoothingDerivate(float sD);
    void setDampingFactor(float dF);

  private:
    float kp, ki, kd;
    float previous_error;
    float previous_derivative;
    float integral;
    float min_windup, max_windup;
    float derivative_smoothing, dampingFactor;
    int min_constrain, max_constrain;
};

#endif
