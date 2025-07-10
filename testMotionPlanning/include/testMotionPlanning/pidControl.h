namespace GoalControl{
    class PID {
    public:
        PID(double p, double i, double d);
        double calculate(double error);

    private:
        double kp;
        double ki;
        double kd;
        double integral;
        double previous_error;
    };

}
