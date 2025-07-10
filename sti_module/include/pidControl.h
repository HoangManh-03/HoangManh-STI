namespace {
class PID{
    public: //publish use function 

        double caculate(double error){
            integral += error;
            double derivative = error - previous_error;
            previous_error = error;
            double output = kp * error + ki * integral + kd * derivative;
            return output;
        }
    private:    //private can use config variable
        double kp;
        double ki;
        double kd;
        double previous_error;
        double integral;
};
}