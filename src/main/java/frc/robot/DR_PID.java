package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;


// Implements the velocity algorithm for PID.
// CVn = CVn-1 + Kp*(En - En-1) + Ki*En*dt + Kd*(En -2*En-1 +En-2)/td
public class DR_PID extends PIDController {
  // Factor for "proportional" control
  protected double m_kp;
  // Factor for "integral" control
  protected double m_ki;
  // Factor for "derivative" control
  protected double m_kd;
  // The error range where "integral" control applies
  protected double m_iZone = Double.POSITIVE_INFINITY;
  // The period (in seconds) of the loop that calls the controller
  protected final double m_period;
  protected double m_maximumIntegral = 1.0;
  protected double m_minimumIntegral = -1.0;
  protected double m_maximumInput;
  protected double m_minimumInput;
  // Do the endpoints wrap around? e.g. Absolute encoder
  protected boolean m_continuous;
  // The error at the time of the most recent call to calculate()
  protected double m_error;
  protected double m_errorDerivative;
  // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
  protected double m_prevError = 0.0;
  protected double m_setpoint;
  protected double m_measurement;
  protected boolean m_haveMeasurement = false;
  protected boolean m_haveSetpoint;

  double m_output = 0.0;
  protected double m_prevOutput = 0.0;
  protected double m_prevMeas = 0.0;
  protected double m_prevPrevMeas = 0.0;

  public DR_PID(double kp, double ki, double kd) {
    super(kp, ki, kd, 0.02);
    this.m_period = 0.02;
  }
  public DR_PID(double kp, double ki, double kd, double period) {
    super(kp, ki, kd, period);
    this.m_period = period;
  }

  @Override
  public double calculate(double measurement) {
    m_measurement = measurement;
    m_prevOutput = m_output;

    if (m_haveMeasurement) {
      m_prevPrevMeas = m_prevMeas;
      m_prevMeas = m_measurement;
      m_prevError = m_error;
    } else {
      // First run, must initialize internal variables.
      m_prevPrevMeas = m_measurement;
      m_prevMeas = m_measurement;
    }
    m_haveMeasurement = true;

    if (m_continuous) {
      double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
      m_error = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
    } else {
      m_error = m_setpoint - m_measurement;
    }

    // CVn = CVn-1 + Kp*(En - En-1) + Ki*En*dt + Kd*(PVn -2*PVn-1 +PVn-2)/td
  
    // If the absolute value of the position error is greater than IZone, reset the total error
    double integral;
    if (Math.abs(m_error) > m_iZone) {
      integral = 0.0;
    } else {
      integral =
          MathUtil.clamp(
              m_ki * m_error * m_period,
              m_minimumIntegral,
              m_maximumIntegral);
    }

    double deltaError = (m_error - m_prevError);
    m_errorDerivative = deltaError / m_period;

    double secDerivMeas = (measurement -2* m_prevMeas + m_prevPrevMeas)/m_period;

    double derivative = MathUtil.clamp(
      m_kd * secDerivMeas,
      m_minimumIntegral, m_maximumIntegral);

    m_output = m_prevOutput + m_kp * deltaError + integral + derivative;
    return m_output;
  }

}
