package frc.robot.controller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;


// Implements the velocity algorithm for PID.
// CVn = CVn-1 + Kp*(En - En-1) + Ki*En*dt + Kd*(En -2*En-1 +En-2)/td
public class DR_PID extends PIDController {
  // Factor for "proportional" control
  private double m_kp;
  // Factor for "integral" control
  private double m_ki;
  // Factor for "derivative" control
  private double m_kd;
  // The error range where "integral" control applies
  private double m_iZone = Double.POSITIVE_INFINITY;
  // The period (in seconds) of the loop that calls the controller
  private final double m_period;
  private double m_maximumIntegral = 1.0;
  private double m_minimumIntegral = -1.0;
  private double m_maximumInput;
  private double m_minimumInput;
  // Do the endpoints wrap around? e.g. Absolute encoder
  private boolean m_continuous;
  // The error at the time of the most recent call to calculate()
  private double m_error;
  private double m_errorDerivative;
  // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
  private double m_prevError = 0.0;
	private double m_setpoint;
  private double m_measurement;
  private boolean m_haveMeasurement;
  private boolean m_haveSetpoint;

	double m_output = 0.0;
	private double m_prevOutput = 0.0;
	private double m_prevPrevError = 0.0;

	public DR_PID(double kp, double ki, double kd) {
		super(kp, ki, kd, 0.02);
		this.m_period = 0.02;
	}
	public DR_PID(double kp, double ki, double kd, double period) {
		super(kp, ki, kd, period);
		this.m_period = period;
	}

	// CVn = CVn-1 + Kp*(En - En-1) + Ki*En*dt + Kd*(En -2*En-1 +En-2)/td
	@Override
	public double calculate(double measurement) {
    m_measurement = measurement;
		m_prevPrevError = m_prevError;
    m_prevError = m_error;
    m_haveMeasurement = true;
		m_prevOutput = m_output;

    if (m_continuous) {
      double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
      m_error = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
    } else {
      m_error = m_setpoint - m_measurement;
    }

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

		double derivative = MathUtil.clamp(
			m_kd * (deltaError + m_prevPrevError - m_prevError)/m_period,
			m_minimumIntegral, m_maximumIntegral);

    m_output = m_prevOutput + m_kp * deltaError + integral + derivative;
		return m_output;
  }

}
