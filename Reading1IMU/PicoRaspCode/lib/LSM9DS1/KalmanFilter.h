// Kalman Filter Class
class KalmanFilter
{
public:
    KalmanFilter() : angle(0), bias(0), P{{1, 0}, {0, 1}} {}

    float getAngle(float newAngle, float newRate, float dt)
    {
        // Predict the new angle and bias
        angle += dt * (newRate - bias);

        // Update estimation error covariance
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + 0.1); // Q angle
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += 0.1; // Q bias

        // Calculate Kalman Gain
        float S = P[0][0] + 1; // R is the measurement noise (set to 1)
        float K[2] = {P[0][0] / S, P[1][0] / S};

        // Update estimate with measurement
        float y = newAngle - angle; // y is the difference between the measured and predicted angle
        angle += K[0] * y;
        bias += K[1] * y;

        // Update error covariance
        P[0][0] -= K[0] * P[0][0];
        P[0][1] -= K[0] * P[0][1];
        P[1][0] -= K[1] * P[0][0];
        P[1][1] -= K[1] * P[0][1];

        return angle;
    }

private:
    float angle, bias;
    float P[2][2]; // Error covariance matrix
};