#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
  public:
    KalmanFilter(float process_noise, float measurement_noise, float estimate_error, float initial_value) {
      Q = process_noise;
      R = measurement_noise;
      P = estimate_error;
      X = initial_value;
    }

    // 应用卡尔曼滤波来更新估计值
    float update(float measurement) {
      // 预测更新
      P = P + Q;

      // 卡尔曼增益
      float K = P / (P + R);

      // 估计更新
      X = X + K * (measurement - X);

      // 更新误差
      P = (1 - K) * P;

      return X;
    }

  private:
    float Q;  // 过程噪声
    float R;  // 测量噪声
    float P;  // 估计误差
    float X;  // 估计值
};

#endif
