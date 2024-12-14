#include "filter.h"
#include <Eigen.h>
#include <math.h>

using namespace Eigen;

static const int n_x = 6; // x,y,z,vx,vy,vz
static const int n_z = 1; // measuring only altitude (z)
static VectorXf x_mean; 
static MatrixXf P;

static MatrixXf Q; // Process noise
static float R = 2.0f; // Measurement noise for altitude (tune as needed)

static bool initialized = false;

void ekfInit(float x, float y, float z, float vx, float vy, float vz) {
    x_mean = VectorXf(n_x);
    x_mean << x, y, z, vx, vy, vz;

    P = MatrixXf::Identity(n_x,n_x)*0.1f;

    Q = MatrixXf::Zero(n_x,n_x);
    // Process noise (tune as needed):
    // Assume some noise in acceleration integration
    Q(0,0)=0.01f; Q(1,1)=0.01f; Q(2,2)=0.01f;
    Q(3,3)=0.1f; Q(4,4)=0.1f; Q(5,5)=0.1f;

    initialized = true;
}

void ekfPredict(float ax, float ay, float az, float dt) {
    if (!initialized) return;

    // State:
    // x_mean = [x, y, z, vx, vy, vz]^T
    float x = x_mean(0);
    float y = x_mean(1);
    float z = x_mean(2);
    float vx = x_mean(3);
    float vy = x_mean(4);
    float vz = x_mean(5);

    // Predict new state
    float x_new = x + vx*dt;
    float y_new = y + vy*dt;
    float z_new = z + vz*dt;
    float vx_new = vx + ax*dt;
    float vy_new = vy + ay*dt;
    float vz_new = vz + az*dt;

    VectorXf x_pred(n_x);
    x_pred << x_new, y_new, z_new, vx_new, vy_new, vz_new;

    // State transition Jacobian F
    MatrixXf F = MatrixXf::Identity(n_x,n_x);
    F(0,3)=dt;
    F(1,4)=dt;
    F(2,5)=dt;

    // For acceleration terms:
    // velocities depend on accelerations, but we treat them as inputs, not states.
    // F is identity except for these linear terms already considered.

    // Update covariance
    P = F*P*F.transpose() + Q;

    x_mean = x_pred;
}

void ekfUpdate(float alt_meas) {
    if (!initialized) return;

    // Measurement: z only
    float z_pred = x_mean(2);

    float y = alt_meas - z_pred; // residual

    // Measurement matrix H: z = [0 0 1 0 0 0] x
    Matrix<float,1,6> H;
    H << 0,0,1,0,0,0;

    // S = H P H^T + R
    float S = (H*P*H.transpose())(0,0) + R;
    // Kalman gain K = P H^T S⁻¹
    MatrixXf K = P*H.transpose()*(1.0f/S);

    x_mean = x_mean + K*y;
    P = P - K*H*P;
}

void ekfGetState(float &x, float &y, float &z, float &vx, float &vy, float &vz) {
    x = x_mean(0);
    y = x_mean(1);
    z = x_mean(2);
    vx = x_mean(3);
    vy = x_mean(4);
    vz = x_mean(5);
}
