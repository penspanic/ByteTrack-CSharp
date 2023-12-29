using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;

namespace ByteTrackCSharp;

public class KalmanFilter
{
    private float std_weight_position_;
    private float std_weight_velocity_;

    private Matrix<float> motion_mat_;
    private Matrix<float> update_mat_;

    public KalmanFilter() : this(std_weight_position: 1.0f / 20, std_weight_velocity: 1.0f / 160)
    {

    }

    public KalmanFilter(float std_weight_position, float std_weight_velocity)
    {
        std_weight_position_ = std_weight_position;
        std_weight_velocity_ = std_weight_velocity;
        const int ndim = 4;
        const float dt = 1;

        motion_mat_ = Matrix<float>.Build.DenseIdentity(8, 8);
        update_mat_ = Matrix<float>.Build.DenseIdentity(4, 8);

        for (int i = 0; i < ndim; i++)
        {
            motion_mat_[i, ndim + i] = dt;
        }
    }
    public void initiate(ref Matrix<float> mean, ref Matrix<float> covariance, Matrix<float> measurement)
    {
        mean.SetSubMatrix(0, 1, 0, 4, measurement.SubMatrix(0, 1, 0, 4));
        mean.SetSubMatrix(0, 1, 4, 4, Matrix<float>.Build.Dense(1, 4, 0.0f));

        Matrix<float> std = Matrix<float>.Build.Dense(1, 8);
        float m = measurement[0, 3];
        std[0, 0] = 2 * std_weight_position_ * m;
        std[0, 1] = 2 * std_weight_position_ * m;
        std[0, 2] = 1e-2f;
        std[0, 3] = 2 * std_weight_position_ * m;
        std[0, 4] = 10 * std_weight_velocity_ * m;
        std[0, 5] = 10 * std_weight_velocity_ * m;
        std[0, 6] = 1e-5f;
        std[0, 7] = 10 * std_weight_velocity_ * m;

        Matrix<float> tmp = std.PointwiseMultiply(std);
        covariance = Matrix<float>.Build.DenseDiagonal(8, 8, i => tmp[0, i]);
    }

    public void predict(ref Matrix<float> mean, ref Matrix<float> covariance)
    {
        var std = MatrixUtil.StateMean();
        float m = mean[0, 3];
        std[0, 0] = std_weight_position_ * m;
        std[0, 1] = std_weight_position_ * m;
        std[0, 2] = 1e-2f;
        std[0, 3] = std_weight_position_ * m;
        std[0, 4] = std_weight_velocity_ * m;
        std[0, 5] = std_weight_velocity_ * m;
        std[0, 6] = 1e-5f;
        std[0, 7] = std_weight_velocity_ * m;

        var tmp = std.PointwiseMultiply(std);
        var motion_cov = DiagonalMatrix.Build.DenseDiagonal(8, 8, i => tmp[0, i]);

        mean = (motion_mat_ * mean.Transpose()).Transpose();
        covariance = motion_mat_ * covariance * (motion_mat_.Transpose()) + motion_cov;
    }

    public void update(ref Matrix<float> mean, ref Matrix<float> covariance, Matrix<float> measurement)
    {
        var projected_mean = MatrixUtil.StateHMean();
        var projected_cov = MatrixUtil.StateHCov();
        project(ref projected_mean, ref projected_cov, mean, covariance);

        var B = (covariance * update_mat_.Transpose()).Transpose();
        var kalman_gain = (projected_cov.Cholesky().Solve(B)).Transpose();
        var innovation = measurement - projected_mean;



        var tmp = innovation * (kalman_gain.Transpose());
        mean = (mean + tmp);
        covariance = covariance - kalman_gain * projected_cov * (kalman_gain.Transpose());
    }

    public void project(ref Matrix<float> projected_mean, ref Matrix<float> projected_covariance,
        Matrix<float> mean, Matrix<float> covariance)
    {
        var std = Matrix<float>.Build.DenseOfArray(new float[,]
        {
            { std_weight_position_ * mean[0, 3] },
            { std_weight_position_ * mean[0, 3] },
            { 1e-1f },
            { std_weight_position_ * mean[0, 3] }
        });

        projected_mean = (update_mat_ * mean.Transpose()).Transpose();
        projected_covariance = update_mat_ * covariance * update_mat_.Transpose();

        Matrix<float> diag = DiagonalMatrix.Build.DenseDiagonal(4, 4, i => std[i, 0]);
        Matrix<float> square = diag.PointwiseMultiply(diag);
        projected_covariance += square;
    }
}