using MathNet.Numerics.LinearAlgebra;

namespace ByteTrackCSharp;

public static class MatrixUtil
{
    public static Matrix<float> StateMean() => Matrix<float>.Build.DenseIdentity(1, 8);
    public static Matrix<float> StateCov() => Matrix<float>.Build.DenseIdentity(8, 8);
    public static Matrix<float> StateHMean() => Matrix<float>.Build.DenseIdentity(1, 4);
    public static Matrix<float> StateHCov() => Matrix<float>.Build.DenseIdentity(4, 4);
    public static Matrix<float> Tlwh() => Matrix<float>.Build.DenseIdentity(1, 4);
    public static Matrix<float> Tlbr() => Matrix<float>.Build.DenseIdentity(1, 4);
    public static Matrix<float> Xyah() => Matrix<float>.Build.DenseIdentity(1, 4);
    public static Matrix<float> DetectBox() => Xyah();
}