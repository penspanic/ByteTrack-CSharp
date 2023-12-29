using System;
using MathNet.Numerics.LinearAlgebra;

namespace ByteTrackCSharp;

public class Rect
{
    private readonly Matrix<float> tlwh;
    public Rect(Rect other)
    {
        tlwh = other.tlwh.Clone();
    }

    public Rect(float x, float y, float width, float height)
    {
        tlwh = Matrix<float>.Build.DenseOfRowMajor(1, 4, new float[] { x, y, width, height });
    }
    public float x() => tlwh[0, 0];
    public void setX(float value) => tlwh[0, 0] = value;
    public float y() => tlwh[0, 1];
    public void setY(float value) => tlwh[0, 1] = value;
    public float width() => tlwh[0, 2];
    public void setWidth(float value) => tlwh[0, 2] = value;
    public float height() => tlwh[0, 3];
    public void SetHeight(float value) => tlwh[0, 3] = value;
    public float br_x() => tlwh[0, 0] + tlwh[0, 2];
    public float br_y() => tlwh[0, 1] + tlwh[0, 3];

    public Matrix<float> getTlbr()
    {
        return Matrix<float>.Build.DenseOfRowMajor(1, 4, new float[4] { x(), y(), x() + width(), y() + height() });
    }

    public Matrix<float> getXyah()
    {
        return Matrix<float>.Build.DenseOfRowMajor(1, 4, new float[4] {
            x() + width() / 2,
            y() + height() / 2,
            width() / height(),
            height()
        });
    }

    public float calcIoU(Rect other)
    {

        float box_area = (other.width() + 1) * (other.height() + 1);
        float iw = Math.Min(x() + width(), other.x() + other.width()) - Math.Max(x(), other.x()) + 1;
        float iou = 0;
        if (iw > 0)
        {
            float ih = Math.Min(y() + height(), other.y() + other.height()) - Math.Max(y(), other.y()) + 1;
            if (ih > 0)
            {
                float ua = (x() + width() - x() + 1) * (y() + height() - y() + 1) + box_area - iw * ih;
                iou = iw * ih / ua;
            }
        }
        return iou;
    }

    public static Rect generate_rect_by_tlbr(Matrix<float> tlbr)
    {
        return new Rect(tlbr[0, 0], tlbr[0, 1], tlbr[0, 2] - tlbr[0, 0], tlbr[0, 3] - tlbr[0, 1]);
    }
    public static Rect generate_rect_by_xyah(Matrix<float> xyah)
    {

        var width = xyah[0, 2] * xyah[0, 3];
        return new Rect(xyah[0, 0] - width / 2, xyah[0, 1] - xyah[0, 3] / 2, width, xyah[0, 3]);
    }
}