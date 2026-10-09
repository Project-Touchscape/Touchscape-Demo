
using UnityEngine;

public static class Utils
{
    // Outer product: a * b^T
    public static Matrix4x4 Outer(Vector3 a, Vector3 b)
    {
        Matrix4x4 m = Matrix4x4.zero;

        m.m00 = a.x * b.x;
        m.m01 = a.x * b.y;
        m.m02 = a.x * b.z;

        m.m10 = a.y * b.x;
        m.m11 = a.y * b.y;
        m.m12 = a.y * b.z;

        m.m20 = a.z * b.x;
        m.m21 = a.z * b.y;
        m.m22 = a.z * b.z;

        return m;
    }

    // Unity does not provide scalar * Matrix4x4 operator overloads. Scale each
    // matrix element explicitly instead of relying on unsupported multiplication.
    public static Matrix4x4 Scale(Matrix4x4 matrix, float scalar)
    {
        Matrix4x4 result = Matrix4x4.zero;

        for (int row = 0; row < 4; row++)
        {
            for (int column = 0; column < 4; column++)
            {
                result[row, column] = matrix[row, column] * scalar;
            }
        }

        return result;
    }

    public static Matrix4x4 Add(Matrix4x4 left, Matrix4x4 right)
    {
        Matrix4x4 result = Matrix4x4.zero;

        for (int row = 0; row < 4; row++)
        {
            for (int column = 0; column < 4; column++)
            {
                result[row, column] = left[row, column] + right[row, column];
            }
        }

        return result;
    }

    public static Matrix4x4 Subtract(Matrix4x4 left, Matrix4x4 right)
    {
        Matrix4x4 result = Matrix4x4.zero;

        for (int row = 0; row < 4; row++)
        {
            for (int column = 0; column < 4; column++)
            {
                result[row, column] = left[row, column] - right[row, column];
            }
        }

        return result;
    }

    public static Matrix4x4 WorldInertiaMatrix(ArticulationBody body)
    {
        Quaternion worldRotation =
                body.transform.rotation * body.inertiaTensorRotation;
        Vector3 x = worldRotation * Vector3.right;
        Vector3 y = worldRotation * Vector3.up;
        Vector3 z = worldRotation * Vector3.forward;

        return Add(
            Add(
                Scale(Outer(x, x), body.inertiaTensor.x),
                Scale(Outer(y, y), body.inertiaTensor.y)),
            Scale(Outer(z, z), body.inertiaTensor.z));
    }

    public static void GetChainedMassProperties(ArticulationBody root, out float totalMass, out Vector3 com, out Matrix4x4 totalInertia)
    {
        ArticulationBody[] bodies =
            root.GetComponentsInChildren<ArticulationBody>();

        totalMass = 0f;
        Vector3 weightedCOM = Vector3.zero;

        foreach (var body in bodies)
        {
            float m = body.mass;
            totalMass += m;
            weightedCOM += m * body.worldCenterOfMass;
        }

        if (totalMass <= 0f)
        {
            com = root.worldCenterOfMass;
            totalInertia = Matrix4x4.zero;
            return;
        }

        com = weightedCOM / totalMass;
        totalInertia = Matrix4x4.zero;

        foreach (var body in bodies)
        {
            float m = body.mass;
            Vector3 bodyCOM = body.worldCenterOfMass;

            Matrix4x4 I =
                WorldInertiaMatrix(body);

            Vector3 r = bodyCOM - com;

            // Parallel-axis theorem:
            // I_com = I_bodyCOM + m * (|r|^2 * identity - r * r^T)
            Matrix4x4 parallelAxis =
                Scale(
                    Subtract(
                        Scale(Matrix4x4.identity, r.sqrMagnitude),
                        Outer(r, r)),
                    m);

            totalInertia = Add(totalInertia, Add(I, parallelAxis));
        }
    }

    public static float MomentOfInertiaAlongAxis(Matrix4x4 inertia, Vector3 axis)
    {
        if (axis.sqrMagnitude <= Mathf.Epsilon)
            return 0f;

        axis.Normalize();

        // The inertia tensor is a 3x3 tensor stored in the upper-left portion of
        // the Matrix4x4. For a unit torque along axis n, alpha = I^-1 * n.
        // The effective moment about that axis is:
        //
        //     I_axis = 1 / (n^T * I^-1 * n)
        //
        // This uses the complete tensor, including products of inertia, rather
        // than treating the tensor as three independent diagonal values.
        float a00 = inertia.m00;
        float a01 = inertia.m01;
        float a02 = inertia.m02;
        float a10 = inertia.m10;
        float a11 = inertia.m11;
        float a12 = inertia.m12;
        float a20 = inertia.m20;
        float a21 = inertia.m21;
        float a22 = inertia.m22;

        float cofactor00 = a11 * a22 - a12 * a21;
        float cofactor01 = a02 * a21 - a01 * a22;
        float cofactor02 = a01 * a12 - a02 * a11;
        float cofactor10 = a12 * a20 - a10 * a22;
        float cofactor11 = a00 * a22 - a02 * a20;
        float cofactor12 = a02 * a10 - a00 * a12;
        float cofactor20 = a10 * a21 - a11 * a20;
        float cofactor21 = a01 * a20 - a00 * a21;
        float cofactor22 = a00 * a11 - a01 * a10;

        float determinant =
            a00 * cofactor00 +
            a01 * cofactor10 +
            a02 * cofactor20;

        if (Mathf.Abs(determinant) <= Mathf.Epsilon)
            return 0f;

        Vector3 adjugateTimesAxis = new Vector3(
            cofactor00 * axis.x + cofactor10 * axis.y + cofactor20 * axis.z,
            cofactor01 * axis.x + cofactor11 * axis.y + cofactor21 * axis.z,
            cofactor02 * axis.x + cofactor12 * axis.y + cofactor22 * axis.z);

        float inverseInertiaAlongAxis = Vector3.Dot(axis, adjugateTimesAxis) / determinant;
        if (inverseInertiaAlongAxis <= Mathf.Epsilon)
            return 0f;

        return 1f / inverseInertiaAlongAxis;
    }

    public static Vector3 AngularVelocityFromQuaternions(Quaternion q1, Quaternion q2, float timeStep)
    {
        // Calculate the angular velocity from two quaternions
        Quaternion delta = Quaternion.Inverse(q1) * q2;
        delta.ToAngleAxis(out float angle, out Vector3 axis);
        if (angle > 180f) angle -= 360f; // Ensure shortest path
        return axis * (angle * Mathf.Deg2Rad / timeStep);
    }

    public static Vector3 MultiplyInertia(Matrix4x4 inertia, Vector3 vector)
    {
        return new Vector3(
            inertia.m00 * vector.x + inertia.m01 * vector.y + inertia.m02 * vector.z,
            inertia.m10 * vector.x + inertia.m11 * vector.y + inertia.m12 * vector.z,
            inertia.m20 * vector.x + inertia.m21 * vector.y + inertia.m22 * vector.z);
    }
}