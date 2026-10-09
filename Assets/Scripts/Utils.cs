
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