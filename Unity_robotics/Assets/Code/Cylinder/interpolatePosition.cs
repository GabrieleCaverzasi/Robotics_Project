using UnityEngine;

public class interpolatePosition : MonoBehaviour
{
    public Vector3[] points; //Array di punti che compongono la traiettoria
    public float animationTime = 2f; //Durata dell'animazione
    private float currentTime = 0f;

    void Update()
    {
        currentTime += Time.deltaTime;
        if (currentTime > animationTime)
        {
            currentTime = 0f;
        }

        float t = currentTime / animationTime;
        Vector3 position = GetInterpolatedPosition(t);
        transform.position = position;
    }

    Vector3 GetInterpolatedPosition(float t)
    {
        if (points.Length == 0)
        {
            return transform.position;
        }
        if (points.Length == 1)
        {
            return points[0];
        }

        int numSections = points.Length - 1;
        int currPt = Mathf.Min(Mathf.FloorToInt(t * (float)numSections), numSections - 1);
        float u = t * (float)numSections - (float)currPt;

        Vector3 p0 = points[currPt];
        Vector3 p1 = points[currPt + 1];
        Vector3 m0 = (currPt > 0) ? 0.5f * (points[currPt + 1] - points[currPt - 1]) : Vector3.zero;
        Vector3 m1 = (currPt < points.Length - 2) ? 0.5f * (points[currPt + 2] - points[currPt]) : Vector3.zero;

        return (2f * Mathf.Pow(u, 3f) - 3f * Mathf.Pow(u, 2f) + 1f) * p0 + (Mathf.Pow(u, 3f) - 2f * Mathf.Pow(u, 2f) + u) * m0 + (-2f * Mathf.Pow(u, 3f) + 3f * Mathf.Pow(u, 2f)) * p1 + (Mathf.Pow(u, 3f) - Mathf.Pow(u, 2f)) * m1;
    }
}
