using UnityEngine;

public class startEndPose : MonoBehaviour
{
    public Transform[] points; //Array di punti che compongono la traiettoria
    public AnimationCurve curve; //Curva di interpolazione
    public float animationTime = 2f; //Durata dell'animazione
    private float currentTime = 0f;
    public Quaternion startRotation;
    public Quaternion endRotation;
    private LineRenderer lineRenderer;
    //public GameObject tracePrefab;


    void Start()
    {
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.positionCount = points.Length;
        for (int i = 0; i < points.Length; i++)
        {
            lineRenderer.SetPosition(i, points[i].position);
        }
    }

    private void FixedUpdate()
    {
        currentTime += Time.deltaTime;
        if (currentTime > animationTime)
        {
            currentTime = 0f;
        }

        float t = currentTime / animationTime;
        float curveValue = curve.Evaluate(t);
        Vector3 position = GetInterpolatedPosition(curveValue);
        Quaternion rotation = Quaternion.Lerp(startRotation, endRotation, t);
        transform.position = position;
        transform.rotation = rotation;
    }

    void Update()
    {


        // Clona l'oggetto e posiziona il clone lungo la traiettoria
        //GameObject trace = Instantiate(tracePrefab, transform.position, transform.rotation);

        // Imposta la posizione del clone alla posizione corrente dell'oggetto
        //trace.transform.position = transform.position;
    }

    Vector3 GetInterpolatedPosition(float t)
    {
        if (points.Length == 0)
        {
            return transform.position;
        }
        if (points.Length == 1)
        {
            return points[0].position;
        }

        int numSections = points.Length - 1;
        int currPt = Mathf.Min(Mathf.FloorToInt(t * (float)numSections), numSections - 1);
        float u = t * (float)numSections - (float)currPt;

        Vector3 p0 = points[currPt].position;
        Vector3 p1 = points[currPt + 1].position;

        return Vector3.Lerp(p0, p1, u);
    }
}

