/*
Il codice completo implementa l'algoritmo RRT per la pianificazione di un percorso, con la possibilità di specificare la posizione iniziale e finale,
il raggio massimo di curvatura, il diametro del catetere, la dimensione del passo, e il numero massimo di iterazioni.
*/

using UnityEngine;
using System.Collections.Generic;

public class RRTPathPlanner : MonoBehaviour
{
    public Transform start;
    public Transform goal;
    public float maxCurvature = 0.02618f; // K max (2.618*10^-2 mm^-1)
    public float diameter = 3.4f; //3.4 mm
    public float stepSize = 0.1f; //0.1 mm
    public int maxIterations = 10000;

    private List<Vector3> nodes;
    private List<int> edges;
    private Vector3 lastSample;
    private int goalNode;

    void Start()
    {
        Debug.Log("Avvio RRTPathPlanner");
        Debug.Log("nodes = new List<Vector3>();");
        nodes = new List<Vector3>();
        Debug.Log("edges = new List<int>();");
        edges = new List<int>();
        Debug.Log("nodes.Add(start.position);");
        nodes.Add(start.position);
        Debug.Log("lastSample = start.position;");
        lastSample = start.position;
    }

    void Update()
    {
        if (nodes.Count < 2 || goalNode < 0)
        {
            Debug.Log("chiama BuildRRT();");
            BuildRRT();
            
        }
        Debug.Log("chiama DrawRRT()");
        DrawRRT();
    }

    /*Il metod BuildRRT è il cuore dell'algoritmo RRT, che genera un albero di campionamento casuale e 
    * cerca di connettere il nodo più vicino a un nuovo campione valido.
    * Se il campione si connette con il nodo finale, viene creato un nuovo nodo finale e l'algoritmo termina.*/
    void BuildRRT()
    {
        Debug.Log("Metodo BuildRRT()");
        int i = 0;
        while (i < maxIterations)
        {
            Debug.Log("Chiama funzione GetRandomSample() e associa valore a sample");
            Vector3 sample = GetRandomSample();
            Debug.Log("Chiama funzione GetNearestNode(sample) e associa valore a nearest");
            int nearest = GetNearestNode(sample);
            Debug.Log("If chiama CheckEdge(nodes[nearest], sample), se sono entrambi true prosegue");
            if (CheckEdge(nodes[nearest], sample))
            {
                int newNode = AddNode(sample);
                AddEdge(nearest, newNode);

                if (CheckEdge(sample, goal.position))
                {
                    goalNode = AddNode(goal.position);
                    AddEdge(newNode, goalNode);
                    break;
                }
            }
            i++;
        }
    }

    /*Il metod GetRandomSample genera un campione casuale all'interno del diametro del robot, con una dimensione del passo specificata.*/
    Vector3 GetRandomSample()
    {
        Vector3 sample;
        float x = Random.Range(-diameter / 2f, diameter / 2f);
        float y = Random.Range(-diameter / 2f, diameter / 2f);
        float z = Random.Range(-diameter / 2f, diameter / 2f);

        sample = lastSample + new Vector3(x, y, z) * stepSize;
        lastSample = sample;

        return sample;
    }

    /*Il metod GetNearestNode trova il nodo più vicino al campione casuale.*/
    int GetNearestNode(Vector3 sample)
    {
        int nearest = 0;
        float minDistance = float.MaxValue;
        for (int i = 0; i < nodes.Count; i++)
        {
            float distance = Vector3.Distance(nodes[i], sample);
            if (distance < minDistance)
            {
                minDistance = distance;
                nearest = i;
            }
        }
        return nearest;
    }

    /*Il metod CheckEdge verifica se il percorso tra due punti è valido, verificando la collisione con gli oggetti nell'ambiente e la curvatura massima consentita.*/
    bool CheckEdge(Vector3 start, Vector3 end)
    {
        Vector3 direction = end - start;
        float distance = direction.magnitude;
        direction.Normalize();

        for (float i = 0; i <= distance; i += stepSize)
        {
            Vector3 point = start + direction * i;

            if (!CheckCurvature(point)) //condizioni di curvatura non rispettate (controllo curvatura = false)
            {
                return false;
            }

            if (CheckCollision(point)) //condizioni di collisione non rispettate (collisione = true)
            {
                return false;
            }
        }
        return true;
    }

    /*Il metod CheckCurvature calcola la curvatura tra tre punti e verifica se rientra nella curvatura massima consentita.*/
    bool CheckCurvature(Vector3 point)
    {
        Vector3 direction = point - nodes[GetNearestNode(point)];
        float distance = direction.magnitude;
        direction.Normalize();

        if (distance < stepSize)
        {
            return true;
        }

        for (float i = 0; i <= distance - stepSize; i += stepSize)
        {
            Vector3 a = nodes[GetNearestNode(point)];//i-1
            Vector3 b = a + direction * i;
            Vector3 c = a + direction * (i + stepSize);

            float curvature = CalculateCurvature(a, b, c);

            if (curvature > maxCurvature)
            {
                return false;
            }
        }
        return true;
    }

    /*Il metod CalculateCurvature calcola la curvatura tra tre punti*/
    float CalculateCurvature(Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 ab = (b - a).normalized;
        Vector3 cb = (b - c).normalized;
        float cosine = Vector3.Dot(ab, cb);

        if (cosine > 1.0f)
        {
            cosine = 1.0f;
        }
        else if (cosine < -1.0f)
        {
            cosine = -1.0f;
        }

        float angle = Mathf.Acos(cosine);

        if (angle < 1e-6f)
        {
            return 0.0f;
        }
        return angle / Vector3.Distance(a, c);
    }

    /*Metod CheckCollision: il codice esegue un raycast verso l'alto dal punto 
     * specificato, con una lunghezza di 0.2 unità. Se il raycast interseca il 
     * MeshCollider, significa che il punto si trova all'interno dell'oggetto 
     * ostacolo e la funzione restituisce true. 
     * Altrimenti, non c'è stata collisione e la funzione restituisce false.*/
    bool CheckCollision(Vector3 point)
    {
        // Crea un raycast che parte dalla posizione del robot e punta verso il punto campionato
        Vector3 direction = point - transform.position;
        Ray ray = new Ray(transform.position, direction);
        RaycastHit hit;

        // Verifica se il raycast interseca un oggetto con MeshCollider
        if (Physics.Raycast(ray, out hit, direction.magnitude))
        {
            MeshCollider meshCollider = hit.collider.GetComponent<MeshCollider>();
            if (meshCollider != null)
            {
                return true; // Collisione trovata
            }
        }
        return false; // Nessuna collisione trovata
    }


    int AddNode(Vector3 position)
    {
        nodes.Add(position);
        return nodes.Count - 1;
    }

    void AddEdge(int start, int end)
    {
        edges.Add(start);
        edges.Add(end);
    }

    void DrawRRT()
    {
        for (int i = 0; i < edges.Count; i += 2)
        {
            Debug.DrawLine(nodes[edges[i]], nodes[edges[i + 1]], Color.green);
        }
    }

    /* Questa funzione prende in input un oggetto della scena e restituisce il volume all'interno del quale deve svilupparsi la ricerca del percorso ottimale dell'RRT
    Bounds GetSearchVolume(GameObject searchVolumeObject)
    {
        // Otteniamo il MeshRenderer dell'oggetto di riferimento
        MeshRenderer searchVolumeMeshRenderer = searchVolumeObject.GetComponent<MeshRenderer>();

        // Se l'oggetto non ha un MeshRenderer, restituiamo una bounding box vuota
        if (searchVolumeMeshRenderer == null)
        {
            return new Bounds(Vector3.zero, Vector3.zero);
        }

        // Altrimenti, restituiamo la bounding box del MeshRenderer
        return searchVolumeMeshRenderer.bounds;
    }*/
}