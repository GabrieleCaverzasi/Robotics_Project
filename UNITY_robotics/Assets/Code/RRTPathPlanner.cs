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
    public float Curvature = 0.02618f; // K max (2.618*10^-2 mm^-1)
    public float diameter_chateter = 3.4f; //3.4 mm
    public int maxIterations = 100;
    public float stepSize_chateter = 10f; //10 mm, distanza tra due "joint"
    private float RealToUnity = 0.735294f; //Questo parametro permette di convertire i valori reali in quelli della scena di unity

    private List<Vector3> nodes;
    private List<int> edges;
    private Vector3 lastSample;
    private int goalNode;
    private bool stop = false;
    private bool firstSample = true; 

    void Start()
    {
        Debug.Log("Avvio RRTPathPlanner");
        nodes = new List<Vector3>();
        edges = new List<int>();
        nodes.Add(start.position);
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

        if (stop)
        {
            Application.Quit();
        }
    }


    /*Il metod BuildRRT è il cuore dell'algoritmo RRT, che genera un albero di campionamento casuale e 
    * cerca di connettere il nodo più vicino a un nuovo campione valido.
    * Se il campione si connette con il nodo finale, viene creato un nuovo nodo finale e l'algoritmo termina.*/

    void BuildNewRTT()
    {
        Debug.Log("Metodo BuildRRT()");
        int i = 0;
        bool found = false;
        do
        {
            Vector3 sample = GetRandomSample();
            if(CheckRandomSample(sample)){
                int nearest = GetNearestNode(sample);
                if (CheckEdge(nodes[nearest], sample))
                {
                    int newNode = AddNode(sample);
                    AddEdge(nearest, newNode);

                    if (CheckEdge(sample, goal.position))
                    {
                        goalNode = AddNode(goal.position);
                        AddEdge(newNode, goalNode);
                        //Debug.Log("Punto Trovato");
                        found = true;
                    }
                }
            }
            i++;
        }while (found == false || i<maxIterations);
        Debug.Log("found = " + found);
        Debug.Log("Numero iterazioni = " + i);
    }

    void BuildRRT()
    {
        Debug.Log("Metodo BuildRRT()");
        int i = 0;

        while (i < maxIterations)
        {
            Vector3 sample = GetRandomSample(); //viene generato un punto casuale (sample)
            if (CheckRandomSample(sample))
            {
                int nearest = GetNearestNode(sample); //viene cercato il nodo più vicino all'interno del grafo
                if (CheckEdge(nodes[nearest], sample))
                {
                    int newNode = AddNode(sample);
                    AddEdge(nearest, newNode);

                    if (CheckEdge(sample, goal.position))
                    {
                        goalNode = AddNode(goal.position);
                        AddEdge(newNode, goalNode);
                        Debug.Log("Punto Trovato (Break), Iterazioni necessarie = " + i);
                        break;
                    }
                }
            }
            i++;
        }
        if (i == maxIterations)
        {
            stop = true;
            Debug.Log("STOP, MAX_ITERATIONS");
        }
    }
    /* spiegazione BuildRRT() nel dettaglio
     * La funzione utilizza un ciclo while che viene eseguito per un massimo di maxIterations volte. 
     * Ad ogni iterazione, viene generato un punto casuale (sample) tramite la funzione GetRandomSample(). 
     * Successivamente, viene cercato il nodo più vicino all'interno del grafo con la funzione GetNearestNode(sample).
     * Se l'arco tra il nodo più vicino e il punto casuale rispetta le condizioni di curvatura e di collisione, viene 
     * creato un nuovo nodo all'interno del grafo (AddNode(sample)) e l'arco viene creato tra il nodo più vicino e il nuovo nodo (AddEdge(nearest, newNode)). 
     * Se l'arco appena creato raggiunge la posizione del goal (goal.position) rispettando le condizioni di curvatura e di collisione, 
     * viene creato un nuovo nodo goal e viene creato un arco tra il nuovo nodo e il nodo goal.
     * Il ciclo while termina quando viene raggiunto il goal o quando il numero massimo di iterazioni è stato raggiunto.
     */


    /*Il metod GetRandomSample genera un campione casuale all'interno del diametro del robot, con una dimensione del passo specificata.*/
    Vector3 GetRandomSample()
    {
        // Dichiarazione di una variabile di tipo Vector3 per il campione casuale
        Vector3 sample;
        float diameter = diameter_chateter * RealToUnity; 
        // Genera tre valori casuali per le coordinate x, y e z del punto, rispettivamente
        float x = Random.Range(-diameter / 2f, diameter / 2f);
        float y = Random.Range(-diameter / 2f, diameter / 2f);
        float z = Random.Range(-diameter / 2f, diameter / 2f);

        // Calcola le coordinate del punto aggiungendo al campione precedente (lastSample) un nuovo vettore che ha
        // come componenti i valori casuali generati moltiplicati per la dimensione dello step (stepSize)
        float stepSize = stepSize_chateter * RealToUnity; 
        sample = lastSample + new Vector3(x, y, z) * stepSize;

        // Aggiorna la variabile lastSample con le coordinate del campione appena generato
        //lastSample = sample;

        // Restituisce il campione casuale come risultato della funzione
        return sample;
    }
    /*Passaggi GetRandomSample() nel dettaglio:
     * Vector3 sample; - Dichiarazione di una variabile di tipo Vector3 per il campione casuale che verrà generato.
     * float x = Random.Range(-diameter / 2f, diameter / 2f); - Genera un valore casuale per la coordinata x del punto, compreso tra -diameter / 2f e diameter / 2f, utilizzando il metodo Random.Range() fornito da Unity.
     * float y = Random.Range(-diameter / 2f, diameter / 2f); - Genera un valore casuale per la coordinata y del punto, compreso tra -diameter / 2f e diameter / 2f.
     * float z = Random.Range(-diameter / 2f, diameter / 2f); - Genera un valore casuale per la coordinata z del punto, compreso tra -diameter / 2f e diameter / 2f.
     * sample = lastSample + new Vector3(x, y, z) * stepSize; - Calcola le coordinate del punto aggiungendo al campione precedente (lastSample) un nuovo vettore che ha come componenti i valori casuali generati moltiplicati per la dimensione dello step (stepSize).
     * lastSample = sample; - Aggiorna la variabile lastSample con le coordinate del campione appena generato, in modo che il prossimo campione possa essere generato a partire da questa posizione.
     * return sample; - Restituisce il campione casuale come risultato della funzione.
    */

    //variazione della funzione, aggiungendo il fatto che i nodi si devono avvicinare al goal
    Vector3 GetRandomSample_var1() 
    {
        // Dichiarazione di una variabile di tipo Vector3 per il campione casuale
        Vector3 sample;
        float diameter = diameter_chateter * RealToUnity;

        // Calcola la distanza tra lastSample e il goal
        float distanceToGoal = Vector3.Distance(lastSample, goal.position);

        // Genera tre valori casuali per le coordinate x, y e z del punto, rispettivamente
        float x = Random.Range(-diameter / 2f, diameter / 2f);
        float y = Random.Range(-diameter / 2f, diameter / 2f);
        float z = Random.Range(-diameter / 2f, diameter / 2f);

        // Calcola le coordinate del punto aggiungendo al campione precedente (lastSample) un nuovo vettore che ha
        // come componenti i valori casuali generati moltiplicati per la dimensione dello step (stepSize)
        float stepSize = stepSize_chateter * RealToUnity;
        sample = lastSample + new Vector3(x, y, z) * stepSize;

        // Calcola la distanza tra il campione casuale e il goal
        float distanceToSample = Vector3.Distance(sample, goal.position);

        do
        {
            // Genera tre valori casuali per le coordinate x, y e z del punto, rispettivamente
            x = Random.Range(-diameter / 2f, diameter / 2f);
            y = Random.Range(-diameter / 2f, diameter / 2f);
            z = Random.Range(-diameter / 2f, diameter / 2f);

            // Calcola le coordinate del punto aggiungendo al campione precedente (lastSample) un nuovo vettore che ha
            // come componenti i valori casuali generati moltiplicati per la dimensione dello step (stepSize)
            stepSize = stepSize_chateter * RealToUnity;
            sample = lastSample + new Vector3(x, y, z) * stepSize;

            // Calcola la distanza tra il campione casuale e il goal
            distanceToSample = Vector3.Distance(sample, goal.position);

            // Continua a generare campioni casuali fino a quando non se ne trova uno che ha una distanza minore dal goal rispetto a lastSample
        } while (distanceToSample >= distanceToGoal);

        // Aggiorna la variabile lastSample con le coordinate del campione appena generato
        lastSample = sample;

        // Restituisce il campione casuale come risultato della funzione
        return sample;
    }
    Vector3 GetRandomSample_var2() 
    {
        Vector3 sample;
        float diameter = diameter_chateter * RealToUnity;
        float stepSize = stepSize_chateter * RealToUnity;
        float maxDistance = Vector3.Distance(goal.position, lastSample);

        do
        {
            float x = Random.Range(-diameter / 2f, diameter / 2f);
            float y = Random.Range(-diameter / 2f, diameter / 2f);
            float z = Random.Range(-diameter / 2f, diameter / 2f);

            sample = lastSample + new Vector3(x, y, z) * stepSize;
        } while (Vector3.Distance(goal.position, sample) > maxDistance);

        lastSample = sample;

        return sample;
    }


    bool CheckRandomSample(Vector3 sample) //funzione che controlla se ci stiamo avvicinando al gol
    {
        float maxDistance = Vector3.Distance(goal.position, lastSample);
        if(Vector3.Distance(goal.position, sample) > maxDistance)
        {
            return false;
        }
        if (firstSample)
        {
            if (CheckFirstDirection(start, sample)==false)
            {
                return false; //direzione del sample non è conforme a quella dei due oggetti di riferimento.
            }
            // Trovato il primo sample, check sulla direzione non più necessario
            firstSample = false;
        }
        // Aggiorna la variabile lastSample con le coordinate del campione generato validato
        lastSample = sample;
        return true;
    }


    bool CheckFirstDirection(Transform start, Vector3 Firstsample)
    {
        // Recupera gli oggetti di riferimento per la direzione
        GameObject directionObject1 = GameObject.Find("Post-puntura-PRE");
        GameObject directionObject2 = GameObject.Find("Post-puntura-POST");

        if (directionObject1 == null || directionObject2 == null)
        {
            Debug.LogError("Non è stato possibile trovare gli oggetti di riferimento per la direzione.");
            return false;
        }

        // Ottiene la direzione dei due oggetti di riferimento
        Vector3 direction = (directionObject2.transform.position - directionObject1.transform.position).normalized;

        // Calcola la direzione tra il punto di partenza e il primo sample generato
        Vector3 sampleDirection = (Firstsample - start.position).normalized;

        // Verifica che le due direzioni siano simili, entro una soglia prestabilita
        float angleThreshold = 5f; // soglia di tolleranza dell'angolo (in gradi)
        float angle = Vector3.Angle(sampleDirection, direction);
        if (angle < angleThreshold)
        {
            return true;
        }
        else
        {
            Debug.Log("La direzione del sample non è conforme a quella dei due oggetti di riferimento.");
            return false;
        }
    }



    /*Il metod GetNearestNode trova il nodo più vicino al campione casuale.*/
    int GetNearestNode(Vector3 sample)
    {
        // Inizializza la variabile nearest con l'indice del primo nodo (0)
        int nearest = 0;

        // Inizializza la variabile minDistance con il valore massimo di un float
        float minDistance = float.MaxValue;

        // Itera su tutti i nodi nella lista dei nodi
        for (int i = 0; i < nodes.Count; i++)
        {
            // Calcola la distanza tra il nodo corrente e il campione casuale utilizzando il metodo Vector3.Distance()
            float distance = Vector3.Distance(nodes[i], sample);

            // Se la distanza è minore della distanza minima finora registrata, aggiorna la variabile minDistance
            // e la variabile nearest con l'indice del nodo corrente
            if (distance < minDistance)
            {
                minDistance = distance;
                nearest = i;
            }
        }
        // Restituisce l'indice del nodo più vicino al campione casuale
        return nearest;
    }
    /* Passaggi GetNearestNode() nel dettaglio:
     * int nearest = 0; - Inizializzazione della variabile nearest con l'indice del primo nodo (0).
     * float minDistance = float.MaxValue; - Inizializzazione della variabile minDistance con il valore massimo di un float, in modo da avere una distanza iniziale più grande di qualsiasi distanza possibile tra un nodo e il campione casuale.
     * for (int i = 0; i < nodes.Count; i++) - Ciclo che itera su tutti i nodi nella lista dei nodi.
     * float distance = Vector3.Distance(nodes[i], sample); - Calcolo della distanza tra il nodo corrente e il campione casuale utilizzando il metodo Vector3.Distance(), che restituisce la distanza euclidea tra due punti in uno spazio tridimensionale.
     * if (distance < minDistance) - Verifica se la distanza tra il nodo corrente e il campione casuale è minore della distanza minima finora registrata.
     * minDistance = distance; - Se la distanza è minore della distanza minima finora registrata, aggiorna la variabile minDistance con il nuovo valore della distanza.
     * nearest = i; - Aggiorna la variabile nearest con l'indice del nodo corrente, che è quello più vicino al campione casuale finora trovato.
     * return nearest; - Restituisce l'indice del nodo più vicino al campione casuale.
    */


    /*Il metod CheckEdge verifica se il percorso tra due punti è valido, verificando la collisione con gli oggetti nell'ambiente e la curvatura massima consentita.*/
    bool CheckEdge(Vector3 start, Vector3 end)
    {
        // Calcola la direzione dell'edge
        Vector3 direction = end - start;

        // Calcola la lunghezza dell'edge
        float distance = direction.magnitude;

        // Normalizza la direzione dell'edge
        direction.Normalize();

        // Itera lungo l'edge con passi di dimensione stepSize
        float stepSize = stepSize_chateter * RealToUnity; 
        for (float i = 0; i <= distance; i += stepSize)
        {
            // Calcola il punto corrispondente all'i-esimo passo lungo l'edge
            Vector3 point = start + direction * i;

            // Verifica se il punto rispetta le condizioni di curvatura
            if (!CheckCurvature(point))
            {
                // Se il punto non rispetta le condizioni di curvatura, restituisce false
                return false;
            }

            // Verifica se il punto rispetta le condizioni di collisione
            if (CheckForObstacle(start, end))
            {
                // Se il punto rispetta le condizioni di collisione, restituisce false
                return false;
            }
        }
        // Se tutti i punti lungo l'edge rispettano le condizioni di curvatura e collisione, restituisce true
        return true;
    }
    /*Passaggi CheckEdge() nel dettaglio:
     * Vector3 direction = end - start; - Calcolo della direzione dell'edge, che corrisponde alla differenza tra il nodo di partenza start e il nodo di arrivo end.
     * float distance = direction.magnitude; - Calcolo della lunghezza dell'edge, utilizzando il metod magnitude() che restituisce la lunghezza del vettore direction.
     * direction.Normalize(); - Normalizzazione della direzione dell'edge, utilizzando il metod Normalize() che restituisce un vettore della stessa direzione ma di lunghezza unitaria.
     * for (float i = 0; i <= distance; i += stepSize) - Ciclo che itera lungo l'edge con passi di dimensione stepSize, partendo dal nodo di partenza start e arrivando al nodo di arrivo end.
     * Vector3 point = start + direction * i; - Calcolo del punto corrispondente all'i-esimo passo lungo l'edge, utilizzando la formula punto = puntoIniziale + direzione * passo.
     * if (!CheckCurvature(point)) - Verifica se il punto corrente rispetta le condizioni di curvatura, utilizzando la funzione CheckCurvature() che restituisce true se il punto rispetta le condizioni di curvatura, false altrimenti.
     * return false; - Se il punto corrente non rispetta le condizioni di curvatura, restituisce false, indicando che l'edge non rispetta le condizioni di curvatura.
     * if (CheckCollision(point)) - Verifica se il punto corrente rispetta le condizioni di collisione, utilizzando la funzione CheckCollision() 
    /*


    /*Il metod CheckCurvature calcola la curvatura tra tre punti e verifica se rientra nella curvatura massima consentita.*/
    bool CheckCurvature(Vector3 point)
    {
        Vector3 direction = point - nodes[GetNearestNode(point)];
        float distance = direction.magnitude;
        direction.Normalize();
        float stepSize = stepSize_chateter * RealToUnity;
        if (distance < stepSize) //punto non coerente con le condizioni di curvatura
        {
            return true;
        }

        for (float i = 0; i <= distance - stepSize; i += stepSize)
        {
            //ottengo i tre punti per calcolare la curvatura
            Vector3 a = nodes[GetNearestNode(point)];//i-1
            Vector3 b = a + direction * i;
            Vector3 c = a + direction * (i + stepSize);

            float curvature = CalculateCurvature2(a, b, c); //richiamo il calcolo della curvatura
            float maxCurvature = Curvature * RealToUnity;
            if (curvature > maxCurvature)
            {
                return false;
            }
        }
        return true;
    }
    /* Passaggi CheckCurvature() nel dettaglio:
     * La funzione inizia calcolando la direzione dal punto dato point al nodo più vicino sulla traiettoria nodes[GetNearestNode(point)].
     * Viene quindi calcolata la distanza distance tra il punto point e il nodo più vicino, e normalizza la direzione.
     * Viene effettuato un controllo: se la distanza tra il punto e il nodo più vicino è minore dello stepSize (che rappresenta la lunghezza di ogni segmento di percorso), la funzione restituisce true.
     * Se la distanza è maggiore dello stepSize, la funzione procede con un ciclo for che scorre i segmenti del percorso tra il nodo più vicino e il punto dato point. 
     * In ogni iterazione viene calcolata la curvatura del segmento di percorso che passa attraverso i tre punti corrispondenti ai nodi precedente (i-1), corrente (i), e successivo (i+1) al punto point.
     * Se la curvatura supera la curvatura massima consentita maxCurvature, la funzione restituisce true.
     * Se la funzione arriva alla fine del ciclo for senza restituire true, il punto soddisfa le condizioni di curvatura e la funzione restituisce false.
    */


    float CalculateCurvature2(Vector3 a, Vector3 b, Vector3 c)
    {
        // Calcola i vettori che vanno dai punti A al punto B e al punto C
        Vector3 ab = b - a;
        Vector3 ac = c - a;

        // Calcola il vettore normale al piano che contiene le due rette passanti per AB e AC
        Vector3 normal = Vector3.Cross(ab, ac);

        // Calcola il punto medio dei segmenti AB e AC
        Vector3 midpointAB = (a + b) / 2;
        Vector3 midpointAC = (a + c) / 2;

        // Calcola il vettore normale al piano che contiene i punti A, B e C
        Vector3 planeNormal = Vector3.Cross(ab, normal);

        // Calcola la distanza tra il punto medio dei segmenti AB e AC e il piano che passa per i punti A, B e C
        float distance = Vector3.Dot(planeNormal, midpointAB - midpointAC) / Vector3.Dot(planeNormal, planeNormal);

        // Calcola le coordinate del centro del cerchio
        Vector3 center = midpointAB + planeNormal * distance;

        // Calcola la distanza tra il centro del cerchio e uno dei punti, che rappresenta il raggio
        float radius = Vector3.Distance(center, a);

        // Calcola la curvatura k come 1/raggio
        float k = 1 / radius;

        return k; 
    }
    /* Passaggi CalculateCurvature2() nel dettaglio:
     * Viene calcolato il vettore AB e il vettore AC, ovvero i vettori che vanno dal punto A ai punti B e C.
     * Viene calcolato il vettore normale al piano che contiene le due rette passanti per AB e AC, utilizzando il prodotto vettoriale tra AB e AC.
     * Viene calcolato il punto medio dei segmenti AB e AC.
     * Viene calcolato il vettore normale al piano che contiene i punti A, B e C, utilizzando il prodotto vettoriale tra AB e il vettore normale al piano calcolato al passaggio precedente.
     * Viene calcolata la distanza tra il punto medio dei segmenti AB e AC e il piano che passa per i punti A, B e C, utilizzando il prodotto scalare tra il vettore normale al piano e il vettore che va dal punto medio di AB a quello di AC, diviso la norma quadra del vettore normale al piano.
     * Vengono calcolate le coordinate del centro del cerchio, sommando al punto medio di AB il prodotto tra il vettore normale al piano e la distanza calcolata al passaggio precedente.
     * Viene calcolata la distanza tra il centro del cerchio e uno dei punti, che rappresenta il raggio.
     * Infine, viene calcolata la curvatura k come l'inverso del raggio.
    */


    bool CheckForObstacle(Vector3 start, Vector3 end)
    {
        // Cerca tutti gli oggetti con il tag "Obstacle" nella scena Unity
        GameObject[] obstacles = GameObject.FindGameObjectsWithTag("Obstacle");

        foreach (GameObject obstacle in obstacles)
        {
            // Se l'oggetto è un collider, controlla se la retta che collega i due punti interseca il collider
            Collider collider = obstacle.GetComponent<Collider>();
            if (collider != null)
            {
                RaycastHit hit;
                if (Physics.Linecast(start, end, out hit))
                {
                    if (hit.collider == collider)
                    {
                        // La retta interseca il collider, quindi attraversa un ostacolo
                        return true;
                    }
                }
            }
        }
        // La retta non interseca alcun ostacolo
        return false;
    }
    /* Passaggi CheckForObstacle) nel dettaglio
     * La funzione CheckForObstacle prende come argomenti due punti nello spazio 3D, 
     * start e end, e restituisce un valore booleano che indica se la retta che collega 
     * i due punti interseca un oggetto nella scena Unity con il tag "Obstacle".
     * Per cercare tutti gli oggetti con il tag "Obstacle" nella scena Unity, 
     * la funzione utilizza la funzione GameObject.FindGameObjectsWithTag, 
     * che restituisce un array di tutti gli oggetti con il tag specificato.
     * Per ogni oggetto trovato con il tag "Obstacle", la funzione controlla 
     * se l'oggetto ha un collider associato. In caso affermativo, 
     * la funzione utilizza la funzione Physics.Linecast per controllare 
     * se la retta che collega i due punti interseca il collider dell'oggetto.
     * Se la retta interseca il collider, la funzione restituisce true, 
     * altrimenti continua a cercare tra gli altri oggetti con il tag "Obstacle". 
     * Se la funzione non trova alcun oggetto con il tag "Obstacle" che 
     * interseca la retta, restituisce false.
    */



    // Aggunge un nuovo nodo alla lista nodes
    int AddNode(Vector3 position)
    {
        nodes.Add(position);
        return nodes.Count - 1;
    }


    // Aggiunge il nodo iniziale e finale alla lista Edge
    void AddEdge(int start, int end)
    {
        edges.Add(start);
        edges.Add(end);
    }


    // Funzione che disegna il percorso compiuto dai nodi generati con RRT
    void DrawRRT()
    {
        for (int i = 0; i < edges.Count; i += 2)
        {
            Debug.DrawLine(nodes[edges[i]], nodes[edges[i + 1]], Color.green);
        }
    }

    // ALTRE FUNZIONI NON UTILIZZATE
    /*Il metod CalculateCurvature calcola la curvatura tra tre punti*/
    /*float CalculateCurvature(Vector3 a, Vector3 b, Vector3 c)
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
    }/*

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