# Robotics_Project
# Il file da leggere è chiamato RRTPathPlanner, all'interno della cartella CODE. Tutte le funzioni sono commentate e poi anche spiegate sotto in maniera dettagliata. Piccolo sunto delle funzioni:
- void BuildRRT():
cuore dell'algoritmo RRT, che genera un albero di campionamento casuale e cerca di connettere il nodo più vicino a un nuovo campione valido. Se il campione si connette con il nodo finale, viene creato un nuovo nodo finale e l'algoritmo termina.

- Vector3 GetRandomSample():
Genera un campione casuale all'interno del diametro del robot, con una dimensione del passo specificata.

- Int GetNearestNode(Vector3 sample):
trova il nodo più vicino al campione casuale

- bool CheckEdge(Vector3 start, Vector3 end):
Verifica se il percorso tra due punti è valido, verificando la collisione con gli oggetti nell'ambiente e la curvatura massima consentita.

- bool CheckCurvature(Vector3 point):
Calcola la curvatura tra tre punti e verifica se rientra nella curvatura massima consentita.

- float CalculateCurvature2(Vector3 a, Vector3 b, Vector3 c):
Calcola la curvatura tra i tre punti nello spazio 3D

- bool CheckForObstacle(Vector3 start, Vector3 end):
Prende come argomenti due punti nello spazio 3D, start e end, e restituisce un valore booleano che indica se la retta che collega i due punti interseca un oggetto nella scena Unity con il tag "Obstacle", nel nostro caso l'atrio.

- int AddNode(Vector3 position):
Aggunge un nuovo nodo alla lista nodes

- void AddEdge(int start, int end)
Aggiunge il nodo iniziale e finale alla lista Edge

- void DrawRRT():
Funzione che disegna il percorso compiuto dai nodi generati con RRT
