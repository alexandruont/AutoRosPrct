using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace TcpServerExample
{
    class RobotsHandler
    {
        // Folosim "ConcurrentDictionary" pentru a gestiona conexiunea, si folosim valoarea "TcpClient"
        static ConcurrentDictionary<string, TcpClient> connectedRobots = new ConcurrentDictionary<string, TcpClient>();

        public void Init()
        {
            int port = 8000; // Portul pe care serverul va asculta conexiunile
            TcpListener server = new TcpListener(IPAddress.Any, port);
            server.Start();
            Console.WriteLine($"Serverul a pornit pe portul {port}.");

            // Bucla principală: acceptăm clienți pe măsură ce se conectează
            while (true)
            {
                TcpClient client = server.AcceptTcpClient();
                // Fiecare client este gestionat într-un thread separat
                Thread clientThread = new Thread(() => HandleClient(client));
                clientThread.Start();
            }
        }

        // Funcția de gestionare a fiecărui client
        static void HandleClient(TcpClient client)
        {
            string clientEndPoint = client.Client.RemoteEndPoint.ToString();
            Console.WriteLine($"Client conectat: {clientEndPoint}");

            try
            {
                NetworkStream stream = client.GetStream();
                byte[] buffer = new byte[1024];
                int bytesRead;
                // Citim datele primite de la client
                while ((bytesRead = stream.Read(buffer, 0, buffer.Length)) != 0)
                {
                    // Convertim datele din bytes în string
                    string message = Encoding.ASCII.GetString(buffer, 0, bytesRead).Trim();
                    Console.WriteLine($"Mesaj primit de la {clientEndPoint}: {message}");

                    // Exemplu de protocol: se așteaptă comenzi de forma:
                    // "CONNECT_ROBOT numeSauIp" sau "DISCONNECT_ROBOT numeSauIp"
                    string[] parts = message.Split(' ');
                    if (parts.Length == 2)
                    {
                        string command = parts[0].ToUpper();
                        string robotIdentifier = parts[1];

                        if (command == "CONNECT_ROBOT")
                        {
                            ConnectRobot(robotIdentifier, client);
                        }
                        else if (command == "DISCONNECT_ROBOT")
                        {
                            DisconnectRobot(robotIdentifier);
                        }
                        else
                        {
                            Console.WriteLine("Comandă necunoscută.");
                        }
                    }
                    else
                    {
                        Console.WriteLine("Formatul comenzii nu este corect.");
                    }

                    // Exemplu de răspuns către client
                    byte[] response = Encoding.ASCII.GetBytes("Comanda a fost procesată.\n");
                    stream.Write(response, 0, response.Length);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Eroare la clientul {clientEndPoint}: {ex.Message}");
            }
            finally
            {
                // Închidem conexiunea clientului
                client.Close();
                Console.WriteLine($"Conexiunea cu {clientEndPoint} a fost închisă.");
            }
        }

        // Funcția de conectare a unui robot
        static void ConnectRobot(string identifier, TcpClient client)
        {
            if (connectedRobots.TryAdd(identifier, client))
            {
                Console.WriteLine($"Robot conectat: {identifier}");
            }
            else
            {
                Console.WriteLine($"Robotul {identifier} este deja conectat.");
            }
        }

        // Funcția de deconectare a unui robot
        static void DisconnectRobot(string identifier)
        {
            if (connectedRobots.TryRemove(identifier, out TcpClient removedClient))
            {
                Console.WriteLine($"Robot deconectat: {identifier}");
                // Închidem conexiunea asociată robotului
                removedClient.Close();
            }
            else
            {
                Console.WriteLine($"Nu s-a găsit robotul {identifier} pentru deconectare.");
            }
        }
    }
}