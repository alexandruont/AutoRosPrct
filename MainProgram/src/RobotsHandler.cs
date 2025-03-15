using System;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using System.Collections.Generic;
using MainProgram.src;

namespace TCPIPServer
{
    public class RobotsHandler
    {
        public static bool _running = true;
        private TcpListener _listener;
        public Dictionary<string, RobotController> _robotControllers = new Dictionary<string, RobotController>();
        public Dictionary<string, ClientController> _clientController = new Dictionary<string, ClientController>();

        public RobotsHandler(int port)
        {
            _listener = new TcpListener(IPAddress.Any, port);
        }

        public async Task StartAsync() // Start server and listen for connexions
        {
            _listener.Start();
            Console.WriteLine("Server started. Waiting for connections...");
            while (_running)
            {
                var client = await _listener.AcceptTcpClientAsync(); // Accept Connections
                int clientTypeIndex;
                NetworkStream stream = client.GetStream();

                byte[] buffer = new byte[4];
                int bytesRead = stream.Read(buffer, 0, buffer.Length);

                // Convert the 4-byte big-endian buffer to an integer
                clientTypeIndex = BitConverter.ToInt32(buffer.Reverse().ToArray(), 0);
                Console.WriteLine("Received integer: " + clientTypeIndex);

                string ipAddress = ((IPEndPoint)client.Client.RemoteEndPoint).Address.ToString();
                switch (clientTypeIndex)
                {
                    case 0:
                        Console.WriteLine($"Robot connected with IP: {ipAddress}");
                        _robotControllers.Add(ipAddress, new RobotController(client, ipAddress, this));
                        foreach(var cl in _clientController.Values){
                            cl.sendConnectedRobots();
                        }
                        break;
                    case 1:
                        Console.WriteLine($"Client Connected with IP: {ipAddress}");
                        _clientController.Add(ipAddress, new ClientController(client, ipAddress, this));
                        break;
                    default:
                        Console.WriteLine($"Unkown user type: {clientTypeIndex}");
                        break;
                }
            }
        }

        private void Stop()
        {
            _listener.Stop();
            _robotControllers.Clear();
        }
    }
}