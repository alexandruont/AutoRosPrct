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
            while (true)
            {
                var client = await _listener.AcceptTcpClientAsync(); // Accept Connections
                int clientTypeIndex;
                byte[] bytes = new byte[sizeof(int)];
                client.GetStream().Read(bytes);
                clientTypeIndex = BitConverter.ToInt32(bytes, 0);

                string ipAddress = ((IPEndPoint)client.Client.RemoteEndPoint).Address.ToString();
                switch (clientTypeIndex)
                {
                    case 0:
                        Console.WriteLine($"Robot connected with IP: {ipAddress}");
                        _robotControllers.Add(ipAddress, new RobotController(client, ipAddress, this));
                        break;
                    case 1:
                        Console.WriteLine($"Client Connected with IP: {ipAddress}");
                        _clientController.Add(ipAddress, new ClientController(client, ipAddress));
                        break;
                    default:
                        Console.WriteLine("Unkown user type");
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