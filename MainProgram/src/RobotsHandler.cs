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
        private static Dictionary<string, RobotController> _robotControllers = new Dictionary<string, RobotController>();

        public RobotsHandler(int port)
        {
            _listener = new TcpListener(IPAddress.Any, port);
        }

        public async Task StartAsync() // Start server si listen pentru conexiuni
        {
            _listener.Start();
            Console.WriteLine("Server started. Waiting for connections...");
            while (true)
            {
                var client = await _listener.AcceptTcpClientAsync(); // Asteapta conexiuni
                Console.WriteLine("Client connected.");
                string ipAddress = ((IPEndPoint)client.Client.RemoteEndPoint).Address.ToString();
                _robotControllers[ipAddress] = new RobotController(client);
            }
        }

        private void Stop()
        {
            _listener.Stop();
            _robotControllers.Clear();
        }
    }
}