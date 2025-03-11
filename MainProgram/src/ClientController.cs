using DataTypes;
using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Text.Json;
using System.Threading;

namespace MainProgram.src
{
    public class ClientController
    {
        public string _ip;
        public string _connectedRobot = "";
        public static char _delimiter = '\n';
        private TcpClient _tcpClient;
        private NetworkStream _stream;
        private Thread _thread;
        private bool _running = true;

        public ClientController(TcpClient tcpClient, string ip)
        {
            _ip = ip;
            _tcpClient = tcpClient;
            _thread = new Thread(new ThreadStart(HandleRequests));
            _thread.Start();
            _stream = _tcpClient.GetStream();
        }

        private void HandleRequests()
        {
            byte[] buffer = new byte[1024];
            StringBuilder sb = new StringBuilder();
            try
            {
                while (_running)
                {
                    int bytesRead = _stream.Read(buffer, 0, buffer.Length);
                    if (bytesRead == 0) break; // Client disconnected

                    sb.Append(Encoding.UTF8.GetString(buffer, 0, bytesRead));

                    while (sb.ToString().Contains(_delimiter))
                    {
                        int index = sb.ToString().IndexOf(_delimiter);
                        string message = sb.ToString(0, index);
                        sb.Remove(0, index + sizeof(char));
                        Console.WriteLine("Received: " + message);
                    }
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }
            finally
            {
                _tcpClient.Close();
            }
        }

        private void HandleMessage(string message)
        {
            try
            {
                var command = JsonSerializer.Deserialize<Header>(message);
                switch (command.reqType) {
                    case ReqType.Set:

                        break;
                    default:
                        Console.WriteLine("Unhandle client request");
                        break;
                }
            }
            catch (JsonException e)
            {
                Console.WriteLine("Failed to parse message: " + e.Message);
            }
        }
    }
}
