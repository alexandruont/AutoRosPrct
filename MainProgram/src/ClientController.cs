using System.Net.Sockets;
using System.Text;
using System.Text.Json;
using TCPIPServer;

namespace MainProgram.src
{
    public class ClientController
    {
        public string _ip;
        private RobotController? _robot;
        public static char _delimiter = '\n';
        private TcpClient _tcpClient;
        private NetworkStream _stream;
        private Thread _thread;
        private RobotsHandler _owner;

        private string _unfinishedCommand;
        private List<string> _commandLog = new List<string>(); // List to record commands

        public ClientController(TcpClient tcpClient, string ip, RobotsHandler owner)
        {
            _ip = ip;
            _tcpClient = tcpClient;
            _owner = owner;
            _stream = _tcpClient.GetStream();
            _thread = new Thread(new ThreadStart(HandleRequests));
            _thread.Start();
        }

        private void HandleRequests()
        {
            byte[] buffer = new byte[1024];
            StringBuilder sb = new StringBuilder();
            try
            {
                while (RobotsHandler._running)
                {
                    int bytesRead = _stream.Read(buffer, 0, buffer.Length);
                    if (bytesRead == 0) break; // Client disconnected
                    sb.Append(Encoding.UTF8.GetString(buffer, 0, bytesRead));

                    while (sb.ToString().Contains(_delimiter))
                    {
                        int index = sb.ToString().IndexOf(_delimiter);
                        if (index == -1) break;
                        string message = sb.ToString(0, index);
                        sb.Remove(0, index + 1);
                        HandleMessage(message); // handle the message
                    }
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }
            finally
            {
                Console.WriteLine("Web Server Disconnected");
                HandleDisconnection();
            }
            _tcpClient.Close();
        }

        private void HandleDisconnection()
        {
            _owner._clientController.Remove(_ip);
            Console.WriteLine($"Client Controller with IP {_ip} has been removed from the list.");
        }

        private void HandleMessage(string message)
        {
            try
            {
                using JsonDocument doc = JsonDocument.Parse(message);
                JsonElement root = doc.RootElement;
                if(root.TryGetProperty("type", out JsonElement ids)){
                    JsonElement prop;
                    if (!root.TryGetProperty(ids.ToString(), out prop)){
                        Console.WriteLine($"Client Controller: Tried to get the propriety: {ids.ToString()}, but failed");
                        return;
                    }
                    switch (ids.ToString()){
                        case "move":
                            int i = 0;
                            int[] res = new int[3];
                            foreach(JsonElement nr in prop.EnumerateArray()){
                                res[i] = nr.GetInt32();
                                i++;
                            }
                            Console.WriteLine($"Client Controller: Got move: {res[0]}, {res[1]}, {res[2]}");
                            if (_robot != null){
                                _robot.sendMoveRequest(res[0], res[1], res[2]);
                            }
                            break;
                        case "arm":
                            break;
                        case "robot":
                            Console.WriteLine(prop.Deserialize<string>());
                            break;
                        case "R_A":
                            sendConnectedRobots();
                            break;
                        default:
                            Console.WriteLine($"Client Controller: Unknown request type: {ids.ToString()}");
                            break;
                    }
                }
            }
            catch (JsonException e)
            {
                Console.WriteLine("Failed to parse message: " + e.Message);
            }
        }
        public void sendConnectedRobots(){
            string resp = $"{{'N_R': {_owner._robotControllers.Count}, 'IP':{{";
            foreach (var ip in _owner._robotControllers.Keys)
            {
                resp += $"'{ip}',";
            }
            resp += "''}}";
            byte[] bytes = Encoding.UTF8.GetBytes(resp);
            _stream.Write(bytes, 0, bytes.Length);
        }
    }
}
