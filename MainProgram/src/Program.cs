using TCPIPServer;

namespace MainProgram.src
{
    class Program
    {
        public static void Main(string[] args)
        {
            Console.WriteLine("Starting server...");
            RobotsHandler Server = new RobotsHandler(port: 8000);
            var tsk = Server.StartAsync();
            while (true) {
                string command = Console.ReadLine();
                if (command == "quit") {
                    break;
                }
            }
            RobotsHandler._running = false;
            tsk.Wait();
            Console.WriteLine("Server Stopped");
        }
    }
}