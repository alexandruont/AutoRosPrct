using TCPIPServer;

namespace MainProgram.src
{
    class Program
    {
        public static void Main(string[] args)
        {
            Console.WriteLine("Starting server...");
            RobotsHandler Server = new RobotsHandler(port: 8000);
            Server.StartAsync().Wait();
        }
    }
}