using TcpServerExample;

namespace MainProgram.src
{
    class Program
    {
        public static void main(string[] args)
        {
            Console.WriteLine("Hello World");
            RobotsHandler Server = new RobotsHandler();
            Server.Init();
        }
    }
}