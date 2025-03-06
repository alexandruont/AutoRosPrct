using System;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

namespace TCPIPServer
{
    public class RobotsHandler
    {
        private TcpListener _listener;

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
                _ = Task.Run(() => ProcessClientAsync(client)); // Proceseaza clientul in mod asincron
            }
        }

        private async Task ProcessClientAsync(TcpClient client)
        {
            try
            {
                using (client)
                {
                    var stream = client.GetStream();

                    while (true)
                    {
                        // Citeste header-ul
                        var headerBuffer = new byte[12]; // Presupun : Type (4 bytes), Info (4 bytes), Size (4 bytes)
                        int headerBytesRead = await stream.ReadAsync(headerBuffer, 0, headerBuffer.Length);
                        if (headerBytesRead == 0) break; // Client disconnected

                        int type = BitConverter.ToInt32(headerBuffer, 0);
                        int info = BitConverter.ToInt32(headerBuffer, 4);
                        int size = BitConverter.ToInt32(headerBuffer, 8);

                        Console.WriteLine($"Received header - Type: {type}, Info: {info}, Size: {size}");

                        // Proceseaaza request-ul in functie de tip si info
                        switch (type)
                        {
                            case 1: // Get image info
                                if (info == 0 && size == 0)
                                {
                                    // Request pentru info imagine
                                    var imageInfo = "Image info: [data]";
                                    var response = Encoding.UTF8.GetBytes(imageInfo);
                                    await stream.WriteAsync(response, 0, response.Length);
                                    Console.WriteLine("Sent image info."); // Trimite raspuns
                                }
                                else
                                {
                                    Console.WriteLine("Invalid request for image info.");
                                }
                                break;

                            case 2: // Post image
                                if (info == 1 && size > 0)
                                {
                                    // Request pentru upload imagine
                                    var dataBuffer = new byte[1024];
                                    int totalBytesRead = 0;
                                    using (var fileStream = new FileStream("uploaded_image.jpg", FileMode.Create, FileAccess.Write))
                                    {
                                        while (totalBytesRead < size)
                                        {
                                            int bytesToRead = Math.Min(dataBuffer.Length, size - totalBytesRead);
                                            int dataBytesRead = await stream.ReadAsync(dataBuffer, 0, bytesToRead);
                                            if (dataBytesRead == 0) break; // Client disconnected

                                            totalBytesRead += dataBytesRead;
                                            await fileStream.WriteAsync(dataBuffer, 0, dataBytesRead);
                                        }
                                    }

                                    Console.WriteLine("Image uploaded successfully.");
                                    var response = Encoding.UTF8.GetBytes("Image uploaded successfully");
                                    await stream.WriteAsync(response, 0, response.Length); // Trimite raspuns
                                }
                                else
                                {
                                    Console.WriteLine("Invalid request for posting image.");
                                }
                                break;

                            default:
                                Console.WriteLine("Unknown request type.");
                                break;
                        }

                        // Daca exista un body citeste data in chunks
                        if (size > 0 && type != 2)
                        {
                            var dataBuffer = new byte[1024];
                            int totalBytesRead = 0;
                            var messageBuilder = new StringBuilder();

                            while (totalBytesRead < size)
                            {
                                int bytesToRead = Math.Min(dataBuffer.Length, size - totalBytesRead);
                                int dataBytesRead = await stream.ReadAsync(dataBuffer, 0, bytesToRead);
                                if (dataBytesRead == 0) break; // Client disconnected

                                totalBytesRead += dataBytesRead;
                                messageBuilder.Append(Encoding.UTF8.GetString(dataBuffer, 0, dataBytesRead));
                            }

                            var message = messageBuilder.ToString();
                            Console.WriteLine($"Received data: {message}");

                            // Trimite raspuns
                            var response = Encoding.UTF8.GetBytes("Message received");
                            await stream.WriteAsync(response, 0, response.Length);
                        }
                    }
                }
            }
            catch (SocketException ex)
            {
                Console.WriteLine($"Socket error: {ex.Message}");
            }
            catch (IOException ex)
            {
                Console.WriteLine($"IO error: {ex.Message}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Unexpected error: {ex.Message}");
            }
            finally
            {
                Console.WriteLine("Client disconnected.");
            }
        }
    }
}
// creem un program care trimite un request la robot de tip "get" si "info type" camera 1

namespace TCPIPClient
{
    class Program
    {
        static async Task Main(string[] args)
        {
            // adresa si port de conectare (pentru local host si port 8000) adresa test = ( 192.168.80.38:8080 ) adresa mea: 192.168.80.52
            string serverAddress = "192.168.80.52";
            int port = 8000; // port tcp standard = 8000
            string robotId = "Robot1"; // recunoaste robotul

            try
            {
                using (TcpClient client = new TcpClient()) 
                {
                    Console.WriteLine("Conectare la robot..."); // informeaza despre starea conexiunii la server
                    await client.ConnectAsync(serverAddress, port); // folosim functia ConnectAsync pentru a ne conecta la server
                    Console.WriteLine("Conectat!"); 

                    using (NetworkStream stream = client.GetStream()) // prin NetworkStream permite citeste si scrie datele din retea
                    {
                        string payloadText = "camera 1";
                        byte[] payloadBytes = Encoding.UTF8.GetBytes(payloadText); // convertim in UTF8 pentru a putea fi citit de retea

                        int type = 1; // unde tipul cererii e get
                        int info = 0; // pentru informatii suplimentare
                        int size = payloadBytes.Length; // lungimea payload-ului

                        byte[] header = new byte[12];
                        Array.Copy(BitConverter.GetBytes(type), 0, header, 0, 4);
                        Array.Copy(BitConverter.GetBytes(info), 0, header, 4, 4);
                        Array.Copy(BitConverter.GetBytes(size), 0, header, 8, 4);

                        await stream.WriteAsync(header, 0, header.Length); // trimitem head-erul pe fluxul retelei
                        if (size > 0)
                        {
                            await stream.WriteAsync(payloadBytes, 0, payloadBytes.Length);
                        }
                        Console.WriteLine("Request trimis: get info type camera 1"); //confirmam trimiterea cererii

                        using (MemoryStream ms = new MemoryStream()) // creem Memorytream pentru a stoca datele (comenzile) de la server
                        {
                            byte[] buffer = new byte[1024];
                            int bytesRead = 0;
                            while ((bytesRead = await stream.ReadAsync(buffer, 0, buffer.Length)) > 0) // cand ReadAsync = 0, conexiunea se opreste
                            {
                                ms.Write(buffer, 0, bytesRead);
                            }

                            byte[] responseBytes = ms.ToArray();

                            if (responseBytes.Length > 0) // daca responseBytes > 0, inseamna ca s-au primit datele
                            {
                                string fileName = "received_image.jpg"; // se asteapta o imagine de tip jpg
                                File.WriteAllBytes(fileName, responseBytes);
                                Console.WriteLine("Imaginea a fost primita si salvata ca " + fileName);

                                Process.Start(new ProcessStartInfo(fileName) { UseShellExecute = true }); // deschidem imaginea
                            }
                            else
                            {
                                string responseText = Encoding.UTF8.GetString(responseBytes); // afisam in consola in cazul in care nu se primeste nimic
                                Console.WriteLine("Raspuns primit: " + responseText);
                            }
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine("Eroare: " + ex.Message);
            }
        }
    }
}