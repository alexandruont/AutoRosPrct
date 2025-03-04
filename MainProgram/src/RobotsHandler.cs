using System;
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