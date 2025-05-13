using DataTypes;
using System.IO.Pipes;
using System.Net.Http.Headers;
using System.Net.Sockets;
using System.Numerics;
using System.Reflection.Metadata;
using System.Runtime.InteropServices;
using TCPIPServer;

namespace MainProgram.src
{
    public class RobotController
    {
        public string _ip;
        private TcpClient _tcpClient;
        private NetworkStream _stream;
        private Thread _thread;
        private RobotsHandler _owner;
        private int _cameraCount = 0;
        private int _armJointsCount = 6;
        private List<CameraImage> _cameraImages = new List<CameraImage>();

        public RobotController(TcpClient mClient, string ip, RobotsHandler owner) {
            _ip = ip;
            _tcpClient = mClient;
            _owner = owner;
            _stream = _tcpClient.GetStream();
            _thread = new Thread(new ThreadStart(HandleRequests));
            _thread.Start();
        }

        private void HandleRequests()
        {
            byte[] headerStream = new byte[Marshal.SizeOf(typeof(Header))];
            try
            {
                while (RobotsHandler._running)
                {
                    int amount = _stream.Read(headerStream, 0, Marshal.SizeOf(typeof(Header)));
                    if (amount == 0){
                        break;
                    }
                    Header reqStart = ByteArrayToStruct(headerStream);
                    reqStart.print();
                    switch (reqStart.reqType)
                    {
                        case ReqType.Get:
                            handleGetRequest(reqStart);
                            break;
                        case ReqType.Set:
                            handleSetRequest(reqStart);
                            break;
                        case ReqType.Post:
                            handlePostRequest(reqStart);
                            break;
                        case ReqType.Specs:
                            handleSpecsRequest(reqStart);
                            break;
                        default:
                            Console.WriteLine("Unable to identify Request type");
                            reqStart.print();
                            break;
                    }
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }
            finally
            {
                HandleDisconnection();
            }
            _tcpClient.Close();
        }

        private void HandleDisconnection()
        {
            _owner._robotControllers.Remove(_ip);
            Console.WriteLine($"RobotController with IP {_ip} has been removed from the list.");
        }

        public void sendArmRequest(int joint, int value){
            Header header;
            header.reqType = ReqType.Set;
            header.infoType = InfoType.Arm;
            header.size = sizeof(int) * 2;
            _stream.Write(StructToByteArray(header), 0, Marshal.SizeOf(header));
            byte[] bytes = new byte[sizeof(int) * 2];
            Buffer.BlockCopy(BitConverter.GetBytes(joint), 0, bytes, 0, sizeof(int));
            Buffer.BlockCopy(BitConverter.GetBytes(value), 0, bytes, sizeof(int), sizeof(int));
            _stream.Write(bytes, 0, sizeof(int) * 2);
        }

        public void sendMoveRequest(double forward, double right, double turn)
        {
            Header header;
            header.reqType = ReqType.Set;
            header.infoType = InfoType.Movement;
            header.size = sizeof(double) * 3;
            _stream.Write(StructToByteArray(header), 0, Marshal.SizeOf(header));
            byte[] bytes = new byte[sizeof(double) * 3];
            Buffer.BlockCopy(BitConverter.GetBytes(forward), 0, bytes, 0, sizeof(double));
            Buffer.BlockCopy(BitConverter.GetBytes(right), 0, bytes, sizeof(double), sizeof(double));
            Buffer.BlockCopy(BitConverter.GetBytes(right), 0, bytes, sizeof(double) * 2, sizeof(double));
            _stream.Write(bytes, 0, sizeof(double) * 3);
        }

        private void handleGetRequest(Header request){
            // Implementation for GET request
        }

        private void handlePostRequest(Header request){
            switch (request.infoType) {
                case InfoType.Camera:
                    byte[] buffer = new byte[sizeof(int)];
                    _stream.Read(buffer, 0, sizeof(int));
                    int index = BitConverter.ToInt32(buffer, 0);
                    Console.WriteLine(index.ToString());

                    int totalBytesToRead = _cameraImages[index].width * _cameraImages[index].height * 3;
                    int bytesRead = 0;
                    while (bytesRead < totalBytesToRead){
                        bytesRead += _stream.Read(_cameraImages[index].imageData, bytesRead, totalBytesToRead - bytesRead);
                        Console.WriteLine($"Read-ed : {bytesRead.ToString()} out of {totalBytesToRead}");
                    }
                    break;
                case InfoType.Arm:
                    // Handle arm control
                    byte[] jointBuffer = new byte[Marshal.SizeOf(typeof(int)) * _armJointsCount];
                    _stream.Read(jointBuffer, 0, jointBuffer.Length);
                    int[] jointAngles = new int[_armJointsCount];
                    for (int i = 0; i < _armJointsCount; i++) {
                        jointAngles[i] = BitConverter.ToInt32(jointBuffer, i * Marshal.SizeOf(typeof(int)));
                    }
                    ControlArm(jointAngles);
                    break;
                case InfoType.None:
                    Console.WriteLine("You can't POST nothing to the server");
                    break;
                default:
                    Console.WriteLine("Unknown Info type for POST request");
                    break;
            }
        }

        private void handleSetRequest(Header request){
            // Implementation for SET request
        }

        private void handleSpecsRequest(Header request){
            byte[] buffer = new byte[sizeof(int) * 2];
            byte[] intBuffer = new byte[sizeof(int)];
            _stream.Read(intBuffer, 0, sizeof(int));
            _cameraCount = BitConverter.ToInt32(intBuffer, 0);
            Console.WriteLine($"IP: {_ip} sent the specs:\n\tNumber of cameras: {_cameraCount}");
            for (int i = 0; i < _cameraCount; i++)
            {
                _stream.Read(buffer, 0, sizeof(int) * 2);
                CameraImage cameraImage;
                cameraImage.width = BitConverter.ToInt32(buffer, 0);
                cameraImage.height = BitConverter.ToInt32(buffer, 4);
                cameraImage.imageData = new byte[cameraImage.width * cameraImage.height * 3];
                _cameraImages.Add(cameraImage);
                Console.WriteLine($"\tCamera{i} with sizes: {cameraImage.width}, {cameraImage.height}");
            }
            _stream.Read(intBuffer, 0, sizeof(int));
            _armJointsCount = BitConverter.ToInt32(intBuffer, 0);
            Console.WriteLine($"\tNumber of arm joints: {_armJointsCount}");
        }

        private void ControlArm(int[] jointAngles) {
            for (int i = 0; i < _armJointsCount; i++) {
                Console.WriteLine($"Joint {i + 1}: {jointAngles[i]} degrees");
            }
        }

        Header ByteArrayToStruct(byte[] bytes)
        {
            if (bytes.Length != Marshal.SizeOf(typeof(Header)))
            {
                throw new ArgumentException($"Byte array length does not match the size of the structure. Expected {Marshal.SizeOf(typeof(Header))} bytes but got {bytes.Length} bytes.");
            }

            Header header = new Header();
            header.reqType = (ReqType)BitConverter.ToInt32(bytes, 0);
            header.infoType = (InfoType)BitConverter.ToInt32(bytes, 4);
            header.size = BitConverter.ToInt32(bytes, 8);

            return header;
        }

        byte[] StructToByteArray(Header header)
        {
            byte[] bytes = new byte[Marshal.SizeOf(typeof(Header))];
            Buffer.BlockCopy(BitConverter.GetBytes((int)header.reqType), 0, bytes, 0, sizeof(int));
            Buffer.BlockCopy(BitConverter.GetBytes((int)header.infoType), 0, bytes, sizeof(int), sizeof(int));
            Buffer.BlockCopy(BitConverter.GetBytes(header.size), 0, bytes, 2 * sizeof(int), sizeof(int));
            return bytes;
        }
    }
}