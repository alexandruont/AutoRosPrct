using DataTypes;
using System.Net.Http.Headers;
using System.Net.Sockets;
using System.Numerics;
using System.Runtime.InteropServices;

namespace MainProgram.src
{
    class RobotController
    {
        private TcpClient _tcpClient;
        private NetworkStream _stream;
        private Thread _thread;
        private bool _running = true;

        private int _cameraCount = 0;
        private int _armJointsCount = 0;
        private List<CameraImage> _cameraImages = new List<CameraImage>();

        public RobotController(TcpClient mClient) {
            _tcpClient = mClient;
            _thread = new Thread(new ThreadStart(HandleRequests));
            _thread.Start();
            _stream = _tcpClient.GetStream();
        }

        private void HandleRequests(){
            byte[] headerStream = new byte[Marshal.SizeOf(typeof(Header))];
            while (_running){
                _stream.Read(headerStream, 0, Marshal.SizeOf(typeof(Header)));
                Header reqStart = ByteArrayToStruct<Header>(headerStream);
                switch(reqStart.reqType){
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
            _tcpClient.Close();
        }
        // This would be helpfull if we have to update something from the robot
        // Now it doesnt have any use
        private void handleGetRequest(Header request){

        }
        // This function is mainly made for handling post of images and lidar data
        private void handlePostRequest(Header request){
            switch (request.infoType) {
                case InfoType.Camera:
                    byte[] buffer = new byte[Marshal.SizeOf(typeof(int))];
                    _stream.Read(buffer);
                    int index = BitConverter.ToInt32(buffer, 0);
                    _stream.Read(_cameraImages[index].imageData, 0, _cameraImages[index].width * _cameraImages[index].height);
                    break;
                case InfoType.Arm:
                    break;
                case InfoType.None:
                    Console.WriteLine("You can't POST nothing to the server");
                    break;
                default:
                    Console.WriteLine("Unkown Info type for POST request");
                    break;
            }
        }
        // This function is used for different variables that needs to be updated
        // This would likely not be used cause we would not have something like this in plan
        private void handleSetRequest(Header request){

        }
        // This function is the most important cause this will let us prepare buffer for handling
        // requests informations in a fast and efficient way
        private void handleSpecsRequest(Header request){
            byte[] buffer = new byte[sizeof(int) * 2];
            _stream.Read(buffer, 0, sizeof(int));
            _cameraCount = BitConverter.ToInt32(buffer, 0);
            for (int i = 0; i < _cameraCount; i++)
            {
                _stream.Read(buffer, 0, sizeof(int) * 2);
                CameraImage cameraImage;
                cameraImage.width = BitConverter.ToInt32(buffer, 0);
                cameraImage.height = BitConverter.ToInt32(buffer, 1);
                cameraImage.imageData = new byte[cameraImage.width * cameraImage.height];
                _cameraImages.Add(cameraImage);
            }
            _stream.Read(buffer, 0, sizeof(int));
            _armJointsCount = BitConverter.ToInt32(buffer, 0);
        }

        public static T ByteArrayToStruct<T>(byte[] bytes) where T : struct
        {
            GCHandle handle = GCHandle.Alloc(bytes, GCHandleType.Pinned);
            try
            {
                return Marshal.PtrToStructure<T>(handle.AddrOfPinnedObject());
            }
            finally
            {
                handle.Free();
            }
        }
    }
}
