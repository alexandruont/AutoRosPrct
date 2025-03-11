
using System.Numerics;

namespace DataTypes
{

    enum ReqType
    {
        Get = 0, // Get any kind of information
        Post = 1, // Post some information. Used for file like data. Mainly used by the robot
        Set = 2, // Set is for updating the arm position and robot speed
        Specs = 3 // Sends a request for robot specifications(like number of cameras)
    }

    enum InfoType
    {
        None = 0,
        Camera = 1,
        Arm = 2,
        Movement = 3,
        ImageSize = 4
    }

    struct Header
    {
        public ReqType reqType;
        public InfoType infoType;
        public Int32 size;

        public void print(){
            Console.WriteLine($"Request Info: \n\tRequest Type: {reqType}\n\tInfo Type: {infoType}\n\tSize: {size}");
        }
    }

    struct CameraImage
    {
        public byte[] imageData;
        public int width;
        public int height;

        public CameraImage(byte[] imageData, int width, int height)
        {
            this.imageData = imageData;
            this.width = width;
            this.height = height;
        }

        public void print()
        {
            Console.WriteLine($"Image Info: \n\tWidth: {width}\n\tHeight: {height}\n\tData Length: {imageData.Length}");
        }
    }
}