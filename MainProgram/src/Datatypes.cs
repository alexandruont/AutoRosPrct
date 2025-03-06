
using System.Numerics;

namespace DataTypes
{

    enum ReqType
    {
        Get, // Get any kind of information
        Post, // Post some information. Used for file like data. Mainly used by the robot

        Set, // Set is for updating the arm position and robot speed
        Specs // Sends a request for robot specifications(like number of cameras)
    }

    enum InfoType
    {
        None = 0,
        Camera = 1,
        Arm = 2,
        Speed = 3,
        ImageSize
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
}