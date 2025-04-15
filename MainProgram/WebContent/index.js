const express = require('express');
const { createServer } = require('node:http');
const { join, parse } = require('node:path');
const { Server } = require('socket.io');
const cors = require('cors'); // Import the cors package
const net = require('net'); // Import the net package

const app = express();
const path = require('path');
const server = createServer(app);
const io = new Server(server, {
  cors: {
    origin: '*', // Allow all origins
    methods: ['GET', 'POST']
  }
});

var availableRobots = [];

app.use(cors()); // Use the cors middleware

var publicDir = require('path').join(__dirname, '/public');
app.use(express.static(publicDir));

app.get('/', (req, res) => {
  res.sendFile(join(__dirname, 'index.html'));
});

app.use(express.static(path.join(__dirname, 'public')));

// TCP client to connect to the TCP server
const tcpClient = new net.Socket();
tcpClient.connect(8000, '192.168.56.1', () => {
    console.log('Connected to TCP server');
    // Create a buffer to hold the 4-byte integer
    const buffer = Buffer.alloc(4);
    const intValue = 1; // The integer value to send
    buffer.writeInt32BE(intValue, 0); // Write the integer to the buffer in big-endian format
    // Send the buffer to the server
    tcpClient.write(buffer);
    tcpClient.write('{"type":"R_A", "R_A":0}\n');
});

tcpClient.on('data', (data) => {
    listR = JSON.parse(data);
    availableRobots = listR.IP;
    availableRobots = availableRobots.filter(function (el) {
        return el != '';
    })
    console.log('Received from TCP server:', data.toString());
});

tcpClient.on('close', () => {
  console.log('Connection to TCP server closed');
});

tcpClient.on('error', (err) => {
  console.error('TCP client error:', err);
});

io.on('connection', (socket) => {
  const clientIp = socket.handshake.address;
  console.log(`User connected from IP: ${clientIp}`);

  socket.on('disconnect', () => {
    console.log('User disconnected');
  });

  socket.on('robot control', (direction) => {
      console.log(`User pressed ${direction}`);
      tcpClient.write(`{"type":"move","move":[${direction}]}\n`);
  });

  socket.on('arm control', (data) => {
      console.log(`Arm control command received: Joint ${data.joint}, Angle ${data.angle}`);
      tcpClient.write(`{"type":"arm","arm":[${data.joint},${data.angle}]}\n`);
  });

    socket.on('IPs', (data) => {
        socket.write('IPs', availableRobots);
    });
});

server.listen(8080, '0.0.0.0', () => {
  console.log('Server running at http://0.0.0.0:8080/');
});
