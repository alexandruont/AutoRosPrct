const express = require('express');
const { createServer } = require('node:http');
const { join } = require('node:path');
const { Server } = require('socket.io');

const app = express();
const path = require('path');
const server = createServer(app);
const io = new Server(server);

var publicDir = require('path').join(__dirname, '/public');
app.use(express.static(publicDir));

app.get('/', (req, res) => {
  res.sendFile(join(__dirname, 'index.html'));
});

app.use(express.static(path.join(__dirname, 'public')));

io.on('connection', (socket) => {
  const clientIp = socket.handshake.address;
  console.log(`User connected from IP: ${clientIp}`);

  socket.on('disconnect', () => {
    console.log('User disconnected');
  });

  socket.on('robot control', (direction) => {
    console.log(`User pressed ${direction}`);
  });
});

server.listen(8080, '0.0.0.0', () => {
  console.log('Server running at http://0.0.0.0:8080/');
});