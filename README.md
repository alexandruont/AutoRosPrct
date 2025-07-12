# AutoRosPrct

A full-stack web interface enabling real-time control and monitoring of a physical robot's. Built with modern technologies to empower responsive, interactive robotics applications.

---

## Features

- **Live robot camera feed**: Seamlessly cycles images in the browser UI to simulate real-time vision.  
- **Directional controls**: Intuitive button grid for movement commands — forward, backward, left, right, rotation.  
- **6-axis robotic arm control**: Range sliders for precise joint angle adjustment; real-time control over WebSockets.  
- **Interactive UI elements**: Notification banner, hamburger menus for navigation and robot selection, with polished animations.
- **Robot Selection based on IP address**: Letting you select a specifyc robot to controll

---

## Tech Stack

- **Frontend**: HTML, CSS, JavaScript  
- **Backend**: Node.js + Express, C#, C++
- **Real-time Communication**: WebSockets via Socket.io, TCP socket server for syncronizing robot's and creating a communication bridge between robot and client
- **Utilities**: CORS middleware, static file serving, responsive design, asyncronous task execution

---

## Architecture

1. **Client-side**: Loads HTML/CSS/JS in the browser to render the UI and handle events.  
2. **Socket.io client**: Captures control events (button presses, slider adjustments) and emits them to the server.  
3. **Node.js server**: Hosts the web app, listens for WebSocket messages, translates them into TCP commands.
4. **TCP server**: Process the request from the Node.js server and organize robot's work, by communicating with them using a custom binary protocol
5. **TCP client**: Runs as a **ROS node** on the robot, and communicates with the **TCP server** using the binary protocol and a C++ library for **TCPClient**
6. **Camera feed**: Simulated by cycling through a set of image files, giving a live-view experience.

---

## Planned Features

1. **Improved Custom Binary Protocol**: Refactor the existing protocol to include a more structured and informative header. Enable dynamic specification of data type sizes by the server, allowing greater flexibility and extensibility.
2. **Modern Web Interface**: Develop a new web-based dashboard using a modern frontend framework. The interface will feature real-time 3D rendering, user authentication, and intuitive controls for monitoring and interaction.
3. **Enhanced TCP Server-Client Communication**: Redesign the TCP communication layer to support multi-client handling. This will allow the server to maintain stable connections with multiple robot instances or applications simultaneously.
4. **Universal ROS-Compatible TCP Client**: Build a lightweight and portable TCP client that can be easily integrated into any ROS-enabled robot with minimal configuration. This will enable seamless communication with the central server, regardless of the robot's hardware.
