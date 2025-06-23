#AutoRosPrct
A full-stack web interface enabling real-time control and monitoring of a physical robot. Built with modern technologies to empower responsive, interactive robotics applications.

##Features
-Live robot camera feed: Seamlessly cycles images in the browser UI to simulate real‑time vision.
-Directional controls: Intuitive button grid for movement commands—forward, backward, left, right, rotation.
-6-axis robotic arm control: Range sliders for precise joint angle adjustment; real‑time control over WebSockets.
-Interactive UI elements: Notification banner, hamburger menus for navigation and robot selection, with polished animations.

##Tech Stack 
-Frontend: HTML, CSS, JavaScript 
-Backend: Node.js + Express
-Real-time: WebSockets via Socket.io, TCP socket to communicate with the robot hardware
-Utilities: CORS middleware, static file serving, responsive design

##Architecture
1.Client-side: Loads HTML/CSS/JS in the browser to render the UI and handle events.
2.Socket.io client: Captures control events (button presses, slider adjustments) and emits them to the server.
3.Node.js server: Hosts the web app, listens for WebSocket messages, translates them into TCP commands.
4.TCP client: Sends robot-specific command JSON via TCP to the robot controller, enabling motion and arm positioning.
5.Camera feed: Simulated by cycling through a set of image files, giving a live-view experience.
