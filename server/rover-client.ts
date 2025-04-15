/**
 * This file demonstrates how a rover client would be implemented.
 * It would typically run on the rover hardware and use rclnodejs to communicate with ROS2.
 * 
 * This is a sample implementation for reference.
 */
import * as zlib from 'zlib';

import WebSocket from 'ws';
import * as rclnodejs from 'rclnodejs';
import * as fs from 'fs';
import path from 'path';





interface SensorData {
  temperature?: number;

  speed?: number;
  latitude?: number;
  longitude?: number;
  batteryLevel?: number;
  signalStrength?: number;

  cpuUsage?: number;
  memoryUsage?: number;
  cameraImage?:string;
  distanceTraveled: number;  // ✅ Add this property
  trips:number;
  currentPosition: { x: number; y: number ; z:number} | null,
  //mapdata?:string;

}

interface MapData {
  width: number;
  height: number;
  resolution: number;
  origin: {
    x: number;
    y: number;
  };
  data: number[];
}


class RoverClient {
  private ws: WebSocket | null = null;
  private roverId: number | null = null;
  private roverIdentifier: string;
  private serverUrl: string;
  private connected = false;


  private rclNode: rclnodejs.Node | null = null;
  private sensorPublishers: Map<string, rclnodejs.Publisher<any>> = new Map();
  private commandSubscription: rclnodejs.Subscription | null = null;
  private reconnectInterval: NodeJS.Timeout | null = null;
  private pingInterval: NodeJS.Timeout | null = null;
  private lastCommandId: number | null = null;
  private imageSubscription: rclnodejs.Subscription | null = null; // Subscription for camera image
  private distanceTraveled :number =0;
  private tripCount:number =0;
  private currentPosition: { x: number; y: number ;z:number } | null = null;
  private startPosition: { latitude: number; longitude: number } | null = null;
  private lastKnownPosition:{ x: number; y:number ; z :number} | null = null;
  private mapdata: MapData|null=null;

  private sensorData: SensorData = {
    temperature: 0,
    speed: 0,
    latitude: 0,
    longitude: 0,
    batteryLevel: 100,
    signalStrength: 0,
    cpuUsage: 0,
    memoryUsage: 0,
    distanceTraveled: 0,
    trips: 0,
    currentPosition: { x: 0, y: 0, z: 0 },
  };
  


  
  
  constructor(serverUrl: string, roverIdentifier: string) {
    this.serverUrl = serverUrl;
    this.roverIdentifier = roverIdentifier;
  }
  
  async initialize() {
    try {
      // Initialize ROS2 node
      await rclnodejs.init();
      this.rclNode = new rclnodejs.Node(`rover_${this.roverIdentifier}`);
      
      // Create publishers for sensor data
      this.setupRosPublishers();
      
      // Create subscription for commands
      this.setupRosSubscriptions();
      
      // Start ROS2 node
      this.rclNode.spin();
      
      console.log('ROS2 node initialized');
      
      // Connect to server
      this.connect();
      
    } catch (error) {
      console.error('Failed to initialize rover client:', error);
    }
  }
  
  private setupRosPublishers() {
    if (!this.rclNode) return;
    
    // Create publishers for various sensor data
    //this.sensorPublishers.set('temperature', this.rclNode.createPublisher('std_msgs/msg/Float32', 'temperature'));
    this.sensorPublishers.set('humidity', this.rclNode.createPublisher('std_msgs/msg/Float32', 'humidity'));
    this.sensorPublishers.set('pressure', this.rclNode.createPublisher('std_msgs/msg/Float32', 'pressure'));
    // Add more publishers for other sensors
  }
  
  private setupRosSubscriptions() {
    if (!this.rclNode) return;
    
    // Subscribe to command topic
    this.commandSubscription = this.rclNode.createSubscription(
      'std_msgs/msg/String',
      'commands',
      (msg: any) => {
        console.log('ROS command received:', msg.data);
        // Process command from ROS
      }
    );

    // Subscribe to camera image topic
    this.imageSubscription = this.rclNode.createSubscription(
      'sensor_msgs/msg/Image',
      '/camera/image_raw', // Topic name for camera images
      (msg: any) => {
        console.log('Received image data');
        // Process image data and save it to disk
    
        const imageBuffer = Buffer.from(msg.data); // Convert message data to buffer
        const savePath = this.getSavePath();
        const filename = `image_${Date.now()}.jpg`;
    
        fs.writeFileSync(path.join(savePath, filename), imageBuffer);
        console.log(`Saved image: ${filename}`);
      }
    );

    // Subscribe to map data (e.g., from SLAM or static map server)
    this.rclNode.createSubscription(
      'nav_msgs/msg/OccupancyGrid',
      'occupancy_grid',//'/map',
      async (msg) => {
        console.log("✅ Received map data from ROS!");

        const compressedMap = await this.handleMapData(msg);
        /*if (compressedMap) {
          console.log("✅ Map data compressed successfully!");*/

        this.pingInterval = setInterval(() => {
          this.sendMapData();
        }, 10000)
      }

    );

    

    this.rclNode.createSubscription('sensor_msgs/msg/Temperature', '/temperature', (msg) => {
      this.sensorData.temperature = msg.temperature || 0;
    });
    
    this.rclNode.createSubscription('geometry_msgs/msg/Twist', '/rover_velocity', (msg) => {
      this.sensorData.speed = msg.linear.x || 0;
    });
    

    
    this.rclNode.createSubscription('sensor_msgs/msg/BatteryState', '/battery_state', (msg) => {
      this.sensorData.batteryLevel = msg.percentage * 100 || 0;
    });
    
    this.rclNode.createSubscription('std_msgs/msg/Float32', '/signal_strength', (msg) => {
      this.sensorData.signalStrength = msg.data || 0;
    });
    
    this.rclNode.createSubscription('std_msgs/msg/Float32MultiArray', '/cpu_usage', (msg :any) => {
      this.sensorData.cpuUsage = msg.data[0] || 0;
    });
    
    this.rclNode.createSubscription('std_msgs/msg/Float32', '/memory_usage', (msg) => {
      this.sensorData.memoryUsage = msg.data || 0;
    });
    
    this.rclNode.createSubscription('geometry_msgs/msg/Point', '/location_on_map', (msg) => {

      this.sensorData.currentPosition= {x:msg.x ,y:msg.y,z:msg.z};
    });
    
    

    
  }

  private getSavePath(): string {
    const now = new Date();
    const year = now.getFullYear();
    const day = now.toISOString().slice(5, 10).replace('-', '-'); // MM-DD format

    const baseDir = path.join(__dirname, 'images', `${year}`);
    const todayDir = path.join(baseDir, day);

    if (!fs.existsSync(todayDir)) {
      fs.mkdirSync(todayDir, { recursive: true });
    }

    return todayDir;
  }

  

  // Function to handle map data and return compressed map data
  private async handleMapData(mapMsg: any): Promise<MapData | null> {
    if (!mapMsg || !mapMsg.info || !mapMsg.data) {
      console.error("Invalid mapMsg received:", mapMsg);
      return null;
    }
    console.log("✅ Processing map data...");

   
    try{
      const mapData:MapData = {
        width: mapMsg.info.width,
        height: mapMsg.info.height,
        resolution: mapMsg.info.resolution,
        origin: {
          x: mapMsg.info.origin.position.x,
          y:mapMsg.info.origin.position.y,
        },
        data: mapMsg.data, // Use the actual map data here
      };
      console.log("✅ Map data structured:", mapData);


      this.mapdata = mapData; // Just store raw object, no compression

      return mapData;
  
      // Compress the map data and return as base64 string
     /* const compressedData = await this.compressMapData(mapData.data);
      this.mapdata = compressedData; // Store compressed map data
      console.log("✅ Map data compressed!");

      return compressedData;*/
      
    } catch (err) {
      console.error('Error compressing map data:', err);
      return null;
    }
  }
  // ✅ Compress Map Data
  private async compressMapData(data: any): Promise<string> {
    return new Promise((resolve, reject) => {
      zlib.gzip(JSON.stringify(data), (err, compressedData) => {
        if (err) {
          console.error("❌ Compression error:", err);

          reject(err);
        } else {
          console.log("✅ Compression successful!");

          resolve(compressedData.toString('base64')); // Convert to base64
        }
      });
    });
  }



     // Function to calculate distance between two GPS coordinates
  private calculateDistance(lat1 : number, lon1:number, lat2 :number, lon2:number) {
    const R = 6371; // Earth radius in km
    const toRad = (deg:number) => (deg * Math.PI) / 180;
  
    const dLat = toRad(lat2 - lat1);
    const dLon = toRad(lon2 - lon1);

    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) *
              Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c*1000; // Distance in km
   

  }

  private updateDistance(lat: number, lon: number): void {
    if (this.lastKnownPosition) {
        const distance = this.calculateDistance(
            this.lastKnownPosition.x,
            this.lastKnownPosition.y,
            lat,
            lon
        );
        console.log("Calculated Distance:", distance);

        if (!isNaN(distance) && distance > 0) {
          this.distanceTraveled = (this.distanceTraveled || 0) + distance;
        }
      }else{
          // Set starting position on first data update
          this.startPosition = { latitude: lat, longitude: lon };
        }

        // Check if the rover has returned to the starting position (within 5m)
        if (this.startPosition) {
            const returnDistance = this.calculateDistance(
                this.startPosition.latitude,
                this.startPosition.longitude,
                lat,
                lon
            );

            if (returnDistance < 5) { // If within 5 meters, count as a completed trip
                this.tripCount += 1;
                console.log("Trip completed! Total trips:", this.tripCount);
            }
        }

        // Update last position
        /*this.lastPosition = { x: lat, y: lon, z:0 };
        this.lastKnownPosition = { x: lat, y: lon ,z:0 }; // Track the last position

        console.log("Updated Distance:", this.distanceTraveled, "m");
        console.log("Last Known Position:", this.lastKnownPosition);*/


      }

    private connect() {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      console.log('Already connected to server.');
      return;
    }

    
    this.ws = new WebSocket(this.serverUrl);
    
    this.ws.on('open', () => {
      console.log('Connected to server');
      this.connected = true;


      // Clear reconnect interval since connection is successful
      if (this.reconnectInterval) {
        clearInterval(this.reconnectInterval);
        this.reconnectInterval = null;
      }

      
      // Send connection message
      this.sendMessage({
        type: 'CONNECT',
        payload: {
          type: 'rover',
          identifier: this.roverIdentifier
        }
      });

      
      // Set up ping interval to keep connection alive
      this.pingInterval = setInterval(() => {
        this.sendSensorData();
      }, 5000);

      
    });
    
    this.ws.on('message', (data) => {
      try {
        const message = JSON.parse(data.toString());
        this.handleMessage(message);
      } catch (error) {
        console.error('Failed to parse message:', error);
      }
    });
    
    this.ws.on('close', () => {
      console.log('Disconnected from server');
      this.connected = false;
      this.roverId = null;


      
      if (this.pingInterval) {
        clearInterval(this.pingInterval);
        this.pingInterval = null;
      }
      
      // Attempt to reconnect
      if (!this.reconnectInterval) {
        this.reconnectInterval = setInterval(() => {
          console.log('Attempting to reconnect...');
          this.connect();
        }, 5000);
      }
    });
    
    this.ws.on('error', (error) => {
      console.error('WebSocket error:', error);
    });
  }
  
  private handleMessage(message: any) {
    switch (message.type) {
      case 'CONNECT':
        // Handle connection response
        if (message.payload.success) {
          console.log('Connection successful');
          this.roverId = message.payload.roverId;

          
          if (this.reconnectInterval) {
            clearInterval(this.reconnectInterval);
            this.reconnectInterval = null;
          }
          
        }
        break;

      case 'SENSOR_DATA':
        if (message.payload.latitude && message.payload.longitude) {
          this.updateDistance(message.payload.latitude, message.payload.longitude);
        }
        break;

      
      case 'COMMAND':
        // Handle command request
        this.handleCommand(message);
        break;
      
      case 'ERROR':
        console.error('Server error:', message.payload);
        break;
    }
  }
  
  private handleCommand(message: any) {
    
    if (!message || !message.payload) {
      console.error('Invalid message structure:', message);
      return;
    }

    const command = message.payload.command;
    const commandId = message.payload.commandId;

    if (!command || typeof command !== 'string') {
      console.error(`Invalid command received: ${command} (ID: ${commandId})`);
      return;
    }

    if (this.lastCommandId !== null && this.lastCommandId === commandId) {
      console.log(`Duplicate command ignored: ${command} (ID: ${commandId})`);
      return;
    }
    this.lastCommandId = commandId; // Track last processed command

    console.log(`Received command: ${command} (ID: ${commandId})`);


  
    // Process command
    let response = '';
    let status = 'success';
    
    try {
      
      
      // Parse and execute command
      const parts = command.split(' ');
      const action = parts[0].toLowerCase();

      if (!this.rclNode) {
        console.error('ROS 2 node not initialized.');
        status = 'failed';
        response = 'ROS 2 node is not available.';

      }else{
        const publisher = this.rclNode.createPublisher('std_msgs/msg/String', 'rover_commands');
      
        switch (action) {
          case 'move':
            if (parts.length < 3) {
              throw new Error('Move command format is incorrect. Expected: move <direction> <distance>');
            }

            // Handle movement command
            const direction = parts[1];
            const distance = parseFloat(parts[2]);

            if (isNaN(distance)) {
              throw new Error('Invalid distance value for move command.');
            }

            response = `Moving ${direction} ${distance} units`;
            console.log(response);

        
            // Publish to ROS 2
            publisher.publish({ data: command });
            console.log(`Published to ROS 2 topic: ${command}`);
            break;
        
          case 'stop':
            console.log('Emergency stop');
            response = 'Emergency stop engaged';
          
            // Publish to ROS2 topic
            publisher.publish({data : command});
            console.log(`Published to ROS 2 topic: ${command}`);
            break;
                


        
          case 'camera':
            if (parts.length < 2) {
              throw new Error('Camera command format is incorrect. Expected: camera <action>');
            }

            // Handle camera command
            const cameraAction = parts[1];
            response = `Camera ${cameraAction} command executed`;
            console.log(response);

            break;
        
          default:
            response = `Unknown command: ${action}`;
            status = 'failed';
            console.error(response);
        }
      }
    } catch (error) {
      console.error('Command execution error:', error);
      status = 'failed';
      response = `Error executing command: ${(error as Error).message}`;
    }
    
    // Send response back to server
    this.sendMessage({
      type: 'COMMAND',
      roverId: this.roverId,
      payload: {
        commandId,
        status,
        response
      }
    });
  }

  private  generateSensorData():SensorData {
    // In a real implementation, this would read from actual sensors via ROS2
    // Here we're generating mock data for demonstration
    const newLatitude = parseFloat((34.0522 + (Math.random() * 0.001 - 0.0005)).toFixed(5));
    const newLongitude = parseFloat((-118.2437 + (Math.random() * 0.001 - 0.0005)).toFixed(5));
     
// Update distance traveled
    this.updateDistance(newLatitude, newLongitude);



    return this.sensorData;
    /*{
      temperature:parseFloat(( 23.4 + (Math.random() * 2 - 1)).toFixed(2)), // 22.4 to 24.4 °C
      //humidity: parseFloat((42 + (Math.random() * 6 - 3)).toFixed(2)), // 39% to 45%
      //pressure: parseFloat((1013 + (Math.random() * 10 - 5)).toFixed(2)), // 1008 to 1018 hPa
      //altitude: parseFloat((124 + (Math.random() * 4 - 2)).toFixed(2)), // 122m to 126m
      //heading: parseFloat((275 + (Math.random() * 10 - 5)).toFixed(2)), // 270° to 280°
      speed:parseFloat(( 0.4 + (Math.random() * 0.2 - 0.1)).toFixed(2)), // 0.3 to 0.5 m/s
      //tilt: parseFloat((2.1 + (Math.random() * 1 - 0.5)).toFixed(2)), // 1.6° to 2.6°
      latitude:newLatitude,
      longitude:newLongitude,
      batteryLevel:parseFloat(( 87 - (Math.random() * 0.1)).toFixed(2)), // Slowly decreasing battery
      signalStrength: parseFloat((85 + (Math.random() * 10 - 5)).toFixed(2)), // 80% to 90%

      cpuUsage: parseFloat((30 + (Math.random() * 40)).toFixed(2)), // 30% to 70% CPU usage
      memoryUsage: parseFloat((50 + (Math.random() * 20)).toFixed(2)), // 50% to 70% Memory usage
      distanceTraveled: this.distanceTraveled || 0,  // Include total distance traveled
      trips: this.tripCount, // New trip count added
      lastPosition: this.lastKnownPosition, // Last known position added
    };*/
  }
  
  private sendSensorData() {
    if (!this.connected || !this.roverId){
      console.log('Skipping sensor data send: Not connected or missing rover ID');
      //writelogs(s);
      return;

    } 
   
    // Get sensor data
    const sensorData = this.generateSensorData();

    // Send telemetry data to server
    this.sendMessage({
      type: 'TELEMETRY',
      roverId: this.roverId,
      payload: {
        sensorData
      }
    });
    
    
    if(this.currentPosition){
    console.log("X:",this.currentPosition.x , "Y:", this.currentPosition.y, "Z:", this.currentPosition.z);
    }
    
    
    
    // Update rover status
    this.sendMessage({
      type: 'STATUS_UPDATE',
      roverId: this.roverId,
      payload: {
        status: 'active'
      }
    });
  }

  // ✅ Send map data separately
  private async sendMapData() {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN || !this.mapdata) return;

    this.ws.send(
      JSON.stringify({
        type: "MAP_DATA",
        roverId: 1,
        timestamp: Date.now(),

        payload: { mapdata: this.mapdata },
      })
    );

    console.log("Sent MAP_DATA");
  }

  
  private sendMessage(message: any) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      console.log('Sending message:', message);
      this.ws.send(JSON.stringify(message));
    }
    else {
        console.log('WebSocket not open, unable to send message.');
        this.connect();
      }
  }

  
  async shutdown() {
    // Clear intervals
    if (this.pingInterval) {
      clearInterval(this.pingInterval);
      this.pingInterval = null;
    }
    
    if (this.reconnectInterval) {
      clearInterval(this.reconnectInterval);
      this.reconnectInterval = null;
    }
    
    // Close WebSocket connection
    if (this.ws) {
      this.ws.terminate();
      this.ws = null;
    }
    
    // Shutdown ROS2 node
    if (this.rclNode) {
      this.rclNode.destroy();
      await rclnodejs.shutdown();
      this.rclNode = null;
    }
    
    console.log('Rover client shutdown complete');
  }
}

// Example usage
const SERVER_URL = process.env.SERVER_URL || 'ws://172.26.220.135:5000/ws';  //*'ws://172.26.220.135:5000/ws'; 172.26.220.135 
const ROVER_ID = process.env.ROVER_ID || 'R_001';

const roverClient = new RoverClient(SERVER_URL, ROVER_ID);
roverClient.initialize().catch(console.error);

// Handle process termination
process.on('SIGINT', async () => {
  console.log('Shutting down rover client...');
  await roverClient.shutdown();
  process.exit(0);
});
