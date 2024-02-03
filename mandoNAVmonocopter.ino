#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> 

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);


// Replace with your network credentials
const char *ssid = "";
const char *password = "";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output1State = "off";


// Assign output variables to GPIO pins
const int output1 = 6; 

void setup() {

  Serial.begin(115200);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  
  // Initialize the output variables as outputs
  pinMode(output1, OUTPUT);
  digitalWrite(output1, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
server.begin();
display.clearDisplay(); //Borra el buffer
display.setTextSize(1); //Establece el tamaño de fuente, admite tamaños de 1 a 8
display.setTextColor(WHITE); //Establece el color 
display.setCursor(35,10); //Establecer las coordenadas para mostrar la posición del texto
display.println("IP:");
display.setCursor(20,20); //Establecer las coordenadas para mostrar la posición del texto
display.println(WiFi.localIP()); 
display.display(); //Muestra el texto 
    delay(5000);

  
}

void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // turns the GPIOs on and off
            if (header.indexOf("GET /1/on") >= 0) {
              Serial.println("LOAD1 on");
              output1State = "on";
              digitalWrite(output1, HIGH);
            } else if (header.indexOf("GET /1/off") >= 0) {
              Serial.println("LOAD1 off");
              output1State = "off";
              digitalWrite(output1, LOW);
            } 

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #5B196A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #5B196A;}</style></head>");

            // Web Page Heading
            client.println("<body><h1>HOME AUTOMATION DUAL OUTPUT</h1>");

            // Display current state, and ON/OFF buttons for OUTPUT1 
            client.println("<p>LOAD1 - State " + output1State + "</p>");

            // If the output1State is off, it displays the ON button       
            if (output1State=="off") {
              client.println("<p><a href=\"/1/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/1/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 

            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}



///////////////////


// solo mueve slide responde motor , 12c probablement efuncione


 #include <WiFi.h>
#include <WebServer.h>
#include <Deneyap_Servo.h>      // Deneyap Servo kütüphanesi eklenmesi
#include "I2Cdev.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include "BMP085.h"
#include "Wire.h"


Servo servo_modo;
Servo ESC;
Servo servo_aleron;

HMC5883L magnetometro;
int16_t mx, my, mz;
float ang_mag;

MPU6050 MPU;
// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int16_t ax, ay, az;
int16_t gx, gy, gz;

long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

BMP085 barometer;
float temperature;
float pressure;
int32_t lastMicros;

int amplitud;
int servo_ang;

//////////////////////////////////////////
// Replace with your network credentials
const char* ssid = "";
const char* password = "";

// Variables to store the current state of the slider, joystick, and switch
int sliderValue = 0;
int scaledX, scaledY;
int joystickXValue = 0;
int joystickYValue = 0;

bool switchValue = false;

WebServer server(80);  // Create a webserver object that listens for HTTP request on port 80

void scaleJoystickValues(int rawX, int rawY, int& scaledX, int& scaledY) {
  // Calculate angle in radians
  float angle = atan2(rawY, rawX);

  // Convert radians to degrees
  int degrees = int(angle * 180 / PI);

  // Adjust for negative values
  if (degrees < 0) {
    degrees += 360;
  }

  // Scale the angle to the desired range (0 to 360)
  degrees = (degrees + 90) % 360;

  // Update scaled X and Y values
  scaledX = degrees;
  scaledY = 0;  // Adjust as needed based on your specific requirements
}

void setup(void){
  Serial.begin(115200);
  WiFi.begin(ssid, password);  // Connect to the network
  Serial.println("");

  // Wait for the Wi-Fi to connect
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  // Print the IP to the serial monitor

  server.on("/", handleRoot);  // Call the 'handleRoot' function when a client requests URI "/"
  server.on("/slider", handleSlider);  // Call the 'handleSlider' function when a client requests URI "/slider"
  server.on("/joystick", HTTP_GET, handleJoystick);  // Call the 'handleJoystick' function when a client requests URI "/joystick"
  server.on("/switch", handleSwitch);  // Call the 'handleSwitch' function when a client requests URI "/switch"

  server.begin();  // Actually start the server
  Serial.println("HTTP server started");

  ///////////////////////////
  servo_aleron.attach(7);
  servo_aleron.write(0); //servo_aleron.write(45); // posicion central^^^^
  //servo_modo.attach(-1);
  //servo_modo.write(127); // posicion central
  //ESC.attach(6);
  //ESC.write(0); // velociddad minima
  //delay(2500);

 Wire.begin();
    MPU.initialize();    //Iniciando el MPU
    if (MPU.testConnection()) Serial.println("MPU iniciado correctamente");
    else Serial.println("Error al iniciar el MPU");
    MPU.setI2CBypassEnabled(true); // set bypass mode
    magnetometro.initialize();
      Serial.println(magnetometro.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

    barometer.initialize();
      Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");
  
  
}

void loop(void){
  server.handleClient();  // Handle a new client request

  // Leer las aceleraciones y velocidades angulares
    MPU.getAcceleration(&ax, &ay, &az);
    MPU.getRotation(&gx, &gy, &gz);
    
  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  float accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y filtro complemento  
  ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
  ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
    
  ang_x_prev=ang_x;
  ang_y_prev=ang_y;

    //Obtenemos del magnetómetro las componentes del campo magnético
    magnetometro.getHeading(&mx, &my, &mz);

    //Calculamos el ángulo del eje X con respecto al norte
    ang_mag = atan2(my, mx);
    ang_mag=ang_mag*(180/M_PI);//convertimos de Radianes a grados
    //calculamos el ángulo equivalente de [-180 180] a [0 360]
    if(ang_mag<0) ang_mag=ang_mag+360;
    
   // servo_modo.write(datos.joy1_X);
    //servo_aleron.write(datos.pot1);
    //ESC.write(sliderValue);

    int angle = scaledX; // varia entre 0 y 360
    
    amplitud = (switchValue == 1) ? 7: 0; // 35:0

    //amplitud = map(datos.pot1, 1, 180, 0, 35);// el mas adecuado, pero por velocidad de trnamision de datos parece fallar
    
    servo_ang = 55 + amplitud * cos((ang_mag - angle) * M_PI / 180); //  motor_speed = 85+45XCOS... 
    servo_aleron.write(servo_ang);
 
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><script>";
  html += "function updateJoystick(event) {";
  html += "  var joystick = document.getElementById('joystick');";
  html += "  var joystickDot = document.getElementById('joystickDot');";
  html += "  var rect = joystick.getBoundingClientRect();";
  html += "  var x = event.clientX - rect.left;";
  html += "  var y = event.clientY - rect.top;";
  html += "  var radius = joystick.clientWidth / 2;";
  html += "  var angle = Math.atan2(y - radius, x - radius);";
  html += "  var distance = Math.min(radius, Math.hypot(x - radius, y - radius));";
  html += "  var newX = radius + distance * Math.cos(angle);";
  html += "  var newY = radius + distance * Math.sin(angle);";
  html += "  joystickDot.style.left = newX + 'px';";
  html += "  joystickDot.style.top = newY + 'px';";
  html += "  var normalizedX = (newX - radius) / radius * 100;";
  html += "  var normalizedY = (newY - radius) / radius * 100;";
  html += "  fetch('/joystick?x=' + normalizedX + '&y=' + normalizedY);";
  html += "}";
  html += "</script></head><body>";
  html += "<h2>ESP32 Controller</h2>";
  html += "<p>Slider: <input type='range' id='slider' name='slider' min='0' max='255' oninput='fetch(\"/slider?value=\" + this.value)'></p>";
  html += "<div class='joystick' id='joystick' style='width: 200px; height: 200px; background-color: #f0f0f0; border-radius: 50%; position: relative;' onmousemove='updateJoystick(event)'>";
  html += "  <div id='joystickDot' style='width: 20px; height: 20px; background-color: #5B196A; border-radius: 50%; position: absolute; top: " + String(joystickYValue) + "%; left: " + String(joystickXValue) + "%; transform: translate(-50%, -50%);'></div>";
  html += "</div>";
  html += "<p>Switch: <input type='checkbox' id='switch' name='switch' onchange='fetch(\"/switch?value=\" + (this.checked ? 1 : 0))'></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}


void handleSlider() {
  sliderValue = server.arg("value").toInt();  // Get the value of the slider
 // Serial.println(sliderValue);
  analogWrite(6, sliderValue);   //ESC.write(sliderValue);
  server.send(200, "text/plain", "Slider value updated");  // Send a response to the client
}

void handleJoystick() {
  int rawX = server.arg("x").toInt();  // Get the raw X value of the joystick
  int rawY = server.arg("y").toInt();  // Get the raw Y value of the joystick
  scaleJoystickValues(rawX, rawY, scaledX, scaledY);

 // Serial.println("Joystick X: " + String(scaledX) + ", Joystick Y: " + String(scaledY));
  server.send(200, "text/plain", "Joystick value updated");  // Send a response to the client

}

void handleSwitch() {
  switchValue = server.arg("value").toInt() == 1;  // Get the value of the switch
 // Serial.println(switchValue);
  server.send(200, "text/plain", "Switch value updated");  // Send a response to the client
}




///////////////////777
