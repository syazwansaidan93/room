# Room Controller Dashboard

This project provides a web-based dashboard for controlling and monitoring an ESP32-C3 device. The setup uses an Orange Pi as a host for the web server, which acts as a proxy to communicate with the ESP32. Cloudflare Tunnel is used to make the dashboard accessible from anywhere on the internet.

### 🔌 Components

* **ESP32-C3:** The core device that runs the logic for controlling devices (e.g., fan, LEDs) and reads sensor data (temperature, humidity, light).
* **Orange Pi Zero 3:** A single-board computer that hosts the web dashboard. It runs a web server (like Apache or Nginx) with PHP support.
* **PHP Script (`api.php`):** A proxy script that runs on the Orange Pi. Its sole purpose is to forward requests from the web browser to the ESP32, bypassing potential cross-origin issues and centralizing the ESP32's IP address.
* **HTML Dashboard (`index.html`):** The user interface served by the Orange Pi. This file contains the JavaScript that sends commands and fetches data through the PHP proxy.
* **Cloudflare Tunnel:** A service that securely exposes the Orange Pi's web server to the internet without requiring a public IP address or port forwarding.

### ⚙️ Setup Guide

#### 1. ESP32-C3

Ensure your ESP32-C3 is programmed with firmware that exposes a web server with the necessary API endpoints. The PHP proxy script expects endpoints like `/data`, `/on-perm`, `/set-brightness`, etc., which should return JSON or plain text data.

#### 2. Orange Pi Server

1.  **Install a Web Server with PHP:** Make sure you have a web server like Apache or Nginx installed on your Orange Pi and that it's configured to process PHP files.
2.  **Place the Files:** Place both `index.html` and `api.php` in your web server's document root directory (e.g., `/var/www/html/` or `/home/wan/`).

#### 3. PHP Configuration (`api.php`)

Open the `api.php` file and modify the `$esp32_base_url` variable to match the local IP address of your ESP32-C3.

```php
// Set the IP address of your ESP32-C3 here.
$esp32_base_url = "[http://192.168.1.4](http://192.168.1.4)"; 
```

#### 4. Cloudflare Tunnel

Set up a Cloudflare Tunnel to expose your Orange Pi's web server to the internet. This allows you to access your dashboard from a public URL provided by Cloudflare, without needing to open ports on your router.

### 📂 Project Structure

```
.
├── index.html
└── api.php
```

### 📋 File Explanations

#### `index.html`

This is the main dashboard file. It uses HTML, CSS (via Tailwind CSS), and JavaScript. The JavaScript is configured to send all API requests to `api.php`, making the frontend agnostic to the ESP32's actual IP address.

#### `api.php`

This script is the crucial link between the dashboard and your ESP32. It performs the following functions:
* **Proxy:** It accepts requests from `index.html` and forwards them to the correct endpoint on your ESP32.
* **IP Hiding:** It prevents the ESP32's private IP address from being exposed in the client-side JavaScript.
* **Error Handling:** It includes basic error handling to gracefully inform the dashboard if it fails to connect to the ESP32.

### 🚀 Usage

Once all components are set up and running, you can access the dashboard by navigating to the public URL provided by your Cloudflare Tunnel. The dashboard will automatically begin fetching data and allow you to send commands to your ESP32-C3.
