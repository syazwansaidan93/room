<?php
    // Set the content type header to application/json
    header('Content-Type: application/json');

    // The local IP address of your ESP32. This value is now stored securely on your server.
    $esp32_base_url = "http://192.168.1.4";

    // Get the requested endpoint from the URL query string.
    $endpoint = isset($_GET['endpoint']) ? $_GET['endpoint'] : null;

    // If no endpoint is provided, return an error.
    if ($endpoint === null) {
        http_response_code(400); // Bad Request
        echo json_encode(["error" => "Missing endpoint parameter"]);
        exit;
    }

    // Get the optional 'value' from the query string for settings.
    $value = isset($_GET['value']) ? $_GET['value'] : null;

    // Construct the full URL to the ESP32.
    $full_url = $esp32_base_url . $endpoint;

    if ($value !== null) {
        $full_url .= "?value=" . urlencode($value);
    }

    // Use file_get_contents to fetch data from the ESP32.
    // The '@' symbol suppresses warnings, which is useful for avoiding errors
    // when the ESP32 might be offline, but you should handle it with a check.
    $response = @file_get_contents($full_url);

    // Check if the request was successful.
    if ($response === FALSE) {
        // If the request failed, return a 503 Service Unavailable error.
        http_response_code(503);
        echo json_encode(["error" => "Could not connect to ESP32."]);
    } else {
        // If the request was successful, pass the ESP32's response back to the client.
        echo $response;
    }
?>
