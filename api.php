<?php

// =========================================================================
// Configuration
// =========================================================================
// Set the IP address of your ESP32-C3 here.
// Make sure to include the http:// part.
// IMPORTANT: This IP should be the address of the ESP32, not the Orange Pi.
$esp32_base_url = "http://192.168.1.4";


// =========================================================================
// Script Logic
// =========================================================================

// Set a content type header to ensure the browser and client-side JavaScript
// correctly interpret the response as JSON or plain text.
// This is important for returning data from the ESP32.
header('Content-Type: application/json');

// Get the requested path from the URL. This is passed in the query string.
$path = isset($_GET['path']) ? $_GET['path'] : '/data';

// Get the 'value' parameter if it exists. This is used for commands like setting brightness.
$value = isset($_GET['value']) ? $_GET['value'] : null;

// Construct the full URL to the ESP32.
// We append the path and value to the base URL.
$full_url = $esp32_base_url . $path;

if ($value !== null) {
    // If a value is provided, add it as a query parameter.
    // We use http_build_query to ensure it's properly formatted.
    $full_url .= '?' . http_build_query(['value' => $value]);
}

// Attempt to fetch the content from the ESP32.
// Use file_get_contents for a simple GET request.
$response = @file_get_contents($full_url);

// Check if the request was successful.
if ($response === false) {
    // If the request fails, return a JSON error message.
    // This allows the front-end to detect that the ESP32 is offline or the request failed.
    http_response_code(503); // Service Unavailable
    echo json_encode([
        'error' => 'Failed to connect to ESP32.',
        'details' => error_get_last() // Get the last PHP error for debugging.
    ]);
} else {
    // If successful, return the response from the ESP32 directly to the browser.
    // This assumes the ESP32 returns valid JSON or plain text.
    echo $response;
}

?>
