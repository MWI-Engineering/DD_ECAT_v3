// udp_communication.c - UDP Communication Protocol for Pi
#include "udp_communication.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <json-c/json.h>

#define UDP_PORT 8888
#define MAX_BUFFER_SIZE 1024
#define STATUS_UPDATE_INTERVAL_MS 100

// Global variables
static int udp_socket = -1;
static struct sockaddr_in server_addr, client_addr;
static socklen_t client_addr_len = sizeof(client_addr);
static int udp_running = 0;
static pthread_t udp_thread, status_thread;
static pthread_mutex_t status_mutex = PTHREAD_MUTEX_INITIALIZER;

// System status structure
static system_status_t current_status = {
    .wheel_position = 0.0f,
    .wheel_speed = 0.0f,
    .button_state = 0,
    .ffb_enabled = true,
    .force_multiplier = 1.0f,
    .current_effect = {0},
    .motor_current = 0.0f,
    .motor_temperature = 25.0f,
    .motor_voltage = 24.0f,
    .motor_load = 0.0f,
    .is_calibrated = false,
    .emergency_stop = false
};

// External references to your existing systems
extern void motor_emergency_stop(void);
extern void motor_calibrate(void);
extern void ffb_enable(bool enable);
extern void ffb_set_force_multiplier(float multiplier);
extern void ffb_reset_all_effects(void);
extern float motor_get_current(void);
extern float motor_get_temperature(void);
extern float motor_get_voltage(void);
extern float motor_get_load(void);
extern float wheel_get_position(void);
extern float wheel_get_speed(void);
extern uint16_t wheel_get_button_state(void);
extern void wheel_get_current_ffb_effect(char* effect_name, int max_len);

// Function prototypes
static void* udp_receiver_thread(void* arg);
static void* status_sender_thread(void* arg);
static void process_command(const char* command, const char* client_ip);
static void send_status_update(void);
static void send_response(const char* response, const char* client_ip);
static json_object* create_status_json(void);
static void update_system_status(void);

/**
 * @brief Initialize UDP communication system
 * @return 0 on success, -1 on failure
 */
int udp_communication_init(void) {
    // Create UDP socket
    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
        perror("UDP: Failed to create socket");
        return -1;
    }

    // Set socket options
    int opt = 1;
    if (setsockopt(udp_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("UDP: Failed to set socket options");
        close(udp_socket);
        return -1;
    }

    // Configure server address
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(UDP_PORT);

    // Bind socket
    if (bind(udp_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("UDP: Failed to bind socket");
        close(udp_socket);
        return -1;
    }

    printf("UDP: Communication initialized on port %d\n", UDP_PORT);
    return 0;
}

/**
 * @brief Start UDP communication threads
 * @return 0 on success, -1 on failure
 */
int udp_communication_start(void) {
    udp_running = 1;

    // Create receiver thread
    if (pthread_create(&udp_thread, NULL, udp_receiver_thread, NULL) != 0) {
        perror("UDP: Failed to create receiver thread");
        return -1;
    }

    // Create status sender thread
    if (pthread_create(&status_thread, NULL, status_sender_thread, NULL) != 0) {
        perror("UDP: Failed to create status thread");
        pthread_cancel(udp_thread);
        return -1;
    }

    printf("UDP: Communication threads started\n");
    return 0;
}

/**
 * @brief Stop UDP communication and cleanup
 */
void udp_communication_stop(void) {
    udp_running = 0;

    // Wait for threads to finish
    if (udp_thread) {
        pthread_join(udp_thread, NULL);
    }
    if (status_thread) {
        pthread_join(status_thread, NULL);
    }

    // Close socket
    if (udp_socket >= 0) {
        close(udp_socket);
        udp_socket = -1;
    }

    printf("UDP: Communication stopped\n");
}

/**
 * @brief UDP receiver thread - handles incoming commands
 */
static void* udp_receiver_thread(void* arg) {
    char buffer[MAX_BUFFER_SIZE];
    char client_ip[INET_ADDRSTRLEN];

    printf("UDP: Receiver thread started\n");

    while (udp_running) {
        // Receive data
        ssize_t bytes_received = recvfrom(udp_socket, buffer, MAX_BUFFER_SIZE - 1, 0,
                                         (struct sockaddr*)&client_addr, &client_addr_len);

        if (bytes_received > 0) {
            buffer[bytes_received] = '\0';
            
            // Get client IP
            inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip, INET_ADDRSTRLEN);
            
            printf("UDP: Received command from %s: %s\n", client_ip, buffer);
            
            // Process command
            process_command(buffer, client_ip);
        } else if (bytes_received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("UDP: Error receiving data");
            break;
        }

        usleep(1000); // 1ms delay
    }

    printf("UDP: Receiver thread stopped\n");
    return NULL;
}

/**
 * @brief Status sender thread - sends periodic status updates
 */
static void* status_sender_thread(void* arg) {
    printf("UDP: Status sender thread started\n");

    while (udp_running) {
        update_system_status();
        send_status_update();
        usleep(STATUS_UPDATE_INTERVAL_MS * 1000); // Convert to microseconds
    }

    printf("UDP: Status sender thread stopped\n");
    return NULL;
}

/**
 * @brief Process incoming commands from GUI
 */
static void process_command(const char* command, const char* client_ip) {
    char response[256];
    
    if (strcmp(command, "EMERGENCY_STOP") == 0) {
        pthread_mutex_lock(&status_mutex);
        current_status.emergency_stop = true;
        pthread_mutex_unlock(&status_mutex);
        
        motor_emergency_stop();
        sprintf(response, "EMERGENCY_STOP_ACKNOWLEDGED");
        printf("UDP: Emergency stop activated\n");
        
    } else if (strcmp(command, "CALIBRATE") == 0) {
        pthread_mutex_lock(&status_mutex);
        current_status.is_calibrated = false;
        pthread_mutex_unlock(&status_mutex);
        
        motor_calibrate();
        
        pthread_mutex_lock(&status_mutex);
        current_status.is_calibrated = true;
        pthread_mutex_unlock(&status_mutex);
        
        sprintf(response, "CALIBRATION_COMPLETE");
        printf("UDP: Calibration completed\n");
        
    } else if (strcmp(command, "ENABLE_FFB") == 0) {
        pthread_mutex_lock(&status_mutex);
        current_status.ffb_enabled = true;
        pthread_mutex_unlock(&status_mutex);
        
        ffb_enable(true);
        sprintf(response, "FFB_ENABLED");
        printf("UDP: FFB enabled\n");
        
    } else if (strcmp(command, "DISABLE_FFB") == 0) {
        pthread_mutex_lock(&status_mutex);
        current_status.ffb_enabled = false;
        pthread_mutex_unlock(&status_mutex);
        
        ffb_enable(false);
        sprintf(response, "FFB_DISABLED");
        printf("UDP: FFB disabled\n");
        
    } else if (strncmp(command, "SET_FORCE_MULTIPLIER:", 21) == 0) {
        float multiplier = atof(command + 21);
        if (multiplier >= 0.0f && multiplier <= 2.0f) {
            pthread_mutex_lock(&status_mutex);
            current_status.force_multiplier = multiplier;
            pthread_mutex_unlock(&status_mutex);
            
            ffb_set_force_multiplier(multiplier);
            sprintf(response, "FORCE_MULTIPLIER_SET:%.2f", multiplier);
            printf("UDP: Force multiplier set to %.2f\n", multiplier);
        } else {
            sprintf(response, "ERROR:INVALID_FORCE_MULTIPLIER");
            printf("UDP: Invalid force multiplier: %.2f\n", multiplier);
        }
        
    } else if (strcmp(command, "RESET_EFFECTS") == 0) {
        ffb_reset_all_effects();
        sprintf(response, "EFFECTS_RESET");
        printf("UDP: All effects reset\n");
        
    } else if (strcmp(command, "GET_STATUS") == 0) {
        // Status will be sent by the status thread
        sprintf(response, "STATUS_REQUEST_ACKNOWLEDGED");
        
    } else if (strcmp(command, "PING") == 0) {
        sprintf(response, "PONG");
        
    } else {
        sprintf(response, "ERROR:UNKNOWN_COMMAND");
        printf("UDP: Unknown command: %s\n", command);
    }

    // Send response
    send_response(response, client_ip);
}

/**
 * @brief Send response to client
 */
static void send_response(const char* response, const char* client_ip) {
    struct sockaddr_in response_addr;
    memset(&response_addr, 0, sizeof(response_addr));
    response_addr.sin_family = AF_INET;
    response_addr.sin_port = htons(8889); // Response port
    inet_pton(AF_INET, client_ip, &response_addr.sin_addr);

    ssize_t bytes_sent = sendto(udp_socket, response, strlen(response), 0,
                               (struct sockaddr*)&response_addr, sizeof(response_addr));
    
    if (bytes_sent < 0) {
        perror("UDP: Failed to send response");
    }
}

/**
 * @brief Update system status from various subsystems
 */
static void update_system_status(void) {
    pthread_mutex_lock(&status_mutex);
    
    // Update wheel status
    current_status.wheel_position = wheel_get_position();
    current_status.wheel_speed = wheel_get_speed();
    current_status.button_state = wheel_get_button_state();
    
    // Update motor status
    current_status.motor_current = motor_get_current();
    current_status.motor_temperature = motor_get_temperature();
    current_status.motor_voltage = motor_get_voltage();
    current_status.motor_load = motor_get_load();
    
    // Update current FFB effect
    wheel_get_current_ffb_effect(current_status.current_effect, sizeof(current_status.current_effect));
    
    // Update timestamp
    current_status.timestamp = time(NULL);
    
    pthread_mutex_unlock(&status_mutex);
}

/**
 * @brief Send status update to all connected clients
 */
static void send_status_update(void) {
    json_object* status_json = create_status_json();
    const char* json_string = json_object_to_json_string(status_json);
    
    // Broadcast to all known clients (you might want to maintain a client list)
    // For now, we'll send to the last known client
    if (client_addr_len > 0) {
        ssize_t bytes_sent = sendto(udp_socket, json_string, strlen(json_string), 0,
                                   (struct sockaddr*)&client_addr, client_addr_len);
        
        if (bytes_sent < 0) {
            perror("UDP: Failed to send status update");
        }
    }
    
    json_object_put(status_json);
}

/**
 * @brief Create JSON object with current system status
 */
static json_object* create_status_json(void) {
    json_object* root = json_object_new_object();
    json_object* wheel_obj = json_object_new_object();
    json_object* motor_obj = json_object_new_object();
    json_object* ffb_obj = json_object_new_object();
    
    pthread_mutex_lock(&status_mutex);
    
    // Wheel status
    json_object_object_add(wheel_obj, "position", json_object_new_double(current_status.wheel_position));
    json_object_object_add(wheel_obj, "speed", json_object_new_double(current_status.wheel_speed));
    json_object_object_add(wheel_obj, "buttons", json_object_new_int(current_status.button_state));
    
    // Motor status
    json_object_object_add(motor_obj, "current", json_object_new_double(current_status.motor_current));
    json_object_object_add(motor_obj, "temperature", json_object_new_double(current_status.motor_temperature));
    json_object_object_add(motor_obj, "voltage", json_object_new_double(current_status.motor_voltage));
    json_object_object_add(motor_obj, "load", json_object_new_double(current_status.motor_load));
    
    // FFB status
    json_object_object_add(ffb_obj, "enabled", json_object_new_boolean(current_status.ffb_enabled));
    json_object_object_add(ffb_obj, "force_multiplier", json_object_new_double(current_status.force_multiplier));
    json_object_object_add(ffb_obj, "current_effect", json_object_new_string(current_status.current_effect));
    
    // System status
    json_object_object_add(root, "wheel", wheel_obj);
    json_object_object_add(root, "motor", motor_obj);
    json_object_object_add(root, "ffb", ffb_obj);
    json_object_object_add(root, "calibrated", json_object_new_boolean(current_status.is_calibrated));
    json_object_object_add(root, "emergency_stop", json_object_new_boolean(current_status.emergency_stop));
    json_object_object_add(root, "timestamp", json_object_new_int64(current_status.timestamp));
    
    pthread_mutex_unlock(&status_mutex);
    
    return root;
}

/**
 * @brief Get current system status (for other modules)
 */
void udp_get_system_status(system_status_t* status) {
    pthread_mutex_lock(&status_mutex);
    *status = current_status;
    pthread_mutex_unlock(&status_mutex);
}

/**
 * @brief Set emergency stop state
 */
void udp_set_emergency_stop(bool emergency_stop) {
    pthread_mutex_lock(&status_mutex);
    current_status.emergency_stop = emergency_stop;
    pthread_mutex_unlock(&status_mutex);
}

/**
 * @brief Log UDP communication events
 */
void udp_log_event(const char* event) {
    time_t now;
    struct tm* timeinfo;
    char timestamp[64];
    
    time(&now);
    timeinfo = localtime(&now);
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);
    
    printf("UDP LOG [%s]: %s\n", timestamp, event);
}
