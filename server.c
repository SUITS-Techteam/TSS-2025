///////////////////////////////////////////////////////////////////////////////////
//                                   Headers
///////////////////////////////////////////////////////////////////////////////////

// Standard C Stuff
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include "cJSON.h"

// Networking Headers
#include "network.h"

// Application
#include "server_data.h"

// Server Variables
#define MAX_LINE_LENGHT 1024

#define TSS_TO_UNREAL_BRAKES_COMMAND 2000
#define TSS_TO_UNREAL_LIGHTS_COMMAND 2001
#define TSS_TO_UNREAL_STEERING_COMMAND 2002
#define TSS_TO_UNREAL_THROTTLE_COMMAND 2003
#define TSS_TO_UNREAL_SWITCH_DEST_COMMAND 2004

// Uncomment this for extra print statements
//#define VERBOSE_MODE 
//#define TESTING_MODE

///////////////////////////////////////////////////////////////////////////////////
//                          Windows Functions
///////////////////////////////////////////////////////////////////////////////////


#if defined(_WIN32)
//This is from https://stackoverflow.com/questions/5404277/porting-clock-gettime-to-windows
void unix_time(struct timespec *spec)
{  __int64 wintime; GetSystemTimeAsFileTime((FILETIME*)&wintime); 
   wintime -=w2ux;  spec->tv_sec  =wintime / exp7;                 
                    spec->tv_nsec =wintime % exp7 *100;
}
int clock_gettime(int fake, struct timespec *spec)
{  static  struct timespec startspec; static double ticks2nano;
   static __int64 startticks, tps =0;    __int64 tmp, curticks;
   QueryPerformanceFrequency((LARGE_INTEGER*)&tmp); //some strange system can
   if (tps !=tmp) { tps =tmp; //init ~~ONCE         //possibly change freq ?
                    QueryPerformanceCounter((LARGE_INTEGER*)&startticks);
                    unix_time(&startspec); ticks2nano =(double)exp9 / tps; }
   QueryPerformanceCounter((LARGE_INTEGER*)&curticks); curticks -=startticks;
   spec->tv_sec  =startspec.tv_sec   +         (curticks / tps);
   spec->tv_nsec =startspec.tv_nsec  + (double)(curticks % tps) * ticks2nano;
         if (!(spec->tv_nsec < exp9)) { spec->tv_sec++; spec->tv_nsec -=exp9; }
   return 0;
}

#define CLOCK_REALTIME 0
#endif



///////////////////////////////////////////////////////////////////////////////////
//                      Helper Functions Declarations
///////////////////////////////////////////////////////////////////////////////////

bool continue_server();
//bool big_endian();
void get_contents();
//void reverse_bytes();
void tss_to_unreal();

///////////////////////////////////////////////////////////////////////////////////
//                               Main Functions
///////////////////////////////////////////////////////////////////////////////////

enum { NS_PER_SECOND = 1000000000 };

void sub_timespec(struct timespec t1, struct timespec t2, struct timespec *td)
{
    td->tv_nsec = t2.tv_nsec - t1.tv_nsec;
    td->tv_sec  = t2.tv_sec - t1.tv_sec;
    if (td->tv_sec > 0 && td->tv_nsec < 0)
    {
        td->tv_nsec += NS_PER_SECOND;
        td->tv_sec--;
    }
    else if (td->tv_sec < 0 && td->tv_nsec > 0)
    {
        td->tv_nsec -= NS_PER_SECOND;
        td->tv_sec++;
    }
}

int main(int argc, char* argv[])
{
    printf("Hello World\n\n");

    // Windows Specific Init
    #if defined(_WIN32)
    WSADATA d;
    if(WSAStartup(MAKEWORD(2, 2), &d)){

        printf("Failed to initialize WSA\n");
        return -1;
    }
    #endif
    FILE *fptr;

    // ----------------- Begin Main Program Space -------------------------
    struct timespec time_begin, time_end, time_delta;

    clock_gettime(CLOCK_REALTIME, &time_begin);

    bool udp_only = false;
    bool protected_mode = false;
    bool local = false;
    unsigned int old_time = 0;

    // Check for running in local host
    char hostname[16];
    char port[6] = "14141";
    
    for (int i = 0; i < argc; i++){
        if (strcmp(argv[i], "--protected") == 0){
            protected_mode = true;
        }
        else if(strcmp(argv[i], "--udp") == 0){
            udp_only = true;
        }
        else if(strcmp(argv[i], "--local") == 0){
            local = true;
        }
    }

    if(local){
        strcpy(hostname, "127.0.0.1");
    }
    else{
        get_ip_address(hostname);
    }

    fptr = fopen("./public/server_ip.txt", "w");
    fprintf(fptr, hostname);
    fclose(fptr);

    char* whitelist[MAX_LINE_LENGHT];
    int whitelist_size = 0;

    // Load Whitelist
    if(protected_mode){
        FILE* fp = fopen("whitelist.csv", "r");
        if (fp == NULL){
            printf("Error opening whitelist file.\n");
        }
        
        char line[MAX_LINE_LENGHT];

        int idx = 0;
        while(fgets(line, sizeof(line), fp)){
            line[strcspn(line, "\n")] = '\0';
            
            char* value = strtok(line, ",");

            while(value){
                whitelist[idx] = value;
                idx++;
                value = strtok(NULL, ",");
            }
        }
        whitelist_size = idx;

        #ifdef TESTING_MODE
            printf("Whitelist: ");
            for (int i = 0; i < whitelist_size; i++){
                if(i == whitelist_size - 1){
                    printf("%s\n\n", whitelist[i]);
                }
                else{
                    printf("%s, " , whitelist[i]);
                }
            }
        #endif
    }

    printf("Launching Server at IP: %s:%s\n", hostname, port);

    SOCKET server;
    SOCKET udp_socket;

    struct sockaddr_in unreal_addr;
    socklen_t unreal_addr_len;
    bool unreal = false;

    // Create server sockets 
    if(udp_only){
        udp_socket = create_udp_socket(hostname, port);
    }
    else{
        server = create_socket(hostname, port);
        udp_socket = create_udp_socket(hostname, port);
    }

    #ifdef TESTING_MODE
        printf("\nBig-endian system: %s\n", big_endian() ? "yes" : "no");
        printf("Protected mode: %s\n", protected_mode ? "ON" : "OFF");
    #endif

    // "Data Base" Data
    struct backend_data_t* backend = init_backend(backend);

    // Client connection Data
    struct client_info_t* clients = NULL;
    
    //////////////////////////////////////////////////////////////////// Server /////////////////////////////////////////////////////////////////////////////////
    while(true){
        fd_set reads;
        reads = wait_on_clients(clients, server, udp_socket);

        // Server Listen Socket got a new message
        if(FD_ISSET(server, &reads)){

            // create a new client
            struct client_info_t* client = get_client(&clients, -1);

            // create client socket
            client->socket = accept(server, (struct sockaddr*) &client->address, &client->address_length);
            if(!ISVALIDSOCKET(client->socket)){
                fprintf(stderr, "accept() failed with error: %d", GETSOCKETERRNO());
            }

            #ifdef VERBOSE_MODE
            if(strcmp(get_client_address(client), hostname)){
                printf("New Connection from %s\n", get_client_address(client));

            }
            #endif

        }

        // Handle UDP
        if(FD_ISSET(udp_socket, &reads)){
            struct client_info_t* udp_clients = NULL;
            struct client_info_t* client = get_client(&udp_clients, -1);

            int received_bytes = recvfrom(udp_socket, client->udp_request, MAX_UDP_REQUEST_SIZE, 0, (struct sockaddr*)&client->udp_addr, &client->address_length);

            if(!big_endian()){
                reverse_bytes(client->udp_request);
                reverse_bytes(client->udp_request + 4);
                reverse_bytes(client->udp_request + 8);   
            }

            unsigned int time = 0;
            unsigned int command = 0;
            char data[4] = {0};

            get_contents(client->udp_request, &time, &command, data);

           #ifdef TESTING_MODE
                printf("\nNew datagram received.\n");

                printf("time: %d, ", time);
                printf("command: %d, ", command);
                float value = 0;
                memcpy(&value, data, 4);
                printf("data float: %f, ", value);
                int valuei = 0;   
                memcpy(&valuei, data, 4);
                printf("data int: %d\n\n", valuei);
            #endif

            //check if it's a GET request
            if (command < 1000){
#ifdef VERBOSE_MODE
                printf("Received a GET request from %s:%d \n", inet_ntoa(client->udp_addr.sin_addr), ntohs(client->udp_addr.sin_port));
#endif
                unsigned char* response_buffer;
                int buffer_size = 0;

                //Send lidar
                if(command == 167 && backend->running_pr_sim >= 0){
                    response_buffer = malloc(sizeof(backend->p_rover[backend->running_pr_sim].lidar) + 8);
                    buffer_size = sizeof(backend->p_rover[backend->running_pr_sim].lidar) + 8;

                    udp_get_pr_lidar(response_buffer + 8, backend);

                    memcpy(response_buffer, &backend->server_up_time, 4);
                    memcpy(response_buffer + 4, &command, 4);

                    if(!big_endian()){
                        reverse_bytes(response_buffer);
                        reverse_bytes(response_buffer + 4);
                    }

                }
                //Handle normal GET request
                else{

                    handle_udp_get_request(command, data, backend);
                    response_buffer = malloc(12);
                    buffer_size = 12;

                    memcpy(response_buffer, &backend->server_up_time, 4);
                    memcpy(response_buffer + 4, &command, 4);
                    memcpy(response_buffer + 8, data, 4);

                    if(!big_endian()){
                        reverse_bytes(response_buffer);
                        reverse_bytes(response_buffer + 4);
                        reverse_bytes(response_buffer + 8);
                    }
                }

                //Send response
                int bytes_sent = sendto(udp_socket, response_buffer, buffer_size, 0, (struct sockaddr*)&client->udp_addr, client->address_length);

#ifdef VERBOSE_MODE
                printf("Sent response to %s:%d\n", inet_ntoa(client->udp_addr.sin_addr), ntohs(client->udp_addr.sin_port));
                printf("Bytes sent: %d\n", bytes_sent);
#endif
                drop_udp_client(&udp_clients, client);
                free(response_buffer);

            }
            //check if it's a POST request
            else if (command < 2000){
                
#ifdef VERBOSE_MODE
                printf("Received a POST request from %s:%d \n", inet_ntoa(client->udp_addr.sin_addr), ntohs(client->udp_addr.sin_port));
#endif          
                handle_udp_post_request(command, data, client->udp_request, backend, received_bytes);

                drop_udp_client(&udp_clients, client);
            }
            //Save address of Unreal
            else if (command == 3000){

                unreal_addr = client->udp_addr;
                unreal_addr_len = client->address_length;
                unreal = true;

                //#ifdef TESTING_MODE
                    printf("Unreal address set to %s:%d\n", inet_ntoa(client->udp_addr.sin_addr), ntohs(client->udp_addr.sin_port));
                //#endif

                drop_udp_client(&udp_clients, client);
            }

            else {
                drop_udp_client(&udp_clients, client);
            }
        }
        
        // Send telemetry values to Unreal if there's an address saved
        if(unreal){
            clock_gettime(CLOCK_REALTIME, &time_end);
            sub_timespec(time_begin, time_end, &time_delta);

            if(time_delta.tv_nsec > 200000000){
                tss_to_unreal(udp_socket, unreal_addr, unreal_addr_len, backend);
                
                clock_gettime(CLOCK_REALTIME, &time_begin);
            }
        }

        // Server-Client Socket got a new message
        struct client_info_t* client = clients;

        //Check if client is on the http whitelist
        if(protected_mode && client){
            bool found = false;

            for (int i = 0; i < whitelist_size; i++){
                if(strcmp(whitelist[i], get_client_address(client)) == 0){
                    found = true;
                }
            }

            if(!found){
                drop_client(&clients, client);
                printf("Client not in whitelist\n");
            }
        }

        if(!udp_only){

            while(client){
                
                struct client_info_t* next_client = client->next;

                // Check if this client has pending request
                if(FD_ISSET(client->socket, &reads)){

                    // Request too big
                    if(MAX_REQUEST_SIZE <= client->received){
                        send_400(client);
                        drop_client(&clients, client);
                        client = next_client;
                        continue;
                    }

                    // read new bytes in
                    int bytes_received = recv(client->socket, client->request + client->received, MAX_REQUEST_SIZE - client->received, 0);

                    if(bytes_received < 1){
                        
                        #ifdef VERBOSE_MODE
                        fprintf(stderr, "Unexpected Disconnect from %s\n", get_client_address(client));
                        #endif
                        drop_client(&clients, client);
                    } else {

                        if(strncmp(client->request, "GET/", 4) == 0){
#ifdef VERBOSE_MODE
                            printf("UDP request\n");
#endif
                        }

                        client->received += bytes_received;
                        client->request[client->received] = 0;

                        // Find marker for the end of the header
                        char* q = strstr(client->request, "\r\n\r\n");
                        if(q){

                            // Received a GET Request
                            if(strncmp(client->request, "GET /", 5) == 0){
                                char* path = client->request + 4;
                                char* end_path = strstr(path, " ");

                                if(!end_path){
                                    send_400(client);
                                    drop_client(&clients, client);
                                } else {
                                    *end_path = 0;
                                    serve_resource(client, path);
                                    #ifdef VERBOSE_MODE
                                    printf("serve_resource %s %s\n", get_client_address(client), path);
                                    #endif
                                    drop_client(&clients, client);
                                }
                            } // Received a POST Request 
                            else if(strncmp(client->request, "POST /", 6) == 0) {

                                // Get the size of the post request
                                if(client->message_size == -1){
                                    char* request_content_size_ptr = strstr(client->request, "Content-Length: ");
                                    if(request_content_size_ptr){
                                        request_content_size_ptr += strlen("Content-Length: ");
                                        client->message_size = atoi(request_content_size_ptr) + (q - client->request) + 4; // The size of the content plus size of the header
                                    } else {
                                        // There is no content size
                                        send_400(client);
                                        drop_client(&clients, client);
                                    }
                                } 
                                
                                if(client->received == client->message_size){

                                    // all bytes loaded
                                    char* request_content = strstr(client->request, "\r\n\r\n");
                                    request_content += 4; // Jump to the beginning of the context
                                
                                    #ifdef VERBOSE_MODE
                                    printf("Received Post Content: \n%s\n", request_content);
                                    #endif

                                    if(!request_content){
                                        send_400(client);
                                        drop_client(&clients, client);
                                    } else {
                                        if(update_resource(request_content, backend)){
                                            send_304(client);
                                        } else {
                                            send_400(client);
                                        }
                                        drop_client(&clients, client);
                                    }
                                }

                            } // Received some other request
                            else {
                                send_400(client);
                                drop_client(&clients, client);
                            }
                        }
                    }
                }

                client = next_client;
            }
        }

        if(!continue_server()) {
            break;
        }

        // Simulate the variables
        simulate_backend(backend);

    }


    // Clean up
    printf("Clean up Database...\n");
    cleanup_backend(backend);

    printf("Closing Sockets...\n");
    CLOSESOCKET(server);
#if defined(_WIN32)
#else
    close(udp_socket);
#endif

    printf("Cleaned up server listen sockets\n");
    int leftover_clients = 0;
    struct client_info_t* client = clients;
    while(client){
        drop_client(&clients, client);
        leftover_clients++;
        client = client->next;
    }
    printf("Cleaned up %d client sockets\n", leftover_clients);

    // ------------------ End Main Program Space -------------------------

    // Windows Specific Cleanup
    #if defined(_WIN32)
    WSACleanup();
    #endif

    printf("\nGoodbye World\n");
    return 0;
}


///////////////////////////////////////////////////////////////////////////////////
//                             Helper Functions
///////////////////////////////////////////////////////////////////////////////////

bool continue_server(){

    // if the user presses the ENTER key, the program will end gracefully
    struct timeval select_wait;
    select_wait.tv_sec = 0;
    select_wait.tv_usec = 0; 
    fd_set console_fd;
    FD_ZERO(&console_fd);
    FD_SET(0, &console_fd);
    int read_files = select(1, &console_fd, 0,0, &select_wait);
    if(read_files > 0) {
        return false;
    } else {
        return true;
    }
}

void get_contents(char* buffer, unsigned int* time, unsigned int* command, unsigned char* data){
    memcpy(time, buffer, 4);
    memcpy(command, buffer + 4, 4);
    memcpy(data, buffer + 8, 4);
}

void tss_to_unreal(int socket, struct sockaddr_in address, socklen_t len, struct backend_data_t* backend){
    if (backend->running_pr_sim < 0) {
        return;
    }
    int brakes = backend->p_rover[backend->running_pr_sim].brakes;
    int lights_on = backend->p_rover[backend->running_pr_sim].lights_on;
    float steering = backend->p_rover[backend->running_pr_sim].steering;
    float throttle = backend->p_rover[backend->running_pr_sim].throttle;
    int switch_dest = backend->p_rover[backend->running_pr_sim].switch_dest;

    unsigned int time = backend->server_up_time;
    unsigned int command = TSS_TO_UNREAL_BRAKES_COMMAND;
    unsigned int command2 = TSS_TO_UNREAL_LIGHTS_COMMAND;
    unsigned int command3 = TSS_TO_UNREAL_STEERING_COMMAND;
    unsigned int command4 = TSS_TO_UNREAL_THROTTLE_COMMAND;
    unsigned int command5 = TSS_TO_UNREAL_SWITCH_DEST_COMMAND;

    unsigned char buffer[12];
    unsigned char buffer2[12];
    unsigned char buffer3[12];
    unsigned char buffer4[12];
    unsigned char buffer5[12];
    
    if(!big_endian()){
        reverse_bytes((unsigned char*)&time);
        reverse_bytes((unsigned char*)&command);
        reverse_bytes((unsigned char*)&command2);
        reverse_bytes((unsigned char*)&command3);
        reverse_bytes((unsigned char*)&command4);
        reverse_bytes((unsigned char*)&command5);
        reverse_bytes((unsigned char*)&steering);
        reverse_bytes((unsigned char*)&throttle);
    }
    
    memcpy(buffer, &time, 4);
    memcpy(buffer + 4, &command, 4);
    memcpy(buffer + 8, &brakes, 4);

    sendto(socket, buffer, sizeof(buffer), 0, (struct sockaddr*)&address, len);

    memcpy(buffer2, &time, 4);
    memcpy(buffer2 + 4, &command2, 4);
    memcpy(buffer2 + 8, &lights_on, 4);

    sendto(socket, buffer2, sizeof(buffer2), 0, (struct sockaddr*)&address, len);

    memcpy(buffer3, &time, 4);
    memcpy(buffer3 + 4, &command3, 4);
    memcpy(buffer3 + 8, &steering, 4);

    sendto(socket, buffer3, sizeof(buffer3), 0, (struct sockaddr*)&address, len);

    memcpy(buffer4, &time, 4);
    memcpy(buffer4 + 4, &command4, 4);
    memcpy(buffer4 + 8, &throttle, 4);

    sendto(socket, buffer4, sizeof(buffer4), 0, (struct sockaddr*)&address, len);

    memcpy(buffer5, &time, 4);
    memcpy(buffer5 + 4, &command5, 4);
    memcpy(buffer5 + 8, &switch_dest, 4);

    sendto(socket, buffer5, sizeof(buffer5), 0, (struct sockaddr*)&address, len);
}