
///////////////////////////////////////////////////////////////////////////////////
//                              Platform Dependent Headers
///////////////////////////////////////////////////////////////////////////////////

#if defined(_WIN32)
    // Windows Networking
    #ifndef _WIN32_WINNT
        #define _WIN32_WINNT 0x0600
    #endif
    #include <winsock2.h>
    #include <iphlpapi.h>
    #include <WS2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    #pragma comment(lib, "iphlpapi.lib")
    typedef int socklen_t;

    #define ISVALIDSOCKET(s) ((s) != INVALID_SOCKET)
    #define CLOSESOCKET(s) closesocket(s)
    #define GETSOCKETERRNO() (WSAGetLastError())
#else
    // Unix Networking
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <ifaddrs.h>
    #include <netdb.h>
    #include <unistd.h>
    #include <errno.h>

    #define SOCKET int
    #define ISVALIDSOCKET(s) ((s) >= 0)
    #define CLOSESOCKET(s) close(s)
    #define GETSOCKETERRNO() (errno)
#endif

#include <time.h>
#include <math.h>
///////////////////////////////////////////////////////////////////////////////////
//                                  Structs
///////////////////////////////////////////////////////////////////////////////////

#define MAX_REQUEST_SIZE 2047
#define MAX_UDP_REQUEST_SIZE 8008

struct client_info_t {
    socklen_t address_length;
    struct sockaddr_storage address;
    struct sockaddr_in udp_addr;
    SOCKET socket;
    char request[MAX_REQUEST_SIZE+1];
    char udp_request[MAX_UDP_REQUEST_SIZE];
    int received;
    int message_size;
    struct timespec* ts;
    struct client_info_t* next;
};

///////////////////////////////////////////////////////////////////////////////////
//                                 Functions
///////////////////////////////////////////////////////////////////////////////////

void get_ip_address(char* hostname_out);

const char* get_content_type(const char* path);

SOCKET create_socket(char* hostname, char* port);

SOCKET create_udp_socket(char* hostname, char* port);

struct client_info_t* get_client(struct client_info_t** clients, SOCKET socket);

void drop_udp_client(struct client_info_t** clients, struct client_info_t* client);

void drop_client(struct client_info_t** clients, struct client_info_t* client);

const char* get_client_address(struct client_info_t* client);

fd_set wait_on_clients(struct client_info_t* clients, SOCKET server, SOCKET udp_socket);

void send_400(struct client_info_t* client);

void send_404(struct client_info_t* client);

void send_201(struct client_info_t* client);

void send_304(struct client_info_t* client);

void reset_client_request_buffer(struct client_info_t* client);

void serve_resource(struct client_info_t* client, const char* path);


struct client_info_t* client_constructor(struct client_info_t* client);

int compare_clients(struct client_info_t* client1, struct client_info_t* client2);

int get_client_index(struct client_info_t* client);

int add_client(struct client_info_t* client);

struct timespec* update_client_time(struct client_info_t* client);

double time_difference(struct timespec *time1, struct timespec *time2);

int rate_limit_required(struct client_info_t* client);

struct client_info_t* get_recent_client(int index);