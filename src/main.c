#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#ifdef __clang__
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wreserved-macro-identifier"
    #pragma clang diagnostic ignored "-Wreserved-identifier"
    #pragma clang diagnostic ignored "-Wdocumentation-unknown-command"
#endif
#include <SDL2/SDL.h>
#pragma GCC diagnostic pop
#ifdef __clang__
    #pragma clang diagnostic pop
#endif
#ifdef __linux__
    #include <fcntl.h>
    #include <linux/input-event-codes.h>
    #include <linux/input.h>
#endif
#include <arpa/inet.h>
#include <ncurses.h>
#include <netinet/in.h>
#include <p101_fsm/fsm.h>
#include <p101_posix/p101_unistd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define LINES 40
#define COLS 40
#define ONE 1
#define ZERO 0
#define TIMER_DELAY 5
#define UNKNOWN_OPTION_MESSAGE_LEN 24
#define UP 1
#define RIGHT 2
#define DOWN 3
#define LEFT 4

#define PORT 12345

typedef struct
{
    char               *dest_ip;
    WINDOW             *win;
    bool                invalid_move;
    int                 local_y;
    int                 local_x;
    int                 remote_y;
    int                 remote_x;
    SDL_GameController *controller;
    int                 local_udp_socket;
    uint16_t            received_value;
    uint16_t            send_value;
} program_data;

enum application_states
{
    SETUP = P101_FSM_USER_START,
    WAIT_FOR_INPUT,
    PROCESS_KEYBOARD_INPUT,
    PROCESS_CONTROLLER_INPUT,
    PROCESS_TIMER_MOVE,
    PROCESS_NETWORK_INPUT,
    MOVE_LOCAL,
    MOVE_REMOTE,
    ERROR
};

static void             parse_arguments(const struct p101_env *env, int argc, char *argv[], bool *bad, bool *will, bool *did, program_data *data);
_Noreturn static void   usage(const char *program_name, int exit_code, const char *message);
static p101_fsm_state_t setup(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t wait_for_input(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t process_keyboard_input(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t process_controller_input(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t process_timer_move(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t process_network_input(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t move_local(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t move_remote(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t state_error(const struct p101_env *env, struct p101_error *err, void *arg);
static void             setup_signal_handler(void);
static void             sigint_handler(int signum);
static void             send_udp_packet(const char *dest_ip, uint16_t send_value);
void                    setup_network_address(struct sockaddr_storage *addr, socklen_t *addr_len, const char *address, in_port_t port, int *err);

static volatile sig_atomic_t exit_flag = 0;    // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char *argv[])
{
    struct p101_error    *error;
    struct p101_env      *env;
    bool                  bad;
    bool                  will;
    bool                  did;
    struct p101_error    *fsm_error;
    struct p101_env      *fsm_env;
    struct p101_fsm_info *fsm;
    program_data          data;

    error = p101_error_create(false);
    env   = p101_env_create(error, true, NULL);
    bad   = false;
    will  = false;
    did   = false;
    setup_signal_handler();
    parse_arguments(env, argc, argv, &bad, &will, &did, &data);
    printf("Dest IP address: %s\n", data.dest_ip);
    fsm_error = p101_error_create(false);
    fsm_env   = p101_env_create(error, true, NULL);
    fsm       = p101_fsm_info_create(env, error, "test-fsm", fsm_env, fsm_error, NULL);

    if(p101_error_has_error(error))
    {
        fprintf(stderr, "Error creating FSM: %s\n", p101_error_get_message(error));
    }
    else
    {
        static struct p101_fsm_transition transitions[] = {
            {P101_FSM_INIT,            SETUP,                    setup                   },
            {SETUP,                    WAIT_FOR_INPUT,           wait_for_input          },
            {WAIT_FOR_INPUT,           PROCESS_KEYBOARD_INPUT,   process_keyboard_input  },
            {WAIT_FOR_INPUT,           PROCESS_CONTROLLER_INPUT, process_controller_input},
            {WAIT_FOR_INPUT,           PROCESS_TIMER_MOVE,       process_timer_move      },
            {WAIT_FOR_INPUT,           PROCESS_NETWORK_INPUT,    process_network_input   },
            {PROCESS_KEYBOARD_INPUT,   MOVE_LOCAL,               move_local              },
            {PROCESS_CONTROLLER_INPUT, MOVE_LOCAL,               move_local              },
            {PROCESS_TIMER_MOVE,       MOVE_LOCAL,               move_local              },
            {PROCESS_NETWORK_INPUT,    MOVE_REMOTE,              move_remote             },
            {PROCESS_KEYBOARD_INPUT,   WAIT_FOR_INPUT,           wait_for_input          }, //  if validation fails
            {PROCESS_CONTROLLER_INPUT, WAIT_FOR_INPUT,           wait_for_input          }, //  if validation fails
            {PROCESS_NETWORK_INPUT,    WAIT_FOR_INPUT,           wait_for_input          }, //  if validation fails
            {MOVE_LOCAL,               WAIT_FOR_INPUT,           wait_for_input          },
            {MOVE_REMOTE,              WAIT_FOR_INPUT,           wait_for_input          },
            {SETUP,                    ERROR,                    state_error             },
            {WAIT_FOR_INPUT,           ERROR,                    state_error             },
            {PROCESS_KEYBOARD_INPUT,   ERROR,                    state_error             },
            {PROCESS_CONTROLLER_INPUT, ERROR,                    state_error             },
            {PROCESS_NETWORK_INPUT,    ERROR,                    state_error             },
            {MOVE_LOCAL,               ERROR,                    state_error             },
            {MOVE_REMOTE,              ERROR,                    state_error             },
            {WAIT_FOR_INPUT,           P101_FSM_EXIT,            NULL                    }, //  if we ask to exit (cntrl c?)
            {ERROR,                    P101_FSM_EXIT,            NULL                    }
        };
        p101_fsm_state_t from_state;
        p101_fsm_state_t to_state;
        p101_fsm_run(fsm, &from_state, &to_state, &data, transitions, sizeof(transitions));
        p101_fsm_info_destroy(env, &fsm);
    }

    // Restore the cursor before exiting
    curs_set(1);
    // deallocates memory and ends ncurses
    endwin();

    free(fsm_env);
    free(env);
    p101_error_reset(error);
    free(error);

    SDL_Quit();

    return EXIT_SUCCESS;
}

static void parse_arguments(const struct p101_env *env, int argc, char *argv[], bool *bad, bool *will, bool *did, program_data *data)
{
    int opt;
    data->dest_ip = NULL;
    opterr        = 0;
    while((opt = p101_getopt(env, argc, argv, "hbdwa:")) != -1)
    {
        switch(opt)
        {
            case 'a':
            {
                data->dest_ip = optarg;
                break;
            }
            case 'b':
            {
                *bad = true;
                break;
            }
            case 'd':
            {
                *did = true;
                break;
            }
            case 'w':
            {
                *will = true;
                break;
            }
            case 'h':
            {
                usage(argv[0], EXIT_SUCCESS, NULL);
            }
            case '?':
            {
                if(optopt == 'c')
                {
                    usage(argv[0], EXIT_FAILURE, "Option '-c' requires a value.");
                }
                else
                {
                    char message[UNKNOWN_OPTION_MESSAGE_LEN];

                    snprintf(message, sizeof(message), "Unknown option '-%c'.", optopt);
                    usage(argv[0], EXIT_FAILURE, message);
                }
            }
            default:
            {
                usage(argv[0], EXIT_FAILURE, NULL);
            }
        }
    }
    if(data->dest_ip == NULL)
    {
        usage(argv[0], EXIT_FAILURE, "Destination IP is required.");
    }

    if(optind < argc)
    {
        usage(argv[0], EXIT_FAILURE, "Too many arguments.");
    }
}

_Noreturn static void usage(const char *program_name, int exit_code, const char *message)
{
    if(message)
    {
        fprintf(stderr, "%s\n", message);
    }

    fprintf(stderr, "Usage: %s [-h] [-b] [-d] [-w]\n", program_name);
    fputs("Options:\n", stderr);
    fputs("  -h   Display this help message\n", stderr);
    fputs("  -b   Display 'bad' transitions\n", stderr);
    fputs("  -w   Display 'will' transitions 'd'\n", stderr);
    fputs("  -d   Display 'did' transitions\n", stderr);
    exit(exit_code);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// This is our setup function
static p101_fsm_state_t setup(const struct p101_env *env, struct p101_error *err, void *arg)
{
    int                lines   = LINES;
    int                cols    = COLS;
    int                local_y = 1;
    int                local_x = 1;
    program_data      *data    = ((program_data *)arg);
    struct sockaddr_in server_addr;

    // set up socket for receiving packets
    data->local_udp_socket = socket(AF_INET, SOCK_DGRAM, 0);    // NOLINT(android-cloexec-socket)
    if(data->local_udp_socket < 0)
    {
        perror("socket");
        return P101_FSM_EXIT;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family      = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port        = htons(PORT);    // Port to listen on

    if(bind(data->local_udp_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("bind");
        close(data->local_udp_socket);
        return P101_FSM_EXIT;
    }

    P101_TRACE(env);
    printf("setting up with dest ip %s!\n", data->dest_ip);

    // Initialize SDL for controller input
    if(SDL_Init(SDL_INIT_GAMECONTROLLER) != 0)
    {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        return ERROR;
    }

    if(SDL_NumJoysticks() > 0)
    {
        data->controller = SDL_GameControllerOpen(0);
        if(!data->controller)
        {
            fprintf(stderr, "Could not open game controller: %s\n", SDL_GetError());
            SDL_Quit();
            return ERROR;
        }
    }
    else
    {
        printf("No game controllers connected.\n");
        SDL_Quit();
    }

    // Initialize ncurses
    // init screen and sets up screen
    initscr();
    // disables buffers on lines
    raw();
    // allows us to get arrow keys
    keypad(stdscr, TRUE);
    // suppresses char echoing
    noecho();
    // Hide the cursor
    curs_set(0);

    // creates the window
    data->win      = newwin(lines, cols, local_y, local_x);
    data->local_x  = ONE;
    data->local_y  = ONE;
    data->remote_x = COLS - 1;
    data->remote_y = LINES - 1;

    // refreshes the screen
    refresh();
    box(data->win, 0, 0);    // borders

    // draw initial dots
    mvwprintw(data->win, data->local_y, data->local_x, "*");
    mvwprintw(data->win, data->local_y, data->local_x, "@");

    wrefresh(data->win);

    return WAIT_FOR_INPUT;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// In this function we will select/poll for keyboard input and/or receiving a UDP packet to our IP address
static p101_fsm_state_t wait_for_input(const struct p101_env *env, struct p101_error *err, void *arg)
{
    program_data *data;
    SDL_Event     event;

    // Setup for monitoring input with timeout
    fd_set             read_fds;
    struct timeval     timeout;
    int                retval;
    struct sockaddr_in client_addr;
    socklen_t          addr_len = sizeof(client_addr);
    int                received_int;

    P101_TRACE(env);
    data = ((program_data *)arg);

    // Handles Invalid Moves
    if(data->invalid_move)
    {
        mvprintw(LINES + 1, 0, "");

        mvprintw(LINES + 1, 0, "INVALID MOVE                              ");    // the line needs to be this long to clear the other text
        wrefresh(data->win);
        data->invalid_move = false;
    }
    else
    {
        mvprintw(LINES + 1, 0, "");

        mvprintw(LINES + 1, 0, "Hit arrow keys or your controller to move.");
        box(data->win, ZERO, ZERO);    // borders
        wrefresh(data->win);
    }
    // timeout
    memset(&read_fds, 0, sizeof(read_fds));
    FD_SET(STDIN_FILENO, &read_fds);
    timeout.tv_sec  = TIMER_DELAY;
    timeout.tv_usec = 0;

    // Triggers timer move unless something is pressed
    retval = select(STDIN_FILENO + 1, &read_fds, NULL, NULL, &timeout);

    // Check for SDL controller events
    while(SDL_PollEvent(&event))
    {
        if(event.type == SDL_CONTROLLERBUTTONDOWN)
        {
            // If directional button is pressed, transition to controller input state
            switch(event.cbutton.button)
            {
                case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
                case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
                case SDL_CONTROLLER_BUTTON_DPAD_UP:
                case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
                    return PROCESS_CONTROLLER_INPUT;
                default:
                    break;
            }
        }
    }

    if(retval == -1)
    {
        perror("select");
        return ERROR;
    }

    if(retval == 0)
    {
        // Timeout occurred, trigger timer-based move
        return PROCESS_TIMER_MOVE;
    }

    if(FD_ISSET(STDIN_FILENO, &read_fds))
    {
        // Input detected, handle keyboard input
        // TODO: process actual keyboard input instead of just advancing to PROCESS_KEYBOARD_INPUT state
        // options for this: 1) use the arrow keys (tricky sequence) or 2) accept numbers
        return PROCESS_KEYBOARD_INPUT;
    }

    if(FD_ISSET(data->local_udp_socket, &read_fds))
    {
        // UDP packet received
        ssize_t bytes_received = recvfrom(data->local_udp_socket, &received_int, sizeof(received_int), 0, (struct sockaddr *)&client_addr, &addr_len);
        if(bytes_received < 0)
        {
            perror("recvfrom");
            return ERROR;
        }

        // Update program data with received integer
        data->received_value = ntohs(received_int);    // Convert from network byte order
        return PROCESS_NETWORK_INPUT;
    }

    return WAIT_FOR_INPUT;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// In this function we will process keyboard input and validate the move.
// If the move is invalid, we will return WAIT_FOR_INPUT
// If the move is valie, we will return MOVE_LOCAL
static p101_fsm_state_t process_keyboard_input(const struct p101_env *env, struct p101_error *err, void *arg)
{
    program_data *data;
    P101_TRACE(env);
    data = ((program_data *)arg);
    box(data->win, ZERO, ZERO);    // borders
    wrefresh(data->win);
    // gets input from the keyboard into the program data somehow
    if(exit_flag == 0)
    {
        // I think eventually we will save the char from the input of when wait_for_input
        // we will need to add a char to our data struct
        int ch = getch();
        switch(ch)
        {
            case KEY_LEFT:
                data->local_x = data->local_x - 1;
                if(data->local_x < 1)
                {
                    data->local_x      = data->local_x + 1;
                    data->invalid_move = true;
                    return WAIT_FOR_INPUT;
                }
                data->send_value = htons(LEFT);    // serialized integer
                return MOVE_LOCAL;
            case KEY_RIGHT:
                data->local_x = data->local_x + 1;
                if(data->local_x >= COLS - 1)
                {
                    data->local_x      = data->local_x - 1;
                    data->invalid_move = true;
                    return WAIT_FOR_INPUT;
                }
                data->send_value = htons(RIGHT);    // serialized integer
                return MOVE_LOCAL;
            case KEY_UP:
                data->local_y = data->local_y - 1;
                if(data->local_y < 1)
                {
                    data->local_y      = data->local_y + 1;
                    data->invalid_move = true;
                    return WAIT_FOR_INPUT;
                }
                data->send_value = htons(UP);    // serialized integer
                return MOVE_LOCAL;
            case KEY_DOWN:
                data->local_y = data->local_y + 1;
                if(data->local_y >= COLS - 1)
                {
                    data->local_y      = data->local_y - 1;
                    data->invalid_move = true;
                    return WAIT_FOR_INPUT;
                }
                data->send_value = htons(DOWN);    // serialized integer
                return MOVE_LOCAL;
            default:
                break;
        }
    }

    // deallocates memory and ends ncurses
    endwin();
    close(data->local_udp_socket);
    printf("exiting...\n");
    return P101_FSM_EXIT;
}

#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// Notes for PS4: Left = 13, Right = 14, Up = 11, Down = 12
static p101_fsm_state_t process_controller_input(const struct p101_env *env, struct p101_error *err, void *arg)
{
    SDL_Event event;

    program_data *data;
    P101_TRACE(env);
    data = ((program_data *)arg);
    box(data->win, ZERO, ZERO);    // borders
    wrefresh(data->win);

    while(SDL_PollEvent(&event))
    {
        if(event.type == SDL_CONTROLLERBUTTONDOWN)
        {
            switch(event.cbutton.button)
            {
                case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
                    data->local_x--;
                    if(data->local_x < 1)
                    {
                        data->local_x++;
                        data->invalid_move = true;
                        return WAIT_FOR_INPUT;
                    }
                    return MOVE_LOCAL;
                case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
                    data->local_x++;
                    if(data->local_x >= COLS - 1)
                    {
                        data->local_x--;
                        data->invalid_move = true;
                        return WAIT_FOR_INPUT;
                    }
                    return MOVE_LOCAL;
                case SDL_CONTROLLER_BUTTON_DPAD_UP:
                    data->local_y--;
                    if(data->local_y < 1)
                    {
                        data->local_y++;
                        data->invalid_move = true;
                        return WAIT_FOR_INPUT;
                    }
                    return MOVE_LOCAL;
                case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
                    data->local_y++;
                    if(data->local_y >= LINES - 1)
                    {
                        data->local_y--;
                        data->invalid_move = true;
                        return WAIT_FOR_INPUT;
                    }
                    return MOVE_LOCAL;
                default:
                    printf("Unhandled button: %d\n", event.cbutton.button);
                    break;
            }
        }
    }

    return WAIT_FOR_INPUT;
}

#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

static p101_fsm_state_t process_timer_move(const struct p101_env *env, struct p101_error *err, void *arg)
{
    uint32_t      direction;
    program_data *data = ((program_data *)arg);
    P101_TRACE(env);
    box(data->win, ZERO, ZERO);
    wrefresh(data->win);

    // Generate random direction: 0 = LEFT, 1 = RIGHT, 2 = UP, 3 = DOWN
    direction = arc4random_uniform(4);
    // Adjust position based on direction
    switch(direction)
    {
        case 0:    // LEFT
            data->local_x--;
            if(data->local_x < 1)
            {
                data->local_x++;
                data->invalid_move = true;
                return WAIT_FOR_INPUT;
            }
            break;
        case 1:    // RIGHT
            data->local_x++;
            if(data->local_x >= COLS - 1)
            {
                data->local_x--;
                data->invalid_move = true;
                return WAIT_FOR_INPUT;
            }
            break;
        case 2:    // UP
            data->local_y--;
            if(data->local_y < 1)
            {
                data->local_y++;
                data->invalid_move = true;
                return WAIT_FOR_INPUT;
            }
            break;
        case 3:    // DOWN
            data->local_y++;
            if(data->local_y >= LINES - 1)
            {
                data->local_y--;
                data->invalid_move = true;
                return WAIT_FOR_INPUT;
            }
            break;
        default:
            break;
    }

    // Trigger MOVE_LOCAL for valid moves
    return MOVE_LOCAL;
}

#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// In this function we will process network input and validate the move.
// If the move is invalid, we will return WAIT_FOR_INPUT
// If the move is valid, we will return MOVE_REMOTE
static p101_fsm_state_t process_network_input(const struct p101_env *env, struct p101_error *err, void *arg)
{
    program_data *data;
    P101_TRACE(env);
    data = ((program_data *)arg);
    printf("process_network_input called with dest_ip %s\n", data->dest_ip);

    switch(data->received_value)
    {
        case LEFT:
            data->remote_x = data->remote_x - 1;
            if(data->remote_x < 1)
            {
                data->remote_x = data->remote_x + 1;
                return WAIT_FOR_INPUT;
            }
            break;
        case RIGHT:
            data->remote_x = data->remote_x + 1;
            if(data->remote_x >= COLS - 1)
            {
                data->remote_x = data->remote_x - 1;
                return WAIT_FOR_INPUT;
            }
            break;
        case UP:
            data->remote_y = data->remote_y - 1;
            if(data->remote_y < 1)
            {
                data->remote_y = data->remote_y + 1;
                return WAIT_FOR_INPUT;
            }
            break;
        case DOWN:
            data->remote_y = data->remote_y + 1;
            if(data->remote_y >= COLS - 1)
            {
                data->remote_y = data->remote_y - 1;
                return WAIT_FOR_INPUT;
            }
            break;
        default:
            break;
    }
    return MOVE_REMOTE;
}

#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// This function will do the move on the local machine
static p101_fsm_state_t move_local(const struct p101_env *env, struct p101_error *err, void *arg)
{
    program_data *data;
    P101_TRACE(env);
    data = ((program_data *)arg);
    wclear(data->win);
    mvwprintw(data->win, data->local_y, data->local_x, "*");
    send_udp_packet(data->dest_ip, data->send_value);
    return WAIT_FOR_INPUT;
}

#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// This function will do the move on the remote machine
static p101_fsm_state_t move_remote(const struct p101_env *env, struct p101_error *err, void *arg)
{
    const program_data *data;
    P101_TRACE(env);
    data = ((program_data *)arg);
    wclear(data->win);
    mvwprintw(data->win, data->remote_y, data->remote_x, "@");
    return WAIT_FOR_INPUT;
}

#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

static p101_fsm_state_t state_error(const struct p101_env *env, struct p101_error *err, void *arg)
{
    P101_TRACE(env);

    return P101_FSM_EXIT;
}

#pragma GCC diagnostic pop

static void setup_signal_handler(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
#if defined(__clang__)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wdisabled-macro-expansion"
#endif
    sa.sa_handler = sigint_handler;
#if defined(__clang__)
    #pragma clang diagnostic pop
#endif
    sigaction(SIGINT, &sa, NULL);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

static void sigint_handler(int signum)
{
    exit_flag = 1;
    printf("SIGINT received. Exiting...\n");
}

#pragma GCC diagnostic pop

void send_udp_packet(const char *dest_ip, uint16_t send_value)
{
    int                     sockfd;
    struct sockaddr_storage dest_addr;
    socklen_t               addr_len;
    int                     ret_val = 0;

    setup_network_address(&dest_addr, &addr_len, dest_ip, PORT, &ret_val);
    if(ret_val != 0)
    {
        perror("Setup network address failed");
        exit_flag = SIGINT;
    }

    // Create a UDP socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);    // NOLINT(android-cloexec-socket)
    if(sockfd < 0)
    {
        perror("Socket creation failed");
        exit_flag = SIGINT;
    }
    else
    {
        // Send the integer
        if(sendto(sockfd, &send_value, sizeof(send_value), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0)
        {
            perror("Sendto failed");
            exit_flag = SIGINT;
        }

        printf("Sent integer: %d\n", send_value);
        // Close the socket
        close(sockfd);
    }
}

void setup_network_address(struct sockaddr_storage *addr, socklen_t *addr_len, const char *address, in_port_t port, int *err)
{
    in_port_t net_port;
    *addr_len = 0;
    net_port  = htons(port);
    memset(addr, 0, sizeof(*addr));

    if(inet_pton(AF_INET, address, &(((struct sockaddr_in *)addr)->sin_addr)) == 1)
    {
        struct sockaddr_in *ipv4_addr;

        ipv4_addr           = (struct sockaddr_in *)addr;
        addr->ss_family     = AF_INET;
        ipv4_addr->sin_port = net_port;
        *addr_len           = sizeof(struct sockaddr_in);
    }
    else if(inet_pton(AF_INET6, address, &(((struct sockaddr_in6 *)addr)->sin6_addr)) == 1)
    {
        struct sockaddr_in6 *ipv6_addr;

        ipv6_addr            = (struct sockaddr_in6 *)addr;
        addr->ss_family      = AF_INET6;
        ipv6_addr->sin6_port = net_port;
        *addr_len            = sizeof(struct sockaddr_in6);
    }
    else
    {
        fprintf(stderr, "%s is not an IPv4 or an IPv6 address\n", address);
        *err = errno;
    }
}
