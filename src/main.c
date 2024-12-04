#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-default"
#ifdef __clang__
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wreserved-macro-identifier"
    #pragma clang diagnostic ignored "-Wreserved-identifier"
    #pragma clang diagnostic ignored "-Wdocumentation-unknown-command"
#endif
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
    #include <SDL2/SDL.h>
#endif
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
#define ERR_NONE 0
#define ERR_NO_DIGITS 1
#define ERR_OUT_OF_RANGE 2
#define ERR_INVALID_CHARS 3

typedef struct
{
    char   *remote_ip;
    char   *local_ip;
    WINDOW *win;
    bool    invalid_move;
    int     local_y;
    int     local_x;
    int     remote_y;
    int     remote_x;
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
    SDL_GameController *controller;
#endif
    int       local_udp_socket;
    uint16_t  received_value;
    uint16_t  send_value;
    int       direction;
    in_port_t local_port;
    in_port_t remote_port;
} program_data;

enum application_states
{
    SETUP = P101_FSM_USER_START,
    WAIT_FOR_INPUT,
    PROCESS_KEYBOARD_INPUT,
    PROCESS_CONTROLLER_INPUT,
    PROCESS_TIMER_MOVE,
    MOVE_LOCAL,
    MOVE_REMOTE,
    ERROR
};

static void             parse_arguments(const struct p101_env *env, int argc, char *argv[], bool *bad, bool *will, bool *did, program_data *data, int *err);
_Noreturn static void   usage(const char *program_name, int exit_code, const char *message);
static p101_fsm_state_t setup(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t wait_for_input(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t process_keyboard_input(const struct p101_env *env, struct p101_error *err, void *arg);
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
static p101_fsm_state_t process_controller_input(const struct p101_env *env, struct p101_error *err, void *arg);
#endif
static p101_fsm_state_t process_timer_move(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t move_local(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t move_remote(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t state_error(const struct p101_env *env, struct p101_error *err, void *arg);
static void             setup_signal_handler(void);
static void             sigint_handler(int signum);
static void             send_udp_packet(const char *remote_ip, in_port_t remote_port, uint16_t send_value);
void                    setup_network_address(struct sockaddr_storage *addr, socklen_t *addr_len, const char *address, in_port_t port, int *err);
void                    cleanup(program_data *data);
int                     process_direction(program_data *data);
in_port_t               convert_port(const char *str, int *err);
int                     socket_connect(const program_data *data);

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
    int                   err = 0;

    error = p101_error_create(false);
    env   = p101_env_create(error, true, NULL);
    bad   = false;
    will  = false;
    did   = false;
    setup_signal_handler();
    parse_arguments(env, argc, argv, &bad, &will, &did, &data, &err);
    if(err != 0)
    {
        perror("port");
        return -1;
    }
    printf("Dest IP address: %s\n", data.remote_ip);
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
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
            {WAIT_FOR_INPUT,           PROCESS_CONTROLLER_INPUT, process_controller_input},
#endif
            {WAIT_FOR_INPUT,           PROCESS_TIMER_MOVE,       process_timer_move      },
            {WAIT_FOR_INPUT,           MOVE_REMOTE,              move_remote             },
            {PROCESS_KEYBOARD_INPUT,   MOVE_LOCAL,               move_local              },
            {PROCESS_CONTROLLER_INPUT, MOVE_LOCAL,               move_local              },
            {PROCESS_TIMER_MOVE,       MOVE_LOCAL,               move_local              },
            {PROCESS_KEYBOARD_INPUT,   WAIT_FOR_INPUT,           wait_for_input          }, //  if validation fails
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
            {PROCESS_CONTROLLER_INPUT, WAIT_FOR_INPUT,           wait_for_input          }, //  if validation fails
#endif
            {PROCESS_TIMER_MOVE,       WAIT_FOR_INPUT,           wait_for_input          }, //  if validation fails
            {MOVE_LOCAL,               WAIT_FOR_INPUT,           wait_for_input          },
            {MOVE_REMOTE,              WAIT_FOR_INPUT,           wait_for_input          },
            {SETUP,                    ERROR,                    state_error             },
            {WAIT_FOR_INPUT,           ERROR,                    state_error             },
            {PROCESS_KEYBOARD_INPUT,   ERROR,                    state_error             },
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
            {PROCESS_CONTROLLER_INPUT, ERROR,                    state_error             },
#endif
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

    //    SDL_Quit();

    return EXIT_SUCCESS;
}

static void parse_arguments(const struct p101_env *env, int argc, char *argv[], bool *bad, bool *will, bool *did, program_data *data, int *err)
{
    int opt;
    data->remote_ip   = NULL;
    data->local_ip    = NULL;
    data->local_port  = 0;
    data->remote_port = 0;
    opterr            = 0;
    while((opt = p101_getopt(env, argc, argv, "hbdwr:l:o:p:")) != -1)
    {
        switch(opt)
        {
            case 'r':
            {
                data->remote_ip = optarg;
                break;
            }
            case 'l':
            {
                data->local_ip = optarg;
                break;
            }
            case 'p':
            {
                data->local_port = convert_port(optarg, err);
                break;
            }
            case 'o':
            {
                data->remote_port = convert_port(optarg, err);
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
    if(data->remote_ip == NULL || data->local_ip == NULL || data->remote_port == 0 || data->local_port == 0)
    {
        usage(argv[0], EXIT_FAILURE, "SRC and Destination IPs and ports are required.");
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

    fprintf(stderr, "Usage: %s -l <local ip addr> -r <remote ip addr> -p <local port> -o <remote port>[-h] [-b] [-d] [-w]\n", program_name);
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
    int           lines   = LINES;
    int           cols    = COLS;
    int           local_y = 1;
    int           local_x = 1;
    program_data *data    = ((program_data *)arg);
    int           check   = 0;
    data->direction       = 0;

    //    // Initialize SDL for controller input
    //    if(SDL_Init(SDL_INIT_GAMECONTROLLER) != 0)
    //    {
    //        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
    //        return ERROR;
    //    }
    //
    //    if(SDL_NumJoysticks() > 0)
    //    {
    //        data->controller = SDL_GameControllerOpen(0);
    //        if(!data->controller)
    //        {
    //            fprintf(stderr, "Could not open game controller: %s\n", SDL_GetError());
    //            SDL_Quit();
    //            return ERROR;
    //        }
    //    }
    //    else
    //    {
    //        printf("No game controllers connected.\n");
    //        //        SDL_Quit();
    //    }
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
    data->win     = newwin(lines, cols, local_y, local_x);
    data->local_x = ONE;
    data->local_y = ONE;

    // I think the remote and local dots have to start in the same place or the logic doesn't make sense
    // We can talk about this
    data->remote_x = ONE;
    data->remote_y = ONE;

    // refreshes the screen
    refresh();
    box(data->win, 0, 0);    // borders

    // draw initial dots
    mvwprintw(data->win, data->local_y, data->local_x, "*");
    mvwprintw(data->win, data->remote_y, data->remote_x, "@");

    wrefresh(data->win);

    check = socket_connect(data);
    if(check < 0)
    {
        perror("socket_connect");
        cleanup(data);
        return ERROR;
    }

    data->local_udp_socket = check;

    return WAIT_FOR_INPUT;
}

#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// In this function we will select/poll for keyboard input and/or receiving a UDP packet to our IP address
static p101_fsm_state_t wait_for_input(const struct p101_env *env, struct p101_error *err, void *arg)
{
    program_data *data;
    //    SDL_Event     event;

    // Setup for monitoring input with timeout
    fd_set             read_fds;
    int                nfds = 0;
    struct timeval     timeout;
    int                retval;
    struct sockaddr_in client_addr;
    socklen_t          addr_len = sizeof(client_addr);
    uint16_t           received_int;

    P101_TRACE(env);
    data = ((program_data *)arg);
    printf("waiting for input...\n");

    // Handles Invalid Moves
    if(data->invalid_move)
    {
        mvprintw(LINES + 1, 0, "INVALID MOVE                              ");    // the line needs to be this long to clear the other text
        wrefresh(data->win);
        data->invalid_move = false;
    }
    else
    {
        mvprintw(LINES + 1, 0, " ");
        mvprintw(LINES + 1, 0, "Hit arrow keys or your controller to move.");
        box(data->win, ZERO, ZERO);    // borders
        wrefresh(data->win);
    }
    // timeout
    memset(&read_fds, 0, sizeof(read_fds));

    FD_SET(STDIN_FILENO, &read_fds);
    FD_SET((long unsigned int)data->local_udp_socket, &read_fds);
    // TODO: JUSTIN!! FD_SET the controller file descriptor here so we can poll for it

    timeout.tv_sec  = TIMER_DELAY;
    timeout.tv_usec = 0;

    // TODO: JUSTIN!!
    // we have to set the number of fds being monitored to the highest fd val plus one!
    // this calculation will change if we are also polling for a controller
    nfds = (nfds > data->local_udp_socket ? nfds : data->local_udp_socket) + 1;

    // Triggers timer move unless something is pressed
    retval = select(nfds, &read_fds, NULL, NULL, &timeout);

    // I commented this out just to rule out any weird things happening with memory... feel free to uncomment OF COURSE
    //  Check for SDL controller events
    //     while(SDL_PollEvent(&event))
    //     {
    //         if(event.type == SDL_CONTROLLERBUTTONDOWN)
    //         {
    //             printf("controller button pressed down\n");
    //             // If directional button is pressed, transition to controller input state
    //             switch(event.cbutton.button)
    //             {
    //                 case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
    //                 case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
    //                 case SDL_CONTROLLER_BUTTON_DPAD_UP:
    //                 case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
    //                     return PROCESS_CONTROLLER_INPUT;
    //                 default:
    //                     break;
    //             }
    //         }
    //     }

    if(retval == -1)
    {
        perror("select");
        cleanup(data);
        printf("exiting due to select...\n");
        return ERROR;
    }

    if(retval == 0)
    {
        // Timeout occurred, trigger timer-based move
        printf("moving with timer\n");
        return PROCESS_TIMER_MOVE;
    }

    if(FD_ISSET(STDIN_FILENO, &read_fds))
    {
        // Input detected, handle keyboard input
        char    buffer[LINES];
        ssize_t bytes_read = read(STDIN_FILENO, buffer, sizeof(buffer) - 1);
        if(bytes_read == -1)
        {
            perror("read");
            cleanup(data);
            printf("exiting due to keyboard read...\n");
            return ERROR;
        }
        buffer[bytes_read] = '\0';
        if(buffer[0] == '\x03')
        {
            printf("Ctrl+C detected, exiting gracefully.\n");
            perror("sigint");
            cleanup(data);
            exit_flag = SIGINT;
            return ERROR;
        }

        // A == up -> 1
        if(buffer[2] == 'A')
        {
            data->direction = UP;
        }
        // B == down -> 3
        else if(buffer[2] == 'B')
        {
            data->direction = DOWN;
        }
        // C == right -> 2
        else if(buffer[2] == 'C')
        {
            data->direction = RIGHT;
        }
        // D == left -> 4
        else if(buffer[2] == 'D')
        {
            data->direction = LEFT;
        }
        return PROCESS_KEYBOARD_INPUT;
    }

    if(FD_ISSET((long unsigned int)data->local_udp_socket, &read_fds))
    {
        ssize_t bytes_received;
        // UDP packet received
        bytes_received = recvfrom(data->local_udp_socket, &received_int, sizeof(received_int), 0, (struct sockaddr *)&client_addr, &addr_len);
        if(bytes_received < 0)
        {
            perror("recvfrom");
            cleanup(data);
            printf("exiting due to recvfrom...\n");
            return ERROR;
        }

        // Update program data with received integer
        data->received_value = ntohs(received_int);    // Convert from network byte order
        return MOVE_REMOTE;
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
    program_data *data = ((program_data *)arg);
    P101_TRACE(env);
    box(data->win, ZERO, ZERO);    // borders
    wrefresh(data->win);
    // gets input from the keyboard into the program data somehow
    if(exit_flag == 0)
    {
        int valid_direction = process_direction(data);
        if(valid_direction == -1)
        {
            return WAIT_FOR_INPUT;
        }
        return MOVE_LOCAL;
    }

    cleanup(data);
    printf(" exiting...\n");
    return P101_FSM_EXIT;
}

#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))

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
#endif

#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

static p101_fsm_state_t process_timer_move(const struct p101_env *env, struct p101_error *err, void *arg)
{
    uint32_t      direction;
    program_data *data = ((program_data *)arg);
    int           valid_direction;
    P101_TRACE(env);
    box(data->win, ZERO, ZERO);
    wrefresh(data->win);

    // Generate random direction: 0 = LEFT, 1 = RIGHT, 2 = UP, 3 = DOWN
    direction       = arc4random_uniform(4);
    data->direction = (int)direction + 1;
    // Adjust position based on direction
    valid_direction = process_direction(data);
    if(valid_direction == -1)
    {
        return WAIT_FOR_INPUT;
    }
    // Trigger MOVE_LOCAL for valid moves
    return MOVE_LOCAL;
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
    mvwprintw(data->win, data->remote_y, data->remote_x, "@");
    send_udp_packet(data->remote_ip, data->remote_port, data->send_value);
    return WAIT_FOR_INPUT;
}

#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// This function will do the move on the remote machine
static p101_fsm_state_t move_remote(const struct p101_env *env, struct p101_error *err, void *arg)
{
    program_data *data = ((program_data *)arg);
    P101_TRACE(env);
    wclear(data->win);

    // I don't think we should actually need to keep the move on the board because the move should have been
    // validated on the sending side. But I kept in the checks minimally just in case
    switch(data->received_value)
    {
        case 4:    // LEFT
            data->remote_x = data->remote_x - 1;
            if(data->remote_x < 1)
            {
                data->remote_x = data->remote_x + 1;
            }
            break;
        case 2:    // RIGHT
            data->remote_x = data->remote_x + 1;
            if(data->remote_x >= COLS - 1)
            {
                data->remote_x = data->remote_x - 1;
            }
            break;
        case 1:    // UP
            data->remote_y = data->remote_y - 1;
            if(data->remote_y < 1)
            {
                data->remote_y = data->remote_y + 1;
            }
            break;
        case 3:    // DOWN
            data->remote_y = data->remote_y + 1;
            if(data->remote_y >= COLS - 1)
            {
                data->remote_y = data->remote_y - 1;
            }
            break;
        default:
            printf("unknown keyboard input\n");
            sleep(2);
    }
    mvwprintw(data->win, data->remote_y, data->remote_x, "@");
    mvwprintw(data->win, data->local_y, data->local_x, "*");
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

void send_udp_packet(const char *remote_ip, in_port_t remote_port, uint16_t send_value)
{
    int                     sockfd;
    struct sockaddr_storage dest_addr;
    socklen_t               addr_len;
    int                     ret_val = 0;

    setup_network_address(&dest_addr, &addr_len, remote_ip, remote_port, &ret_val);
    if(ret_val != 0)
    {
        perror("Setup network address failed");
        exit_flag = SIGINT;
    }

    // Create a UDP socket
    sockfd = socket(dest_addr.ss_family, SOCK_DGRAM, 0);    // NOLINT(android-cloexec-socket)
    if(sockfd < 0)
    {
        perror("Socket creation failed");
        exit_flag = SIGINT;
    }
    else
    {
        // Send the integer
        if(sendto(sockfd, &send_value, sizeof(send_value), 0, (struct sockaddr *)&dest_addr, addr_len) < 0)
        {
            perror("Sendto failed");
            exit_flag = SIGINT;
        }
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
        //        char                str[INET_ADDRSTRLEN];

        ipv4_addr           = (struct sockaddr_in *)addr;
        addr->ss_family     = AF_INET;
        ipv4_addr->sin_port = net_port;
        *addr_len           = sizeof(struct sockaddr_in);
        //        printf("IPv4 address: %s\n", inet_ntop(AF_INET, &(ipv4_addr->sin_addr), str, sizeof(str)));
    }
    else if(inet_pton(AF_INET6, address, &(((struct sockaddr_in6 *)addr)->sin6_addr)) == 1)
    {
        struct sockaddr_in6 *ipv6_addr;
        //        char                 str[INET6_ADDRSTRLEN];
        ipv6_addr            = (struct sockaddr_in6 *)addr;
        addr->ss_family      = AF_INET6;
        ipv6_addr->sin6_port = net_port;
        *addr_len            = sizeof(struct sockaddr_in6);
        //        printf("IPv6 address: %s\n", inet_ntop(AF_INET6, &(ipv6_addr->sin6_addr), str, sizeof(str)));
    }
    else
    {
        fprintf(stderr, "%s is not an IPv4 or an IPv6 address\n", address);
        *err = errno;
    }
}

void cleanup(program_data *data)
{
    if(data->win)
    {
        delwin(data->win);
    }
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
    if(data->controller)
    {
        SDL_GameControllerClose(data->controller);
    }
#endif
    if(data->local_udp_socket >= 0)
    {
        close(data->local_udp_socket);
        data->local_udp_socket = -1;
    }
}

int process_direction(program_data *data)
{
    switch(data->direction)
    {
        case 4:    // LEFT
            data->local_x = data->local_x - 1;
            if(data->local_x < 1)
            {
                data->local_x      = data->local_x + 1;
                data->invalid_move = true;
                return -1;
            }
            data->send_value = htons(LEFT);    // serialized integer
            break;
        case 2:    // RIGHT
            data->local_x = data->local_x + 1;
            if(data->local_x >= COLS - 1)
            {
                data->local_x      = data->local_x - 1;
                data->invalid_move = true;
                return -1;
            }
            data->send_value = htons(RIGHT);    // serialized integer
            break;
        case 1:    // UP
            data->local_y = data->local_y - 1;
            if(data->local_y < 1)
            {
                data->local_y      = data->local_y + 1;
                data->invalid_move = true;
                return -1;
            }
            data->send_value = htons(UP);    // serialized integer
            break;
        case 3:    // DOWN
            data->local_y = data->local_y + 1;
            if(data->local_y >= COLS - 1)
            {
                data->local_y      = data->local_y - 1;
                data->invalid_move = true;
                return -1;
            }
            data->send_value = htons(DOWN);    // serialized integer
            break;
        default:
            printf("unknown keyboard input\n");
            sleep(2);
            return -1;
    }
    return 0;
}

in_port_t convert_port(const char *str, int *err)
{
    in_port_t port;
    char     *endptr;
    long      val;

    *err  = ERR_NONE;
    port  = 0;
    errno = 0;
    val   = strtol(str, &endptr, 10);    // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    // Check if no digits were found
    if(endptr == str)
    {
        *err = ERR_NO_DIGITS;
        goto done;
    }

    // Check for out-of-range errors
    if(val < 0 || val > UINT16_MAX)
    {
        *err = ERR_OUT_OF_RANGE;
        goto done;
    }

    // Check for trailing invalid characters
    if(*endptr != '\0')
    {
        *err = ERR_INVALID_CHARS;
        goto done;
    }

    port = (in_port_t)val;

done:
    return port;
}

int socket_connect(const program_data *data)
{
    struct sockaddr_storage server_addr;
    socklen_t               addr_len;
    int                     sock;
    int                     ret_val = 0;
    setup_network_address(&server_addr, &addr_len, data->local_ip, data->local_port, &ret_val);
    if(ret_val != 0)
    {
        perror("Setup network address failed");
        return -1;
    }

    // set up socket for receiving packets
    sock = socket(server_addr.ss_family, SOCK_DGRAM, 0);    // NOLINT(android-cloexec-socket)

    if(sock < 0)
    {
        perror("socket");
        return -1;
    }
    printf("Socket created with fd: %d\n", sock);

    // bind socket
    if(bind(sock, (struct sockaddr *)&server_addr, addr_len) < 0)
    {
        perror("bind");
        close(sock);
        return -1;
    }
    printf("Socket bound to port %d\n", data->local_port);
    return sock;
}
