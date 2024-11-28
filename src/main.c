#include <SDL2/SDL.h>
#include <fcntl.h>
#include <linux/input-event-codes.h>
#include <linux/input.h>
#include <ncurses.h>
#include <p101_fsm/fsm.h>
#include <p101_posix/p101_unistd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define LINES 40
#define COLS 40
#define ONE 1
#define ZERO 0

typedef struct
{
    const char *host_ip;
    const char *dest_ip;
    WINDOW     *win;
    bool        invalid_move;
    int         y0;
    int         x0;
} program_data;

static void             parse_arguments(const struct p101_env *env, int argc, char *argv[], bool *bad, bool *will, bool *did, program_data *data);
_Noreturn static void   usage(const char *program_name, int exit_code, const char *message);
static p101_fsm_state_t setup(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t wait_for_input(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t process_keyboard_input(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t process_network_input(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t move_local(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t move_remote(const struct p101_env *env, struct p101_error *err, void *arg);
static p101_fsm_state_t state_error(const struct p101_env *env, struct p101_error *err, void *arg);
static void             setup_signal_handler(void);
static void             sigint_handler(int signum);

static volatile sig_atomic_t exit_flag = 0;    // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

enum application_states
{
    SETUP = P101_FSM_USER_START,    // 2
    WAIT_FOR_INPUT,
    PROCESS_KEYBOARD_INPUT,
    PROCESS_CONTROLLER_INPUT,
    PROCESS_NETWORK_INPUT,
    MOVE_LOCAL,
    MOVE_REMOTE,
    ERROR
};

#define UNKNOWN_OPTION_MESSAGE_LEN 24

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
    printf("Host IP address: %s\n", data.host_ip);
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
            {P101_FSM_INIT,            SETUP,                  setup                 },
            {SETUP,                    WAIT_FOR_INPUT,         wait_for_input        },
            {WAIT_FOR_INPUT,           PROCESS_KEYBOARD_INPUT, process_keyboard_input},
            {WAIT_FOR_INPUT,           PROCESS_NETWORK_INPUT,  process_network_input },
            {PROCESS_KEYBOARD_INPUT,   MOVE_LOCAL,             move_local            },
            {PROCESS_CONTROLLER_INPUT, MOVE_LOCAL,             move_local            },
            {PROCESS_NETWORK_INPUT,    MOVE_REMOTE,            move_remote           },
            {PROCESS_KEYBOARD_INPUT,   WAIT_FOR_INPUT,         wait_for_input        }, //  if validation fails
            {PROCESS_CONTROLLER_INPUT, WAIT_FOR_INPUT,         wait_for_input        }, //  if validation fails
            {PROCESS_NETWORK_INPUT,    WAIT_FOR_INPUT,         wait_for_input        }, //  if validation fails
            {MOVE_LOCAL,               WAIT_FOR_INPUT,         wait_for_input        },
            {MOVE_REMOTE,              WAIT_FOR_INPUT,         wait_for_input        },
            {SETUP,                    ERROR,                  state_error           },
            {WAIT_FOR_INPUT,           ERROR,                  state_error           },
            {PROCESS_KEYBOARD_INPUT,   ERROR,                  state_error           },
            {PROCESS_CONTROLLER_INPUT, ERROR,                  state_error           },
            {PROCESS_NETWORK_INPUT,    ERROR,                  state_error           },
            {MOVE_LOCAL,               ERROR,                  state_error           },
            {MOVE_REMOTE,              ERROR,                  state_error           },
            {WAIT_FOR_INPUT,           P101_FSM_EXIT,          NULL                  }, //  if we ask to exit (cntrl c?)
            {ERROR,                    P101_FSM_EXIT,          NULL                  }
        };
        p101_fsm_state_t from_state;
        p101_fsm_state_t to_state;
        p101_fsm_run(fsm, &from_state, &to_state, &data, transitions, sizeof(transitions));
        p101_fsm_info_destroy(env, &fsm);
    }
    // deallocates memory and ends ncurses
    endwin();

    free(fsm_env);
    free(env);
    p101_error_reset(error);
    free(error);

    return EXIT_SUCCESS;
}

static void parse_arguments(const struct p101_env *env, int argc, char *argv[], bool *bad, bool *will, bool *did, program_data *data)
{
    int opt;
    data->host_ip = "";
    data->dest_ip = "";
    opterr        = 0;
    while((opt = p101_getopt(env, argc, argv, "hbdwa:s:")) != -1)
    {
        switch(opt)
        {
            case 'a':
            {
                data->host_ip = optarg;
                break;
            }
            case 's':
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
    int           lines = LINES;
    int           cols  = COLS;
    int           y0    = 1;
    int           x0    = 1;
    program_data *data;
    P101_TRACE(env);
    data = ((program_data *)arg);
    printf("setting up with %s and %s!\n", data->host_ip, data->dest_ip);

    // init screen and sets up screen
    initscr();

    // disables buffers on lines
    raw();

    // allows us to get arrow keys
    keypad(stdscr, TRUE);

    // suppresses char echoing
    noecho();

    // creates the window
    data->win = newwin(lines, cols, y0, x0);
    data->x0  = ONE;
    data->y0  = ONE;

    // refreshes the screen
    refresh();
    box(data->win, 0, 0);    // borders
    wrefresh(data->win);

    return WAIT_FOR_INPUT;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// In this function we will select/poll for keyboard input and/or receiving a UDP packet to our IP address
static p101_fsm_state_t wait_for_input(const struct p101_env *env, struct p101_error *err, void *arg)
{
    program_data *data;
    P101_TRACE(env);
    data = ((program_data *)arg);

    if(data->invalid_move)
    {
        mvwprintw(data->win, ONE, ONE, "INVALID MOVE!\n");
        wrefresh(data->win);
        data->invalid_move = false;
    }

    // we will only return process keyboard input if we receive keyboard input, of course
    return PROCESS_KEYBOARD_INPUT;
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
                data->x0 = data->x0 - 1;
                if(data->x0 < 1)
                {
                    data->x0           = data->x0 + 1;
                    data->invalid_move = true;
                    return WAIT_FOR_INPUT;
                }
                return MOVE_LOCAL;
            case KEY_RIGHT:
                data->x0 = data->x0 + 1;
                if(data->x0 >= COLS - 1)
                {
                    data->x0           = data->x0 - 1;
                    data->invalid_move = true;
                    return WAIT_FOR_INPUT;
                }
                return MOVE_LOCAL;
            case KEY_UP:
                data->y0 = data->y0 - 1;
                if(data->y0 < 1)
                {
                    data->y0           = data->y0 + 1;
                    data->invalid_move = true;
                    return WAIT_FOR_INPUT;
                }
                return MOVE_LOCAL;
            case KEY_DOWN:
                data->y0 = data->y0 + 1;
                if(data->y0 >= COLS - 1)
                {
                    data->y0           = data->y0 - 1;
                    data->invalid_move = true;
                    return WAIT_FOR_INPUT;
                }
                return MOVE_LOCAL;
            default:
                break;
        }
    }

    // deallocates memory and ends ncurses
    endwin();
    printf("exiting...\n");

    return P101_FSM_EXIT;
}

#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

static p101_fsm_state_t process_controller_input(const struct p101_env *env, struct p101_error *err, void *arg)
{
    program_data       *data = (program_data *)arg;
    SDL_Event           event;
    SDL_GameController *controller = NULL;
    P101_TRACE(env);

    // Initialize SDL GameController subsystem
    if(SDL_Init(SDL_INIT_GAMECONTROLLER) != 0)
    {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        return ERROR;
    }

    if(SDL_NumJoysticks() > 0)
    {
        controller = SDL_GameControllerOpen(0);
        if(!controller)
        {
            fprintf(stderr, "Could not open game controller: %s\n", SDL_GetError());
            SDL_Quit();
            return WAIT_FOR_INPUT;
        }
    }
    else
    {
        fprintf(stderr, "No game controllers connected.\n");
        SDL_Quit();
        return WAIT_FOR_INPUT;
    }

    // Poll for controller events
    while(SDL_PollEvent(&event))
    {
        if(event.type == SDL_CONTROLLERBUTTONDOWN || event.type == SDL_CONTROLLERBUTTONUP)
        {
            printf("Button event: button %d %s\n", event.cbutton.button, event.type == SDL_CONTROLLERBUTTONDOWN ? "pressed" : "released");

            switch(event.cbutton.button)
            {
                case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
                    data->x0--;
                    if(data->x0 < 1)
                    {
                        data->x0++;
                        data->invalid_move = true;
                        SDL_GameControllerClose(controller);
                        SDL_Quit();
                        return WAIT_FOR_INPUT;
                    }
                    SDL_GameControllerClose(controller);
                    SDL_Quit();
                    return MOVE_LOCAL;

                case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
                    data->x0++;
                    if(data->x0 >= COLS - 1)
                    {
                        data->x0--;
                        data->invalid_move = true;
                        SDL_GameControllerClose(controller);
                        SDL_Quit();
                        return WAIT_FOR_INPUT;
                    }
                    SDL_GameControllerClose(controller);
                    SDL_Quit();
                    return MOVE_LOCAL;

                case SDL_CONTROLLER_BUTTON_DPAD_UP:
                    data->y0--;
                    if(data->y0 < 1)
                    {
                        data->y0++;
                        data->invalid_move = true;
                        SDL_GameControllerClose(controller);
                        SDL_Quit();
                        return WAIT_FOR_INPUT;
                    }
                    SDL_GameControllerClose(controller);
                    SDL_Quit();
                    return MOVE_LOCAL;

                case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
                    data->y0++;
                    if(data->y0 >= LINES - 1)
                    {
                        data->y0--;
                        data->invalid_move = true;
                        SDL_GameControllerClose(controller);
                        SDL_Quit();
                        return WAIT_FOR_INPUT;
                    }
                    SDL_GameControllerClose(controller);
                    SDL_Quit();
                    return MOVE_LOCAL;

                default:
                    printf("Unhandled button: %d\n", event.cbutton.button);
                    break;
            }
        }
    }

    SDL_GameControllerClose(controller);
    SDL_Quit();
    return WAIT_FOR_INPUT;
}

#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// In this function we will process network input and validate the move.
// If the move is invalid, we will return WAIT_FOR_INPUT
// If the move is valid, we will return MOVE_REMOTE
static p101_fsm_state_t process_network_input(const struct p101_env *env, struct p101_error *err, void *arg)
{
    const program_data *data;
    P101_TRACE(env);
    data = ((program_data *)arg);
    printf("process_network_input called with host_ip %s and dest_ip %s\n", data->host_ip, data->dest_ip);
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
    printf("move_local called with host_ip %s and dest_ip %s\n", data->host_ip, data->dest_ip);
    wclear(data->win);
    mvwprintw(data->win, data->y0, data->x0, "*");
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
    printf("move_remote called with host_ip %s and dest_ip %s\n", data->host_ip, data->dest_ip);
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
