/* cansim V1 - write a message to the CAN network and see what happens as a result.
 *
 * The Control node firmware runs on a simulated bus together with a simulated motor controller
 * and BMS. Everything that happens is printed as a log line, timed relative to your last action
 * ("[+  12 ms] - ..."), in the colours of the team logo.
 *
 * Live mode (typing in a console): the network runs continuously in real time, the log appears as
 * things happen, and you can type a new command at any moment; each action restarts the timer.
 * Piped mode (commands from a file or pipe): each action runs until the network has been quiet for
 * QUIET_MS, then the next line is read, so scripted runs are reproducible.
 *
 * See README.md for instructions. */
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <conio.h>
#include <io.h>
#include <windows.h>
#ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#endif
#else
#include <unistd.h>
#endif
#include "can_proto.h"
#include "control_node.h"
#include "fault_mgr.h"
#include "sim_dbc.h"
#include "sim_util.h"
#include "sim_world.h"
#include "vehicle_fsm.h"

#define MAX_ARGS 16
#define QUIET_MS 300        /* an action has "settled" when nothing was logged for this long */
#define SETTLE_MAX_MS 5000  /* piped mode: never wait longer than this for an action to settle */

static volatile bool quit_requested;
static bool live;           /* real-time interactive feed (console input and output) */

/* Log state: times are relative to the last action (message, pedal, enable, release). */
static uint32_t action_t0;      /* simulated time of the last action */
static uint32_t last_logged;    /* simulated time of the last log line */
static int lines_since_action;
static bool settled = true;     /* the "settled" line for the last action has been printed */

/* Verdict on a frame you sent: decided in the ms after it crossed the bus. */
static bool verdict_pending;
static uint16_t verdict_id;
static can_proto_stats_t verdict_stats;

/* Live mode: the line being typed, redrawn below every log line. */
static char input[256];
static size_t input_len;

/* Colours (ANSI 256), muted from the team logo: the terminal's own text colour for text (the paper
 * boat), violet for the prompt and banner (the sail), amber for log times (the gold streaks).
 * Empty when the output is not a console (e.g. piped into a file), so the text stays clean there. */
static const char *plain = "", *accent = "", *stamp = "", *reset = "";

static void enable_colours(void)
{
#ifdef _WIN32
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode;
    if (!_isatty(_fileno(stdout)) || !GetConsoleMode(h, &mode) ||
        !SetConsoleMode(h, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING))
        return;
#else
    if (!isatty(fileno(stdout))) return;
#endif
    plain = "\x1b[0m";              /* default text colour, not bold */
    accent = "\x1b[1;38;5;140m";    /* bold muted violet, #af87d7 */
    stamp = "\x1b[38;5;179m";       /* muted amber, #d7af5f */
    reset = "\x1b[0m";
}

/* ------------------------------------------------------------------ the log */

static void prompt(void) { printf("%s~>%s %s", accent, plain, live ? input : ""); }

/* One log line, timed relative to the last action. In live mode it is printed above the line
 * being typed, which is then redrawn. */
static void log_line(const char *fmt, ...)
{
    char text[320];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof text, fmt, ap);
    va_end(ap);
    if (live) printf("\r\x1b[K");
    printf("  %s[+%6u ms]%s - %s\n", stamp, (unsigned)(sim_world()->now - action_t0), plain, text);
    if (live) prompt();
    last_logged = sim_world()->now;
    lines_since_action++;
}

static void on_event(uint32_t t, const char *text)
{
    (void)t;
    log_line("%s", text);
}

/* Starts the timer for a new action; the log lines that follow are its reaction. */
static void start_action(void)
{
    action_t0 = sim_world()->now;
    last_logged = action_t0;
    lines_since_action = 0;
    settled = false;
    printf("  Log:\n");
}

static void format_frame(const hal_can_frame_t *f, char *buf, size_t n)
{
    const sim_msg_t *m = sim_dbc_by_id(f->id);
    char hex[32] = "", dec[160] = "";
    size_t used = 0;
    for (uint8_t i = 0; i < f->dlc && i < 8; i++)
        used += (size_t)snprintf(hex + used, sizeof hex - used, "%s%02X", i ? " " : "", f->data[i]);
    if (m && f->dlc == m->dlc) sim_dbc_format(m, f->data, dec, sizeof dec);
    else if (m) snprintf(dec, sizeof dec, "(length %u, but %s has %u bytes)", f->dlc, m->name, m->dlc);
    else snprintf(dec, sizeof dec, "(unknown ID)");
    snprintf(buf, n, "0x%03X %-13s [%u] %-23s %s", f->id, m ? m->name : "?", f->dlc, hex, dec);
}

/* The Control node's own messages, as last seen on the bus: a change in any field is logged. */
static const uint16_t dut_ids[2] = { CAN_ID_CONTROL, CAN_ID_VEHICLE_STATUS };
static hal_can_frame_t out_last[2];
static uint32_t out_count[2];

static void reset_outputs(void)
{
    for (int i = 0; i < 2; i++) {
        out_count[i] = sim_bus_count(dut_ids[i]);
        sim_bus_last(dut_ids[i], &out_last[i], NULL);
    }
}

static void log_output_changes(void)
{
    for (int i = 0; i < 2; i++) {
        const sim_msg_t *m = sim_dbc_by_id(dut_ids[i]);
        hal_can_frame_t now;
        if (sim_bus_count(dut_ids[i]) == out_count[i]) continue;   /* no new frame */
        sim_bus_last(dut_ids[i], &now, NULL);
        if (out_count[i] > 0) {
            for (uint8_t f = 0; f < m->n_fields; f++) {
                const sim_field_t *fd = &m->fields[f];
                double a = sim_dbc_get(fd, out_last[i].data), b = sim_dbc_get(fd, now.data);
                char va[32], vb[32], faults[160] = "";
                if (strcmp(fd->name, "alive") == 0 || a == b) continue;
                sim_dbc_format_value(fd, a, va, sizeof va);
                sim_dbc_format_value(fd, b, vb, sizeof vb);
                if (strcmp(fd->name, "fault_mask") == 0) {
                    size_t used = (size_t)snprintf(faults, sizeof faults, "  (%s", (unsigned)b ? "" : "no faults");
                    for (int c = 0, n = 0; c < 16; c++)
                        if ((unsigned)b & (1u << c))
                            used += (size_t)snprintf(faults + used, sizeof faults - used, "%s%s", n++ ? " " : "",
                                                     fault_name((fault_code_t)c));
                    snprintf(faults + used, sizeof faults - used, ")");
                }
                log_line("%s.%s: %s -> %s%s%s%s", m->name, fd->name, va, vb, *fd->unit ? " " : "", fd->unit, faults);
            }
        }
        out_last[i] = now;
        out_count[i] = sim_bus_count(dut_ids[i]);
    }
}

static void log_settled(void)
{
    char cause[64] = "";
    if (vehicle_fsm_latched_fault() != F_COUNT)
        snprintf(cause, sizeof cause, " (cause: %s)", fault_name(vehicle_fsm_latched_fault()));
    log_line("%s - Control node: %s%s, throttle %.0f %%, enable %d",
             lines_since_action ? "settled" : "no reaction", vehicle_state_name(vehicle_fsm_state()), cause,
             vehicle_fsm_throttle(), vehicle_fsm_enable());
    settled = true;
}

/* Advances the network by 1 ms and logs everything it caused. */
static void step(void)
{
    sim_world_step();
    if (verdict_pending) {
        /* The frame crossed the bus in this ms and the Control node has processed it. */
        can_proto_stats_t s;
        can_proto_get_stats(&s);
        if (s.rx_dlc_errors > verdict_stats.rx_dlc_errors)
            log_line("Control node REJECTED the frame: wrong length (counted as a DLC error)");
        else if (s.rx_range_errors > verdict_stats.rx_range_errors)
            log_line("Control node REJECTED the frame: a value is out of range (counted as a range error)");
        else if (can_proto_rx_state(verdict_id) != CAN_RX_UNUSED)
            log_line("Control node accepted the frame :)");
        verdict_pending = false;
    }
    log_output_changes();
    if (!settled && sim_world()->now - last_logged >= QUIET_MS) log_settled();
}

/* Piped mode: run until the last action has settled. */
static void run_until_settled(void)
{
    while (!settled && sim_world()->now - action_t0 < SETTLE_MAX_MS) step();
    if (!settled) log_settled();
}

static void print_outputs_now(void)
{
    char buf[200];
    for (int i = 0; i < 2; i++) {
        hal_can_frame_t f;
        if (!sim_bus_last(dut_ids[i], &f, NULL)) continue;
        format_frame(&f, buf, sizeof buf);
        printf("    %s\n", buf);
    }
}

/* ------------------------------------------------------------------ parsing */

static const sim_msg_t *parse_msg(const char *s)
{
    const sim_msg_t *m = sim_dbc_by_name(s);
    long id;
    if (m) return m;
    if (sim_parse_long(s, &id)) return sim_dbc_by_id((uint16_t)id);
    return NULL;
}

static bool parse_duration(const char *s, uint32_t *ms)
{
    char *end;
    double v = strtod(s, &end);
    if (end == s || v < 0) return false;
    if (*end == '\0' || sim_streq(end, "ms")) *ms = (uint32_t)(v + 0.5);
    else if (sim_streq(end, "s")) *ms = (uint32_t)(v * 1000.0 + 0.5);
    else return false;
    return true;
}

/* ------------------------------------------------------------------ sending */

/* The simulated node that normally sends this message, if it is running; NULL otherwise. */
static const char *sim_sender_name(const sim_msg_t *m)
{
    sim_world_t *w = sim_world();
    if (strcmp(m->sender, "motor") == 0 && w->motor.enabled) return "motor controller";
    if (strcmp(m->sender, "bms") == 0 && w->bms.enabled) return "BMS";
    return NULL;
}

static const char *who_receives(uint16_t id, char *buf, size_t n)
{
    size_t used = 0;
    buf[0] = '\0';
    for (uint8_t i = 0; i < sim_bus_port_count(); i++) {
        sim_port_t *p = sim_bus_port(i);
        if (!sim_port_accepts(p, id)) continue;
        used += (size_t)snprintf(buf + used, n - used, "%s%s", used ? ", " : "",
                                 strcmp(p->name, "dut") == 0 ? "Control node" : p->name);
    }
    return used ? buf : "no simulated node";
}

static void send_frame(const hal_can_frame_t *f)
{
    char buf[200], recv[128];

    sim_world_send_manual(f);   /* crosses the bus in the next ms */
    can_proto_get_stats(&verdict_stats);
    verdict_id = f->id;
    verdict_pending = true;

    format_frame(f, buf, sizeof buf);
    printf("  Sent       : %s\n", buf);
    printf("  Received by: %s\n", who_receives(f->id, recv, sizeof recv));
    start_action();
}

static int send_raw(int argc, char **argv)
{
    hal_can_frame_t f;
    long id;
    memset(&f, 0, sizeof f);
    if (argc < 1 || !sim_parse_long(argv[0], &id) || id < 0 || id > 0x7FF) {
        printf("  ERROR: usage: raw <id> [byte ...]   e.g. raw 0x200 1A 04 FA 00\n");
        return -1;
    }
    if (argc - 1 > 8) {
        printf("  ERROR: at most 8 data bytes\n");
        return -1;
    }
    f.id = (uint16_t)id;
    for (int i = 1; i < argc; i++) {
        char *end;
        unsigned long b = strtoul(argv[i], &end, 16);
        if (*end || b > 0xFF) {
            printf("  ERROR: bad byte '%s' (hex, e.g. 1A)\n", argv[i]);
            return -1;
        }
        f.data[f.dlc++] = (uint8_t)b;
    }
    send_frame(&f);
    return 0;
}

static int send_message(const sim_msg_t *m, int argc, char **argv)
{
    hal_can_frame_t f;
    const char *node = sim_sender_name(m);
    char held[160] = "";
    size_t used = 0;
    memset(&f, 0, sizeof f);
    f.id = m->id;
    (void)sim_bus_last(m->id, &f, NULL);   /* fields you leave out keep their last value on the bus */
    f.dlc = m->dlc;
    for (int i = 0; i < argc; i++) {
        char *eq = strchr(argv[i], '=');
        int fi;
        double v;
        if (!eq) {
            printf("  ERROR: expected field=value, got '%s'\n", argv[i]);
            return -1;
        }
        *eq = '\0';
        fi = sim_dbc_field_index(m, argv[i]);
        if (fi < 0) {
            printf("  ERROR: %s has no field '%s'. Fields:", m->name, argv[i]);
            for (uint8_t k = 0; k < m->n_fields; k++) printf(" %s", m->fields[k].name);
            printf("\n");
            return -1;
        }
        if (!sim_dbc_parse_value(&m->fields[fi], eq + 1, &v)) {
            printf("  ERROR: bad value '%s'\n", eq + 1);
            return -1;
        }
        if (!sim_dbc_set(&m->fields[fi], f.data, v)) printf("  WARNING: %s does not fit the field, clamped\n", argv[i]);
        if (node) {
            /* The node keeps sending this value; otherwise its next periodic frame would undo it. */
            char val[32];
            sim_override_set(m, fi, v);
            sim_dbc_format_value(&m->fields[fi], v, val, sizeof val);
            used += (size_t)snprintf(held + used, sizeof held - used, "%s%s=%s", used ? " " : "",
                                     m->fields[fi].name, val);
        }
    }
    if (node && used)
        printf("  The simulated %s keeps sending %s in every %s frame ('release %s' to stop).\n", node, held,
               m->name, m->name);
    send_frame(&f);
    return 0;
}

/* ------------------------------------------------------------------ commands */

static void print_help(void)
{
    printf("  Send a message (fields you leave out keep their last value on the bus):\n"
           "    <Message> [field=value ...]     e.g.  MotorTemps motor_temp=105\n"
           "                                          BMSData voltage=41.5 soc=12\n"
           "    raw <id> [byte ...]             e.g.  raw 0x201 10 27 00   (any ID/length, bytes in hex)\n"
           "    release <Message> | release all  the simulated node goes back to its own values\n"
           "  Driver inputs of the Control node:\n"
           "    pedal <percent> [percent_B]     e.g.  pedal 50\n"
           "    enable on|off\n"
           "  Other:\n"
           "    messages                        list messages and their fields\n"
           "    status                          current state of all nodes\n"
           "    wait <ms>                       scripts only: let time pass (wait 2s); live time runs by itself\n"
           "    help, quit\n"
           "  The log shows everything that happens, timed from your last action: [+  12 ms] - ...\n"
           "  You can type a new command at any time.\n");
}

static void print_messages(void)
{
    for (uint8_t i = 0; i < sim_dbc_count(); i++) {
        const sim_msg_t *m = sim_dbc_at(i);
        const can_msg_def_t *d = can_msg_def(m->id);
        printf("  0x%03X %-13s every %4u ms, from %-5s:", m->id, m->name, d->period_ms,
               strcmp(m->sender, "dut") == 0 ? "Control node" : m->sender);
        for (uint8_t f = 0; f < m->n_fields; f++)
            printf(" %s%s%s%s", m->fields[f].name, *m->fields[f].unit ? "[" : "", m->fields[f].unit,
                   *m->fields[f].unit ? "]" : "");
        printf("\n");
    }
}

static void print_status(void)
{
    sim_world_t *w = sim_world();
    static const char *const rx_names[] = { "UNUSED", "NO_DATA", "OK", "TIMEOUT", "RECOVERY" };
    char buf[128];
    printf("  t = %u ms\n", (unsigned)w->now);
    printf("  Control node : %s", vehicle_state_name(vehicle_fsm_state()));
    if (vehicle_fsm_latched_fault() != F_COUNT) printf(" (cause: %s)", fault_name(vehicle_fsm_latched_fault()));
    printf(", throttle %.1f %%, enable %d, limit %.0f %%\n", vehicle_fsm_throttle(), vehicle_fsm_enable(),
           control_node_limit_pct());
    printf("  Inputs       : pedal %.0f %% / %.0f %%, enable switch %s\n", w->pedal_a_pct, w->pedal_b_pct,
           w->enable_switch ? "ON" : "OFF");
    printf("  Active faults:");
    if (fault_active_mask() == 0) printf(" none");
    for (int c = 0; c < F_COUNT; c++)
        if (fault_is_active((fault_code_t)c)) printf(" %s", fault_name((fault_code_t)c));
    printf("\n  Receiving    :");
    for (uint8_t i = 0; i < can_msg_def_count(); i++) {
        const can_msg_def_t *d = can_msg_def_at(i);
        if (can_proto_rx_state(d->id) != CAN_RX_UNUSED) printf(" %s=%s", d->name, rx_names[can_proto_rx_state(d->id)]);
    }
    printf("\n  Motor ctrl   : %s, %.0f rpm, %.1f A, %.1f C\n", sim_motor_state_name(w->motor.state), w->motor.rpm,
           w->motor.current_a, w->motor.motor_temp_c);
    printf("  BMS          : %.1f V, %.1f A, SOC %.1f %%\n", w->bms.voltage_v, w->bms.current_a, w->bms.soc_pct);
    printf("  Held values  :");
    if (sim_override_count() == 0) printf(" none");
    printf("\n");
    for (int i = 0; sim_override_describe(i, buf, sizeof buf); i++) printf("    %s\n", buf);
    printf("  Last frames from the Control node:\n");
    print_outputs_now();
}

static void exec_line(char *line)
{
    char *argv[MAX_ARGS];
    int argc = 0;
    char *p = line;
    const sim_msg_t *m;

    if ((unsigned char)p[0] == 0xEF && (unsigned char)p[1] == 0xBB && (unsigned char)p[2] == 0xBF) p += 3;
    for (char *c = p; *c; c++)
        if (*c == '#' || *c == '\r' || *c == '\n') { *c = '\0'; break; }
    while (*p && argc < MAX_ARGS) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = '\0';
    }
    if (argc == 0) return;

    if (sim_streq(argv[0], "quit") || sim_streq(argv[0], "exit")) {
        quit_requested = true;
    } else if (sim_streq(argv[0], "help")) {
        print_help();
    } else if (sim_streq(argv[0], "messages") || sim_streq(argv[0], "list")) {
        print_messages();
    } else if (sim_streq(argv[0], "status")) {
        print_status();
    } else if (sim_streq(argv[0], "release")) {
        if (argc != 2 || (!sim_streq(argv[1], "all") && (m = parse_msg(argv[1])) == NULL)) {
            printf("  ERROR: usage: release <Message> | release all\n");
            return;
        }
        if (sim_streq(argv[1], "all")) sim_override_clear_all();
        else if (!sim_override_clear(m, -1)) {
            printf("  No values held for %s\n", m->name);
            return;
        }
        printf("  Released: the simulated nodes send their nominal values again\n");
        start_action();
    } else if (sim_streq(argv[0], "raw")) {
        send_raw(argc - 1, argv + 1);
    } else if (sim_streq(argv[0], "send") && argc >= 2 && (m = parse_msg(argv[1])) != NULL) {
        send_message(m, argc - 2, argv + 2);
    } else if ((m = sim_dbc_by_name(argv[0])) != NULL) {
        send_message(m, argc - 1, argv + 1);
    } else if (sim_streq(argv[0], "pedal")) {
        double a, b;
        if (argc < 2 || argc > 3 || !sim_parse_double(argv[1], &a) || (argc == 3 && !sim_parse_double(argv[2], &b))) {
            printf("  ERROR: usage: pedal <percent> [percent_B]\n");
            return;
        }
        if (argc == 2) b = a;
        sim_world_set_pedal(a, b);
        printf("  Pedal set to %.0f %% / %.0f %%\n", a, b);
        start_action();
    } else if (sim_streq(argv[0], "enable")) {
        if (argc != 2 || (!sim_streq(argv[1], "on") && !sim_streq(argv[1], "off"))) {
            printf("  ERROR: usage: enable on|off\n");
            return;
        }
        sim_world_set_enable(sim_streq(argv[1], "on"));
        printf("  Enable switch %s\n", sim_streq(argv[1], "on") ? "ON" : "OFF");
        start_action();
    } else if (sim_streq(argv[0], "wait")) {
        uint32_t ms;
        if (live) {
            printf("  Time is running live; the log keeps going by itself.\n");
            return;
        }
        if (argc != 2 || !parse_duration(argv[1], &ms) || ms < 1) {
            printf("  ERROR: usage: wait <ms>   e.g. wait 500, wait 2s\n");
            return;
        }
        while (ms--) step();   /* keeps logging, timed from the last action */
    } else {
        printf("  ERROR: '%s' is not a message or command. Type 'help' or 'messages'.\n", argv[0]);
    }
}

#ifdef _WIN32
/* ------------------------------------------------------------------ live mode */

#define HISTORY 32
static char history[HISTORY][sizeof input];
static int history_count, history_pos;

static uint64_t wall_ms(void)
{
    LARGE_INTEGER freq, count;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&count);
    return (uint64_t)(count.QuadPart * 1000 / freq.QuadPart);
}

static void set_input(const char *text)
{
    snprintf(input, sizeof input, "%s", text);
    input_len = strlen(input);
    printf("\r\x1b[K");
    prompt();
}

/* Handles one key; returns true when a complete line is in `input`. */
static bool handle_key(void)
{
    int ch = _getch();
    if (ch == 0 || ch == 0xE0) {   /* arrow keys: up/down browse the command history */
        int key = _getch();
        if (key == 72 && history_pos > 0) set_input(history[--history_pos]);
        else if (key == 80 && history_pos < history_count) {
            history_pos++;
            set_input(history_pos < history_count ? history[history_pos] : "");
        }
        return false;
    }
    if (ch == '\r') return true;
    if (ch == 3) {                 /* Ctrl+C */
        quit_requested = true;
        return false;
    }
    if (ch == '\b') {
        if (input_len > 0) {
            input[--input_len] = '\0';
            printf("\b \b");
        }
        return false;
    }
    if (ch >= 32 && ch < 127 && input_len < sizeof input - 1) {
        input[input_len++] = (char)ch;
        input[input_len] = '\0';
        putchar(ch);
    }
    return false;
}

/* The network runs in real time (1 simulated ms per real ms); keys are handled as they come. */
static void run_live(void)
{
    uint64_t wall0 = wall_ms();
    uint32_t sim0 = sim_world()->now;

    printf("running in live mode, type at any time\n\n");
    prompt();
    while (!quit_requested) {
        uint32_t target;
        while (!quit_requested && _kbhit()) {
            char line[sizeof input];
            if (!handle_key()) continue;
            printf("\n");
            snprintf(line, sizeof line, "%s", input);
            if (input_len > 0 && (history_count == 0 || strcmp(history[history_count - 1], input) != 0)) {
                if (history_count == HISTORY) memmove(history[0], history[1], sizeof history[0] * (HISTORY - 1));
                else history_count++;
                snprintf(history[history_count - 1], sizeof history[0], "%s", input);
            }
            history_pos = history_count;
            input[0] = '\0';
            input_len = 0;
            exec_line(line);
            if (!quit_requested) prompt();
        }
        target = sim0 + (uint32_t)(wall_ms() - wall0);
        if ((int32_t)(target - sim_world()->now) > 2000) {   /* fell far behind (e.g. a long print): resync */
            sim0 += (uint32_t)(target - sim_world()->now) - 2000;
            target = sim0 + (uint32_t)(wall_ms() - wall0);
        }
        while ((int32_t)(target - sim_world()->now) > 0) step();
        Sleep(1);
    }
    printf("\n");
}
#endif

static void on_ctrl_c(int sig)
{
    (void)sig;
    quit_requested = true;   /* leave the loop normally, so the terminal colours are reset */
}

int main(int argc, char **argv)
{
    char line[512];
    bool allow_live = !(argc > 1 && strcmp(argv[1], "--no-live") == 0);

    setvbuf(stdout, NULL, _IONBF, 0);
    signal(SIGINT, on_ctrl_c);
    enable_colours();
    printf("%s", plain);
    sim_world_reset(NULL, 0);
    sim_world_run(200);   /* boot: the nodes start sending, the Control node reaches STANDBY */
    reset_outputs();
    action_t0 = sim_world()->now;
    sim_set_event_sink(on_event);
#ifdef _WIN32
    /* Live only with a real console on both sides, and colours working (they need VT processing). */
    live = allow_live && _isatty(_fileno(stdin)) && *reset != '\0';
#endif

    printf("%s  ~ can simulator v1.8 ~   %s for all commands type 'help', 'exit' to exit\n", accent, plain);
    printf("network running: control node in %s, simulated motor controller and BMS sending (t = %u ms).\n",
           vehicle_state_name(vehicle_fsm_state()), (unsigned)sim_world()->now);
#ifdef _WIN32
    if (live) run_live();
#endif
    while (!live && !quit_requested) {   /* piped mode */
        printf("\n");
        prompt();
        if (!fgets(line, sizeof line, stdin)) break;
        exec_line(line);
        run_until_settled();
    }
    printf("  bye bye%s\n", reset);
    return 0;
}
