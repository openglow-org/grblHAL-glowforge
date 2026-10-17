/*
  serial_test.c - host unit test for the Grbl stream: the RX ring under a
  sender that ignores flow control, the welcome banner on a connect, the
  TX ring's stall bound, the -b listen address, and the start notice

  grblHAL is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  Copyright 2026 514 LLC d/b/a OpenGlow
  Written by Scott Wiederhold
  SPDX-License-Identifier: GPL-3.0-or-later

  A sender that writes more than the RX ring holds (Bf: reports 1023
  characters free) overruns it. The ring must never hand the parser a
  mangled line: the line that overran is dropped whole (the part already
  in the ring is unwritten, the rest is discarded through its newline),
  every line before it is delivered intact, real-time characters keep
  passing while the ring is full, and the overrun is reported once
  through serial_rx_overflow_take() so the driver can abort the job.

  The RX cases push bytes into rx_byte() the way rx_poll() does and read
  them back through the stream's read function. The TX cases run the
  pump: a sender that connects to the listening socket is welcomed with
  the banner (from a top-level poll, never from inside a blocked write),
  a settings dump queues in the ring without waiting on the sender, a
  sender that pauses under a second keeps its connection, and one that
  pauses longer is dropped with the ring flushed.

  Keeping senders out: a peer on this host is told from one on the
  network by its address, the dual-stack socket's mapped IPv4 included;
  while senders are kept out, one from the network is turned away with a
  message line and nothing of the session changes (no generation, no
  banner), one from this host connects, and the connected sender is
  dropped unless it is on this host. A sender line is open from the
  core's first byte of it to its status, so a dwell or a wait that runs
  inside a line reads as the sender at work.

  The -b listen address (src/bind_addr.c): an IPv6 literal binds as
  written, an IPv4 literal in its v4-mapped form on the dual-stack socket
  (all sixteen bytes written, whatever the output held), and a host name
  is refused, as is a short dotted form: an IPv4 literal is the dotted
  quad (main.c exits on the refusal).

  The start notice (src/main.c): the test runs the controller's entry
  point with its callees stubbed. The controller logs one notice that
  names the overrides in force, in the lists' order, names only, and no
  notice when none is in force. A test hook is in force only in a
  null-sink run: with GFSINK set the notice names none. The notice's
  lists are held to the sources. Every getenv and every call of the
  gate, gfio_test_hook, in the .c and .h files of src/ names its
  variable, with or without white space before its parenthesis
  (secure_getenv too), the notice's own loops and the gate's own code
  aside. ENV_OVERRIDES names each variable that a getenv reads, once,
  but the three the supervisor sets on every spawn. TEST_HOOKS names
  each variable that the sources read through the gate, once, and no
  getenv reads one. The test reads the sources from DRIVER_SRC, which
  CMakeLists.txt sets.
*/
#include <dirent.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ifaddrs.h>
#include <net/if.h>

#include "bind_addr.h"

/* --- stubs the serial source links against -------------------------- */

static int realtime_seen;           /* bytes taken by the real-time path */

bool protocol_enqueue_realtime_command(uint8_t c)
{
    if(c == '?' || c == '!' || c == '~' || c == 0x18) {
        realtime_seen++;
        return true;
    }
    return false;
}

void driver_request_exit(void) {}
uint8_t platform_poll_stdin(void) { return 0; }
bool gflaser_resume_gate(void) { return false; }   /* no held laser job here */
struct pollfd;
void ctlport_poll(void) {}                          /* no controller port here */
unsigned ctlport_pollfds(struct pollfd *fds) { (void)fds; return 0; }

/* --- driver source under test ---------------------------------------- */
#include "../src/serial.c"

/* --- grbl core stubs (declared by the headers the source pulled in) --- */
grbl_hal_t hal;
grbl_t grbl;
bool stream_rx_suspend(stream_rx_buffer_t *rx, bool suspend)
{ (void)rx; (void)suspend; return false; }
bool stream_connected(void) { return true; }
parser_state_t gc_state;        /* welcome() clears its last_error on a connect */

/* The core's banner, as the report module writes it through the stream. */
#define BANNER "\r\nGrblHAL test ['$' for help]\r\n"
static int banners;
static void fake_banner(stream_write_ptr write) { banners++; write(BANNER); }

/* The blocking callback the core gives a full ring: the real one runs
   the protocol loop's real-time pass, which polls the stream. The stand-in
   polls, and plays the sender: a peer that starts reading `peer_after_s`
   after the write blocked (never, when peer_reads is false). */
static int peer_rd = -1;
static bool peer_reads;
static double peer_after_s, blocked_at;
static int cb_calls;

static double now_s(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + t.tv_nsec / 1e9;
}

static bool blocking_cb(void)
{
    if(cb_calls++ == 0)
        blocked_at = now_s();
    if(peer_reads && now_s() - blocked_at >= peer_after_s) {
        char buf[512];
        (void)!read(peer_rd, buf, sizeof(buf));
    }
    serial_poll();
    struct timespec ts = { 0, 1000000 };
    nanosleep(&ts, NULL);
    return true;
}

static int listen_loopback(uint16_t *port)
{
    int fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = 0,
                              .sin_addr.s_addr = htonl(INADDR_LOOPBACK) };
    socklen_t sl = sizeof(sa);
    if(fd < 0 || bind(fd, (struct sockaddr *)&sa, sl) != 0 || listen(fd, 4) != 0 ||
       getsockname(fd, (struct sockaddr *)&sa, &sl) != 0)
        return -1;
    *port = ntohs(sa.sin_port);
    return fd;
}

static int connect_loopback(uint16_t port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = htons(port),
                              .sin_addr.s_addr = htonl(INADDR_LOOPBACK) };
    if(fd < 0 || connect(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0)
        return -1;
    return fd;
}

/* What a sender receives within `ms`; "" when nothing comes. */
static const char *received(int fd, int ms)
{
    static char buf[256];
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    size_t len = 0;
    while(len < sizeof(buf) - 1 && poll(&pfd, 1, ms) > 0) {
        ssize_t n = read(fd, buf + len, sizeof(buf) - 1 - len);
        if(n <= 0)
            break;
        len += (size_t)n;
        ms = 50;                         /* the rest of the same burst */
    }
    buf[len] = '\0';
    return buf;
}

/* An address of this host that is not the loopback, for a peer the
   stream must see as one from the network. */
static bool own_address(struct in_addr *out)
{
    struct ifaddrs *list, *i;
    bool found = false;
    if(getifaddrs(&list) != 0)
        return false;
    for(i = list; i && !found; i = i->ifa_next) {
        if(!i->ifa_addr || i->ifa_addr->sa_family != AF_INET || !(i->ifa_flags & IFF_UP))
            continue;
        struct in_addr a = ((struct sockaddr_in *)i->ifa_addr)->sin_addr;
        if((ntohl(a.s_addr) >> 24) != 127) {
            *out = a;
            found = true;
        }
    }
    freeifaddrs(list);
    return found;
}

static int listen_any(uint16_t *port)
{
    int fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = 0, .sin_addr.s_addr = htonl(INADDR_ANY) };
    socklen_t sl = sizeof(sa);
    if(fd < 0 || bind(fd, (struct sockaddr *)&sa, sl) != 0 || listen(fd, 4) != 0 ||
       getsockname(fd, (struct sockaddr *)&sa, &sl) != 0)
        return -1;
    *port = ntohs(sa.sin_port);
    return fd;
}

static int connect_to(struct in_addr a, uint16_t port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = htons(port), .sin_addr = a };
    if(fd < 0 || connect(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0)
        return -1;
    return fd;
}

static bool loopback_text(const char *text)
{
    struct sockaddr_storage ss;
    memset(&ss, 0, sizeof(ss));
    if(strchr(text, ':')) {
        struct sockaddr_in6 *v6 = (struct sockaddr_in6 *)&ss;
        v6->sin6_family = AF_INET6;
        inet_pton(AF_INET6, text, &v6->sin6_addr);
    } else {
        struct sockaddr_in *v4 = (struct sockaddr_in *)&ss;
        v4->sin_family = AF_INET;
        inet_pton(AF_INET, text, &v4->sin_addr);
    }
    return peer_loopback(&ss);
}

static status_code_t fake_status(status_code_t status) { return status; }

/* Bytes serialPutC accepted; a drop ends the count. */
static int put_bytes(int n)
{
    int i = 0;
    while(i < n && serialPutC('x'))
        i++;
    return i;
}

/* --- test driver ----------------------------------------------------- */

static int failures;

#define CHECK(cond, msg) do { \
    if (cond) { printf("  ok: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); failures++; } \
} while (0)

static void push(const char *s)
{
    while(*s)
        rx_byte((uint8_t)*s++);
}

/* Read every complete line out of the ring. Returns the count; the
   lines land in `out`, and `bad` counts lines that are not one of the
   two shapes the test sent. */
static int drain(char out[][32], int max, int *bad, const char *a, const char *b)
{
    int n = 0, len = 0;
    char line[32];
    int16_t c;
    *bad = 0;
    while((c = serialGetC()) != SERIAL_NO_DATA) {
        if(c == '\n') {
            line[len] = '\0';
            if(n < max)
                snprintf(out[n], 32, "%s", line);
            if(strcmp(line, a) != 0 && strcmp(line, b) != 0)
                (*bad)++;
            n++;
            len = 0;
        } else if(len < 31)
            line[len++] = (char)c;
    }
    if(len)                          /* a partial line left in the ring */
        (*bad)++;
    return n;
}

/* The variables the supervisor sets on every spawn: never an override,
   so the start notice leaves them out of its list. */
static const char *const spawn_env[] = { "GFSINK", "GF_PULSE_FD", "GF_REPORT_SECRET" };

#define ENV_NAMES 64
typedef char env_name_t[40];

static bool env_has(const env_name_t *set, int n, const char *name)
{
    for(int i = 0; i < n; i++)
        if(!strcmp(set[i], name))
            return true;
    return false;
}

static bool env_spawned(const char *name)
{
    for(size_t i = 0; i < sizeof(spawn_env) / sizeof(*spawn_env); i++)
        if(!strcmp(spawn_env[i], name))
            return true;
    return false;
}

/* The variable name quoted at p ("NAME": capitals, digits, underscores),
   copied to out. Returns the character after its closing quote, or NULL
   when p holds none. */
static const char *env_quoted(const char *p, char *out, size_t len)
{
    size_t n = 0;
    if(*p++ != '"')
        return NULL;
    for(; (*p >= 'A' && *p <= 'Z') || (*p >= '0' && *p <= '9') || *p == '_'; p++)
        if(n + 1 < len)
            out[n++] = *p;
    out[n] = '\0';
    return *p == '"' && n ? p + 1 : NULL;
}

static char *env_slurp(const char *path)
{
    FILE *f = fopen(path, "rb");
    char *text = NULL;
    long size = -1;
    if(f && fseek(f, 0, SEEK_END) == 0 && (size = ftell(f)) >= 0 && fseek(f, 0, SEEK_SET) == 0 &&
       (text = malloc((size_t)size + 1)) != NULL)
        text[fread(text, 1, (size_t)size, f)] = '\0';
    if(f)
        fclose(f);
    return text;
}

/* Every call of `word` in text: a quoted name joins set, an argument that
   an entry of `own` begins is the notice's own loop or the gate's own
   code, and any other argument is a read that no list can name. Returns
   the count of those. */
static int env_calls(const char *text, const char *word, const char *const *own,
                     env_name_t *set, int *n)
{
    int unnamed = 0;
    for(const char *p = text; p && (p = strstr(p, word)) != NULL; ) {
        env_name_t name;
        p += strlen(word);
        p += strspn(p, " \t\r\n");
        if(*p != '(')
            continue;                   /* the word in a comment, not a call */
        p++;
        p += strspn(p, " \t\r\n");
        if(env_quoted(p, name, sizeof(name))) {
            if(!env_has(set, *n, name) && *n < ENV_NAMES)
                strcpy(set[(*n)++], name);
            continue;
        }
        bool exempt = false;
        for(const char *const *o = own; *o; o++)
            exempt = exempt || !strncmp(p, *o, strlen(*o));
        unnamed += !exempt;
    }
    return unnamed;
}

/* The names of the table in main.c that `decl` opens, into set; a name
   listed twice is said, under `label`, and counted in *differ. Returns
   false when main.c holds no such table or it names nothing. */
static bool env_table(const char *main_c, const char *decl, const char *label,
                      env_name_t *set, int *n, int *differ)
{
    const char *table = main_c ? strstr(main_c, decl) : NULL;
    const char *end = table ? strstr(table, "};") : NULL;
    for(const char *p = table; p && end && p < end; p++) {
        env_name_t name;
        const char *next = env_quoted(p, name, sizeof(name));
        if(!next)
            continue;
        if(env_has(set, *n, name)) {
            printf("  %s names %s twice\n", label, name);
            (*differ)++;
        } else if(*n < ENV_NAMES)
            strcpy(set[(*n)++], name);
        p = next - 1;
    }
    return end != NULL && *n > 0;
}

/* --- the controller's entry point, for its start notice --------------
   main.c runs with its callees stubbed: the settings file, the platform
   and the core do nothing, and fflog keeps the notice lines it is given,
   so the test reads what the controller logs as it starts. */
#define NOTICE "environment overrides in force:"
static char notice[512];
static int notices;

void fflog_init(const char *ident) { (void)ident; }
void fflog(int prio, const char *fmt, ...)
{
    char line[512];
    va_list ap;
    (void)prio;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if(!strncmp(line, NOTICE, strlen(NOTICE))) {
        notices++;
        snprintf(notice, sizeof(notice), "%s", line);
    }
}
bool set_eeprom_name(const char *name) { (void)name; return true; }
void eeprom_close(void) {}
void platform_init(void) {}
int grbl_enter(void) { return 0; }

#define main controller_main
#include "../src/main.c"
#undef main

int main(void)
{
    static char got[200][32];
    const char *line = "G1 X-30 F1500\n";      /* 14 bytes, as a fill sends them */
    const char *after = "G1 Y0.3 F1500\n";
    int bad;

    printf("RX ring under a sender that ignores flow control:\n");

    /* The ring takes RX_BUFFER_SIZE - 1 = 1023 bytes. 73 lines of 14
       bytes is 1022, one byte short: the 74th line's first byte fits and
       its second overruns, so the byte already in the ring has to be
       unwritten. Then the sender keeps going for two more lines, with a
       '?' in the middle of the overrun. */
    serialRxFlush();
    for(int i = 0; i < 73; i++)
        push(line);
    CHECK(serialRxFree() == 1, "73 lines of 14 bytes leave one byte free");
    CHECK(!serial_rx_overflow_take(), "no overrun reported while the ring merely fills");

    push("G1 X-3");                             /* the 74th line begins... */
    push("?");                                  /* a status poll during the overrun */
    push("0 F1500\n");                          /* ...and ends */
    push(after);                                /* the 75th line, whole */
    CHECK(realtime_seen == 1, "a real-time character passes while the ring is full");
    CHECK(serial_rx_overflow_take(), "the overrun is reported once");
    CHECK(!serial_rx_overflow_take(), "and only once");

    /* Make room and let the sender's next line in, so the read side sees
       what a sender feeding against Bf would deliver after the overrun. */
    int n = drain(got, 200, &bad, "G1 X-30 F1500", "G1 Y0.3 F1500");
    CHECK(n == 73, "every line that fit is delivered");
    CHECK(bad == 0, "no line is mangled and no partial line is left behind");
    CHECK(n > 0 && strcmp(got[n - 1], "G1 X-30 F1500") == 0,
          "the last delivered line is the last one that fit, not a fragment");

    /* The lines after the overrun are dropped through the newline of the
       line that overran, and the ring is back to whole lines after it. */
    push(after);
    n = drain(got, 200, &bad, "G1 X-30 F1500", "G1 Y0.3 F1500");
    CHECK(n == 1 && bad == 0 && strcmp(got[0], "G1 Y0.3 F1500") == 0,
          "a line sent after the overrun arrives whole");

    /* --- the banner on a connect --------------------------------------- */
    printf("The welcome banner on a connect:\n");
    grbl.report.init_message = fake_banner;
    hal.stream_blocking_callback = blocking_cb;
    uint16_t port;
    int lfd = listen_loopback(&port);
    CHECK(lfd >= 0, "a listening socket on the loopback");
    serial_set_listen_fd(lfd);

    int sender = connect_loopback(port);
    CHECK(sender >= 0, "a sender connects");
    serial_poll();                                  /* accept, welcome, drain */
    (void)received(sender, 500);                    /* its banner */

    /* A second connect displaces the first and is welcomed in turn - but
       not from a poll that runs inside a blocked write. */
    int sender2 = connect_loopback(port);
    tx_blocked = true;                              /* as serialPutC leaves it while it waits */
    serial_poll();
    CHECK(client_fd >= 0 && banners == 1 && received(sender2, 100)[0] == '\0',
          "a poll inside a blocked write accepts the sender but holds the banner");
    tx_blocked = false;
    serial_poll();
    CHECK(banners == 2 && strcmp(received(sender2, 500), BANNER) == 0,
          "the next top-level poll writes the banner to the new sender");
    CHECK(received(sender, 100)[0] == '\0', "the displaced sender gets nothing");
    close(sender);
    close(sender2);
    drop_client();

    /* --- a new sender starts with a clean parser error state ------------- */
    printf("A connect clears a held g-code error:\n");
    /* The core holds every g-code line after one that errored, until a
       blank line acknowledges it; the hold outlives the connection. A
       soft-limit-rejected jog leaves it. The sender that connects next
       never sent that line, so the banner clears it. */
    gc_state.last_error = 15;                       /* Status_TravelExceeded, from a prior jog */
    int sender3 = connect_loopback(port);
    CHECK(sender3 >= 0, "a fresh sender connects");
    serial_poll();                                  /* accept + welcome */
    CHECK(gc_state.last_error == Status_OK,
          "the connect cleared the held error so the new sender's first line is not refused");
    close(sender3);
    drop_client();
    serial_set_listen_fd(-1);
    close(lfd);

    /* --- keeping senders out ------------------------------------------- */
    printf("Keeping senders out:\n");
    CHECK(loopback_text("127.0.0.1") && loopback_text("127.5.5.5") && loopback_text("::1") &&
          loopback_text("::ffff:127.0.0.1"), "this host: the loopback net, ::1, and the mapped loopback");
    CHECK(!loopback_text("10.0.0.1") && !loopback_text("192.168.1.5") && !loopback_text("::ffff:192.168.1.5") &&
          !loopback_text("2001:db8::1") && !loopback_text("::"), "the network: every other address");

    uint16_t aport;
    int lany = listen_any(&aport);
    CHECK(lany >= 0, "a listening socket on every address");
    serial_set_listen_fd(lany);
    serial_keep_out(true);
    CHECK(serial_keeping_out(), "senders are kept out");
    int b0 = banners;
    int local = connect_loopback(aport);
    serial_poll();
    CHECK(local >= 0 && client_fd >= 0 && client_loopback && banners == b0 + 1,
          "a sender on this host connects and is welcomed");
    CHECK(!serial_drop_sender() && client_fd >= 0, "dropping the sender leaves one on this host");
    close(local);
    drop_client();

    struct in_addr own;
    if(own_address(&own)) {
        unsigned gen = serial_client_generation();
        b0 = banners;
        int net = connect_to(own, aport);
        serial_poll();
        CHECK(net >= 0 && client_fd < 0 && serial_client_generation() == gen && banners == b0 &&
              client_peer[0] == '\0', "a sender from the network is turned away, and nothing of the session changes");
        CHECK(strstr(received(net, 500), "[MSG:") != NULL, "it reads why");
        char c;
        CHECK(read(net, &c, 1) == 0, "and the socket is closed");
        close(net);

        serial_keep_out(false);
        net = connect_to(own, aport);
        serial_poll();
        CHECK(client_fd >= 0 && !client_loopback && strcmp(received(net, 500), BANNER) == 0,
              "let back in, a sender from the network connects");
        serial_keep_out(true);
        gen = serial_client_generation();
        CHECK(serial_drop_sender() && client_fd < 0 && serial_client_generation() == gen + 1,
              "keeping senders out drops the one connected, as a disconnect");
        CHECK(strstr(received(net, 500), "[MSG:") != NULL, "which reads why");
        close(net);
    } else
        printf("  skipped: this host has no address but the loopback\n");
    serial_keep_out(false);
    serial_set_listen_fd(-1);
    close(lany);

    /* --- a sender line is open until its status ------------------------- */
    printf("A sender line is open until its status:\n");
    status_message_chain = fake_status;
    serialRxFlush();
    push("G4 P1");
    CHECK(serial_sender_midline() && !serial_sender_line_open(), "a line being written is mid-line, not yet open");
    push("\n");
    CHECK(!serial_sender_midline() && !serial_sender_line_open(), "a whole line the core has not read is neither");
    int16_t ch;
    while((ch = serialGetC()) != SERIAL_NO_DATA && ch != '\n')
        CHECK(serial_sender_line_open(), "each byte the core reads keeps it open");
    CHECK(ch == '\n' && serial_sender_line_open(), "read whole and running, it is open");
    inject_status_message(Status_OK);
    CHECK(!serial_sender_line_open(), "its status closes it");
    push("\n");
    while((ch = serialGetC()) != SERIAL_NO_DATA)
        ;
    CHECK(!serial_sender_line_open(), "an empty line opens nothing");
    push("G1 X1\n");
    (void)serialGetC();
    CHECK(serial_sender_line_open(), "a line the core began is open");
    serialRxFlush();
    CHECK(!serial_sender_line_open(), "a flush closes a line that will get no status");

    /* --- the TX ring and its stall bound --------------------------------- */
    printf("The TX ring under a sender that stops reading:\n");

    /* The sender is a pipe with a 4 KiB capacity, already full: every
       byte from here on waits in the ring for the reader. */
    int pfd[2];
    CHECK(pipe2(pfd, O_NONBLOCK | O_CLOEXEC) == 0 && fcntl(pfd[1], F_SETPIPE_SZ, 4096) >= 0,
          "a pipe stands in for the sender's socket");
    {
        char fill[4096];
        memset(fill, 'f', sizeof(fill));
        while(write(pfd[1], fill, sizeof(fill)) > 0)
            ;
    }
    peer_rd = pfd[0];
    client_fd = pfd[1];
    txbuffer.head = txbuffer.tail = 0;

    /* A $$ dump is about 750 bytes on the machine; twice that queues
       without a single wait on the sender. */
    cb_calls = 0;
    peer_reads = false;
    CHECK(put_bytes(1500) == 1500 && cb_calls == 0 && serialTxCount() == 1500,
          "a 1500-byte report queues in the ring without waiting on the sender");

    /* The ring fills; a sender that resumes reading 900 ms later keeps
       its connection and every byte. */
    cb_calls = 0;
    peer_reads = true;
    peer_after_s = 0.9;
    int accepted = put_bytes(1200);
    double waited = now_s() - blocked_at;
    CHECK(accepted == 1200 && client_fd == pfd[1], "a 900 ms pause in the sender's reading drops nothing");
    CHECK(cb_calls > 0 && waited >= 0.9 && waited < 1.5,
          "the write waited for the sender, in the blocking callback");

    /* Drain everything the reader can take, then fill the ring again
       with a sender that reads only after 1100 ms: the bound drops it at
       one second, with the ring flushed. */
    {
        char buf[512];
        while(read(pfd[0], buf, sizeof(buf)) > 0)
            serial_poll();
        while(write(pfd[1], buf, sizeof(buf)) > 0)
            ;
    }
    cb_calls = 0;
    peer_after_s = 1.1;
    put_bytes(TX_BUFFER_SIZE);                      /* fills, then blocks on the last byte */
    waited = now_s() - blocked_at;
    CHECK(client_fd < 0, "a sender that pauses past the bound is dropped");
    CHECK(waited >= 1.0 && waited < 1.1, "the drop comes at the one-second bound");
    CHECK(serialTxCount() == 1, "the ring is flushed with the drop; only the byte that waited remains");
    close(pfd[0]);

    /* --- the -b listen address ------------------------------------------- */
    printf("The -b listen address: an IPv6 literal as written, an IPv4 literal v4-mapped, a host name and a "
           "short dotted form refused:\n");
    struct in6_addr bound;
    static const uint8_t loop4[4] = { 127, 0, 0, 1 };
    CHECK(bind_addr_parse("::1", &bound) == 0 && memcmp(&bound, &in6addr_loopback, sizeof(bound)) == 0,
          "::1 parses to in6addr_loopback, as written");
    memset(&bound, 0xa5, sizeof(bound));    /* a stale output: the mapped form is written whole */
    CHECK(bind_addr_parse("127.0.0.1", &bound) == 0 && IN6_IS_ADDR_V4MAPPED(&bound) &&
          bound.s6_addr[10] == 0xff && bound.s6_addr[11] == 0xff && memcmp(&bound.s6_addr[12], loop4, 4) == 0,
          "127.0.0.1 parses to ::ffff:127.0.0.1");
    CHECK(bind_addr_parse("localhost", &bound) == -1,
          "a host name is refused: the option takes a literal, never a name");
    CHECK(bind_addr_parse("1.2.3", &bound) == -1,
          "a short dotted form is refused: an IPv4 literal is the dotted quad");

    /* --- the start notice's lists ---------------------------------------- */
    printf("The start notice's lists of environment overrides and test hooks, held to the sources:\n");
    static env_name_t hooks[ENV_NAMES];         /* TEST_HOOKS, read from main.c */
    int n_hooks = 0;
    {
        static env_name_t reads[ENV_NAMES], gated[ENV_NAMES], listed[ENV_NAMES];
        /* The exempt arguments: the notice's two loops, and the gate's
           own read and its definition in glowforge_io.h. */
        static const char *const getenv_own[] = { "ENV_OVERRIDES[i])", "hook)", NULL };
        static const char *const gate_own[] = { "TEST_HOOKS[i])", "const char *hook)", NULL };
        int n_reads = 0, n_gated = 0, n_listed = 0, files = 0, unnamed = 0, gate_unnamed = 0;
        int differ = 0, hook_differ = 0;
        DIR *dir = opendir(DRIVER_SRC);
        struct dirent *e;
        while(dir && (e = readdir(dir)) != NULL) {
            size_t l = strlen(e->d_name);
            if(l < 3 || e->d_name[l - 2] != '.' || (e->d_name[l - 1] != 'c' && e->d_name[l - 1] != 'h'))
                continue;
            char path[1024];
            snprintf(path, sizeof(path), "%s/%s", DRIVER_SRC, e->d_name);
            char *text = env_slurp(path);
            files += text != NULL;
            unnamed += env_calls(text, "getenv", getenv_own, reads, &n_reads);
            gate_unnamed += env_calls(text, "gfio_test_hook", gate_own, gated, &n_gated);
            free(text);
        }
        if(dir)
            closedir(dir);
        char *main_c = env_slurp(DRIVER_SRC "/main.c");
        bool listed_read = env_table(main_c, "ENV_OVERRIDES[] = {", "the list", listed, &n_listed, &differ);
        bool hooks_read = env_table(main_c, "TEST_HOOKS[] = {", "TEST_HOOKS", hooks, &n_hooks, &hook_differ);
        free(main_c);
        for(int i = 0; i < n_reads; i++)
            if(!env_spawned(reads[i]) && !env_has(listed, n_listed, reads[i])) {
                printf("  the sources read %s, which the list does not name\n", reads[i]);
                differ++;
            }
        for(int i = 0; i < n_listed; i++)
            if(env_spawned(listed[i]) || !env_has(reads, n_reads, listed[i])) {
                printf("  the list names %s, %s\n", listed[i],
                       env_spawned(listed[i]) ? "which the supervisor sets" : "which no source reads");
                differ++;
            }
        for(int i = 0; i < n_gated; i++)
            if(!env_has(hooks, n_hooks, gated[i])) {
                printf("  the sources read %s through the gate, which TEST_HOOKS does not name\n", gated[i]);
                hook_differ++;
            }
        for(int i = 0; i < n_hooks; i++)
            if(!env_has(gated, n_gated, hooks[i]) || env_has(reads, n_reads, hooks[i])) {
                printf("  TEST_HOOKS names %s, %s\n", hooks[i],
                       env_has(reads, n_reads, hooks[i]) ? "which a getenv reads past the gate"
                                                        : "which no source reads through the gate");
                hook_differ++;
            }
        CHECK(files > 10 && listed_read, "the driver sources and main.c's list were read");
        CHECK(unnamed == 0, "every getenv in the sources names its variable");
        CHECK(differ == 0,
              "the list names every variable the sources read, once, but the supervisor's three");
        CHECK(hooks_read && gate_unnamed == 0 && hook_differ == 0,
              "TEST_HOOKS names each variable that the sources read through the gate, once, and no "
              "getenv reads one");
    }

    /* --- the start notice ------------------------------------------------ */
    printf("The start notice names the overrides in force, names only:\n");
    {
        char prog[] = "grblHAL_glowforge";
        char *args[] = { prog, NULL };
        for(size_t i = 0; i < sizeof(ENV_OVERRIDES) / sizeof(*ENV_OVERRIDES); i++)
            unsetenv(ENV_OVERRIDES[i]);
        for(int i = 0; i < n_hooks; i++)
            unsetenv(hooks[i]);
        unsetenv("GFSINK");                                        /* a null-sink run */
        setenv("FFLOG_CONF", "/nonexistent/forgefirm.conf", 1);    /* the list's first */
        setenv("GFSINK_LEAD_MS", "5", 1);
        setenv("GF_VERDICT_FILE", "/nonexistent/verdict.json", 1); /* the test hooks' last */
        notices = 0;
        controller_main(1, args);
        CHECK(notices == 1 &&
              !strcmp(notice, NOTICE " FFLOG_CONF GFSINK_LEAD_MS GF_VERDICT_FILE"),
              "the controller logs one notice naming the overrides that are set, and no value");
        /* With GFSINK set, as on a machine, the controller honors no test
           hook: GF_STATE_DIR and GF_VERDICT_FILE are set, and not in force. */
        setenv("GFSINK", "/dev/glowforge", 1);
        setenv("GF_STATE_DIR", "/nonexistent/state", 1);
        notices = 0;
        controller_main(1, args);
        CHECK(notices == 1 && !strcmp(notice, NOTICE " FFLOG_CONF GFSINK_LEAD_MS"),
              "with GFSINK set, the notice names no test hook: the controller honors none");
        unsetenv("GFSINK");
        for(size_t i = 0; i < sizeof(ENV_OVERRIDES) / sizeof(*ENV_OVERRIDES); i++)
            unsetenv(ENV_OVERRIDES[i]);
        for(int i = 0; i < n_hooks; i++)
            unsetenv(hooks[i]);
        notices = 0;
        controller_main(1, args);
        CHECK(notices == 0, "with none set, it logs no notice");
    }

    printf(failures ? "FAIL: %d check(s) failed\n"
                    : "PASS: an overrun drops the overrunning line whole, keeps every earlier "
                      "line, passes real-time characters and is reported once; a connect is "
                      "welcomed from a top-level poll; the ring holds a settings dump and "
                      "drops a sender only after a second without progress; senders from the "
                      "network are kept out on demand and one on this host is not; a sender "
                      "line is open until its status\n",
           failures);
    return failures ? 1 : 0;
}
