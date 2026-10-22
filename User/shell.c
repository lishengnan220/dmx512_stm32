/**
  ******************************************************************************
  * @file    shell.c
  * @brief   Interactive debug command line - implementation.
  *
  * Command set (case-insensitive, values decimal or 0x hex):
  *   help                         this help
  *   info                         firmware / mapping info
  *   echo <text...>               echo back the rest of the line
  *   set  <seg> <r> <g> <b> <w>   set one segment (seg 0 = broadcast all)
  *   all  <r> <g> <b> <w>         broadcast RGBW to every segment
  *   ch   <channel> <value>       write a single DMX channel (1..512)
  *   off                          switch every segment off
  *   status                       show system state + segment 1 data
  *   test                         run a red/green/blue/white/off animation
  *
  * The animation is advanced from Shell_Poll(), which runs once per main-loop
  * iteration (once per DMX frame, ~44 Hz).
  ******************************************************************************
  */

#include "shell.h"
#include "debug.h"
#include "dmx512.h"
#include "status.h"

/* ============================ RX ring buffer ============================ */
#define SHELL_RX_SIZE        64u
static volatile uint8_t shell_rx_buf[SHELL_RX_SIZE];
static volatile uint8_t shell_rx_head = 0u;
static volatile uint8_t shell_rx_tail = 0u;

/* ============================ Line buffer ============================ */
#define SHELL_LINE_MAX       64u
static char    shell_line[SHELL_LINE_MAX];
static uint8_t shell_line_len = 0u;

/* ============================ Test animation ============================ */
#define SHELL_TEST_COUNT     5u     /* red, green, blue, white, off */
#define SHELL_TEST_TICKS     20u    /* ~20 frames x 22.7 ms ~ 0.45 s per step */
static const uint8_t shell_test_color[SHELL_TEST_COUNT][4] =
{
    { 255u,   0u,   0u,   0u },    /* red    */
    {   0u, 255u,   0u,   0u },    /* green  */
    {   0u,   0u, 255u,   0u },    /* blue   */
    {   0u,   0u,   0u, 255u },    /* white  */
    {   0u,   0u,   0u,   0u }     /* off    */
};
static uint8_t  shell_test_running = 0u;
static uint8_t  shell_test_step    = 0u;
static uint16_t shell_test_tick    = 0u;

/* ============================ Helpers ============================ */

/* Case-insensitive string equality. */
static uint8_t Shell_StrEqCi(const char *a, const char *b)
{
    while ((*a != '\0') && (*b != '\0'))
    {
        char ca = *a;
        char cb = *b;
        if ((ca >= 'a') && (ca <= 'z')) { ca = (char)(ca - ('a' - 'A')); }
        if ((cb >= 'a') && (cb <= 'z')) { cb = (char)(cb - ('a' - 'A')); }
        if (ca != cb) { return 0u; }
        a++;
        b++;
    }
    return (*a == *b) ? 1u : 0u;
}

/* Parse a decimal or 0x-hex number into *out. Returns 1 on success. */
static uint8_t Shell_ParseNum(const char *s, uint16_t *out)
{
    uint16_t v      = 0u;
    uint8_t  digits = 0u;
    uint8_t  base   = 10u;

    if ((s[0] == '0') && ((s[1] == 'x') || (s[1] == 'X')))
    {
        base = 16u;
        s += 2;
    }

    while (*s != '\0')
    {
        uint8_t d;
        char    c = *s;

        if      ((c >= '0') && (c <= '9')) { d = (uint8_t)(c - '0'); }
        else if ((c >= 'a') && (c <= 'f')) { d = (uint8_t)(c - 'a' + 10); }
        else if ((c >= 'A') && (c <= 'F')) { d = (uint8_t)(c - 'A' + 10); }
        else { return 0u; }

        if (d >= base) { return 0u; }

        v = (uint16_t)(v * base + d);
        digits++;
        s++;
    }

    if (digits == 0u) { return 0u; }
    *out = v;
    return 1u;
}

/* ============================ Command handlers ============================ */

static void Shell_CmdHelp(void)
{
    DBG_STR("Commands:\r\n");
    DBG_STR("  help                     this help\r\n");
    DBG_STR("  info                     firmware / hardware mapping\r\n");
    DBG_STR("  echo <text...>           echo back the rest of the line\r\n");
    DBG_STR("  set  <seg> <r> <g> <b> <w>  set segment (seg 0 = all), RGBW 0-255\r\n");
    DBG_STR("  all  <r> <g> <b> <w>     broadcast RGBW to all segments\r\n");
    DBG_STR("  ch   <channel> <value>   write single channel (1-512)\r\n");
    DBG_STR("  off                      switch all segments off\r\n");
    DBG_STR("  status                   show state + segment 1 data\r\n");
    DBG_STR("  test                     run RGBW self-test animation\r\n");
}

static void Shell_CmdInfo(void)
{
    DBG_STR("DMX512 RGBW Light Controller\r\n");
    DBG_STR("  MCU       : STM32F103VE @ 72 MHz\r\n");
    DBG_STR("  Segments  : "); DBG_SendDecU16(DMX512_SEGMENT_COUNT);
    DBG_STR("  (RGBW, "); DBG_SendDecU16(DMX512_SEGMENT_CHANNEL); DBG_STR(" ch each)\r\n");
    DBG_STR("  Channels  : "); DBG_SendDecU16(DMX512_CHANNEL_NUM); DBG_STR("\r\n");
    DBG_STR("  DMX UART  : USART2 PA2, 250 kbps 8N2 (dir PA1)\r\n");
    DBG_STR("  Console   : USART3 PB10/PB11 (USB-TTL), 115200 8N1\r\n");
    DBG_STR("            :  host 7-byte frame + shell commands\r\n");
}

static void Shell_CmdEcho(uint8_t argc, char **argv)
{
    uint8_t i;
    DBG_STR("echo:");
    for (i = 1u; i < argc; i++)
    {
        DBG_STR(" ");
        DBG_STR(argv[i]);
    }
    DBG_STR("\r\n");
}

/* Stop any running animation and apply RGBW to a segment. */
static void Shell_ApplyRGBW(uint8_t seg, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    shell_test_running = 0u;
    DMX512_SetSegment(seg, r, g, b, w);

    DBG_STR("OK seg="); DBG_SendDecU16(seg);
    DBG_STR(" R="); DBG_SendDecU16(r);
    DBG_STR(" G="); DBG_SendDecU16(g);
    DBG_STR(" B="); DBG_SendDecU16(b);
    DBG_STR(" W="); DBG_SendDecU16(w);
    DBG_STR("\r\n");

    Status_Set((seg == 0u) ? STATUS_BROADCAST : STATUS_SINGLE);
}

static void Shell_CmdSet(uint8_t argc, char **argv)
{
    uint16_t seg, r, g, b, w;

    if (argc != 6)
    {
        DBG_STR("usage: set <seg> <r> <g> <b> <w>\r\n");
        return;
    }
    if ((!Shell_ParseNum(argv[1], &seg)) || (seg > DMX512_SEGMENT_COUNT))
    {
        DBG_STR("bad segment (0 = all, 1.."); DBG_SendDecU16(DMX512_SEGMENT_COUNT); DBG_STR(")\r\n");
        return;
    }
    if ((!Shell_ParseNum(argv[2], &r)) || (r > 255u))
    {
        DBG_STR("bad R (0-255)\r\n");
        return;
    }
    if ((!Shell_ParseNum(argv[3], &g)) || (g > 255u))
    {
        DBG_STR("bad G (0-255)\r\n");
        return;
    }
    if ((!Shell_ParseNum(argv[4], &b)) || (b > 255u))
    {
        DBG_STR("bad B (0-255)\r\n");
        return;
    }
    if ((!Shell_ParseNum(argv[5], &w)) || (w > 255u))
    {
        DBG_STR("bad W (0-255)\r\n");
        return;
    }

    Shell_ApplyRGBW((uint8_t)seg, (uint8_t)r, (uint8_t)g, (uint8_t)b, (uint8_t)w);
}

static void Shell_CmdAll(uint8_t argc, char **argv)
{
    uint16_t r, g, b, w;

    if (argc != 5)
    {
        DBG_STR("usage: all <r> <g> <b> <w>\r\n");
        return;
    }
    if ((!Shell_ParseNum(argv[1], &r)) || (r > 255u)) { DBG_STR("bad R (0-255)\r\n"); return; }
    if ((!Shell_ParseNum(argv[2], &g)) || (g > 255u)) { DBG_STR("bad G (0-255)\r\n"); return; }
    if ((!Shell_ParseNum(argv[3], &b)) || (b > 255u)) { DBG_STR("bad B (0-255)\r\n"); return; }
    if ((!Shell_ParseNum(argv[4], &w)) || (w > 255u)) { DBG_STR("bad W (0-255)\r\n"); return; }

    Shell_ApplyRGBW(0u, (uint8_t)r, (uint8_t)g, (uint8_t)b, (uint8_t)w);
}

static void Shell_CmdCh(uint8_t argc, char **argv)
{
    uint16_t ch, val;

    if (argc != 3)
    {
        DBG_STR("usage: ch <channel> <value>\r\n");
        return;
    }
    if ((!Shell_ParseNum(argv[1], &ch)) || (ch < 1u) || (ch > DMX512_CHANNEL_NUM))
    {
        DBG_STR("bad channel (1.."); DBG_SendDecU16(DMX512_CHANNEL_NUM); DBG_STR(")\r\n");
        return;
    }
    if ((!Shell_ParseNum(argv[2], &val)) || (val > 255u))
    {
        DBG_STR("bad value (0-255)\r\n");
        return;
    }

    DMX512_SetChannel((uint16_t)ch, (uint8_t)val);
    DBG_STR("OK ch="); DBG_SendDecU16(ch);
    DBG_STR(" = "); DBG_SendDecU16(val);
    DBG_STR("\r\n");
}

static void Shell_CmdOff(void)
{
    uint8_t s;

    shell_test_running = 0u;
    for (s = 1u; s <= DMX512_SEGMENT_COUNT; s++)
    {
        DMX512_SetSegment(s, 0u, 0u, 0u, 0u);
    }
    DBG_STR("OK all off\r\n");
    Status_Set(STATUS_RUNNING);
}

static void Shell_CmdStatus(void)
{
    const char *name;

    switch (Status_Get())
    {
        case STATUS_BOOT:      name = "BOOT";      break;
        case STATUS_RUNNING:   name = "RUNNING";   break;
        case STATUS_SINGLE:    name = "SINGLE";    break;
        case STATUS_BROADCAST: name = "BROADCAST"; break;
        case STATUS_ERROR:     name = "ERROR";     break;
        default:               name = "UNKNOWN";   break;
    }

    DBG_STR("state: "); DBG_STR(name); DBG_STR("\r\n");
    DBG_STR("seg1: R="); DBG_SendDecU16(DMX512_GetChannel(1u));
    DBG_STR(" G="); DBG_SendDecU16(DMX512_GetChannel(2u));
    DBG_STR(" B="); DBG_SendDecU16(DMX512_GetChannel(3u));
    DBG_STR(" W="); DBG_SendDecU16(DMX512_GetChannel(4u));
    DBG_STR("\r\n");
}

static void Shell_CmdTest(void)
{
    shell_test_running = 1u;
    shell_test_step    = 0u;
    shell_test_tick    = 0u;
    DBG_STR("OK self-test started (red -> green -> blue -> white -> off)\r\n");
}

/* Advance the test animation by one main-loop tick (~22.7 ms). */
static void Shell_TestTick(void)
{
    if (shell_test_running == 0u)
    {
        return;
    }

    if (shell_test_tick < SHELL_TEST_TICKS)
    {
        shell_test_tick++;
        return;
    }
    shell_test_tick = 0u;

    if (shell_test_step < SHELL_TEST_COUNT)
    {
        DMX512_SetSegment(0u,
                          shell_test_color[shell_test_step][0],
                          shell_test_color[shell_test_step][1],
                          shell_test_color[shell_test_step][2],
                          shell_test_color[shell_test_step][3]);

        DBG_STR("test step "); DBG_SendDecU16(shell_test_step);
        DBG_STR(": R="); DBG_SendDecU16(shell_test_color[shell_test_step][0]);
        DBG_STR(" G="); DBG_SendDecU16(shell_test_color[shell_test_step][1]);
        DBG_STR(" B="); DBG_SendDecU16(shell_test_color[shell_test_step][2]);
        DBG_STR(" W="); DBG_SendDecU16(shell_test_color[shell_test_step][3]);
        DBG_STR("\r\n");

        Status_Set(STATUS_BROADCAST);
        shell_test_step++;
    }
    else
    {
        shell_test_running = 0u;
        DMX512_SetSegment(0u, 0u, 0u, 0u, 0u);
        DBG_STR("test done\r\n");
    }
}

/* Parse and execute one NUL-terminated command line. */
static void Shell_Exec(char *line)
{
    char   *argv[8];
    uint8_t argc = 0u;
    char   *p = line;

    /* Tokenize in place: split on spaces/tabs, collect argv pointers. */
    while (*p != '\0')
    {
        while ((*p == ' ') || (*p == '\t'))
        {
            *p = '\0';
            p++;
        }
        if (*p == '\0')
        {
            break;
        }
        if (argc < 8u)
        {
            argv[argc] = p;
            argc++;
        }
        while ((*p != '\0') && (*p != ' ') && (*p != '\t'))
        {
            p++;
        }
    }

    if (argc == 0u)
    {
        return;
    }

    if      (Shell_StrEqCi(argv[0], "help")   || Shell_StrEqCi(argv[0], "?"))      { Shell_CmdHelp(); }
    else if (Shell_StrEqCi(argv[0], "info"))                                        { Shell_CmdInfo(); }
    else if (Shell_StrEqCi(argv[0], "echo"))                                        { Shell_CmdEcho(argc, argv); }
    else if (Shell_StrEqCi(argv[0], "set"))                                         { Shell_CmdSet(argc, argv); }
    else if (Shell_StrEqCi(argv[0], "all"))                                         { Shell_CmdAll(argc, argv); }
    else if (Shell_StrEqCi(argv[0], "ch"))                                          { Shell_CmdCh(argc, argv); }
    else if (Shell_StrEqCi(argv[0], "off"))                                         { Shell_CmdOff(); }
    else if (Shell_StrEqCi(argv[0], "status"))                                      { Shell_CmdStatus(); }
    else if (Shell_StrEqCi(argv[0], "test"))                                        { Shell_CmdTest(); }
    else
    {
        DBG_STR("unknown cmd '"); DBG_STR(argv[0]); DBG_STR("' (type 'help')\r\n");
    }
}

/* ============================ Public interface ============================ */

void Shell_Init(void)
{
    shell_rx_head  = 0u;
    shell_rx_tail  = 0u;
    shell_line_len = 0u;
    shell_test_running = 0u;

    DBG_STR("\r\nDMX512 shell ready. Type 'help'.\r\n> ");
}

void Shell_RxByte(uint8_t byte)
{
    uint8_t next = (uint8_t)((shell_rx_head + 1u) % SHELL_RX_SIZE);

    if (next != shell_rx_tail)   /* drop bytes only when full */
    {
        shell_rx_buf[shell_rx_head] = byte;
        shell_rx_head = next;
    }
}

void Shell_Poll(void)
{
    /* 1. Drain the ring buffer into the line buffer, echo as we go. */
    while (shell_rx_tail != shell_rx_head)
    {
        uint8_t c = shell_rx_buf[shell_rx_tail];
        shell_rx_tail = (uint8_t)((shell_rx_tail + 1u) % SHELL_RX_SIZE);

        if ((c == '\r') || (c == '\n'))
        {
            if (shell_line_len > 0u)
            {
                shell_line[shell_line_len] = '\0';
                DBG_STR("\r\n");
                Shell_Exec(shell_line);
                shell_line_len = 0u;
                DBG_STR("> ");
            }
        }
        else if (shell_line_len < (SHELL_LINE_MAX - 1u))
        {
            shell_line[shell_line_len] = (char)c;
            shell_line_len++;
            Debug_SendByte(c);          /* echo the typed character */
        }
    }

    /* 2. Advance the self-test animation if it is running. */
    Shell_TestTick();
}
