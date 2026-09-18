"""슬롯 메뉴 TUI와 주행 중 키 입력. 추가 패키지 없이 ANSI + termios."""

import atexit
import select
import sys

try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None

USE_COLOR = sys.stdout.isatty()


def _paint(code, text):
    if not USE_COLOR:
        return text
    return f"\033[{code}m{text}\033[0m"


def c_ok(text):
    return _paint("32", text)


def c_info(text):
    return _paint("36", text)


def c_warn(text):
    return _paint("33", text)


def c_err(text):
    return _paint("31;1", text)


def c_auto(text):
    return _paint("35;1", text)


def c_dim(text):
    return _paint("2", text)


def c_bold(text):
    return _paint("1", text)


def clear_screen():
    if sys.stdout.isatty():
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()


def draw_box(title, lines, width=58):
    inner = width - 2
    top = "┌" + "─" * inner + "┐"
    bot = "└" + "─" * inner + "┘"
    title_s = f" {title} "
    mid = max(0, inner - len(title_s))
    head = "│" + title_s + " " * mid
    head = head[:inner] + "│"

    def row(text):
        raw = _strip_ansi(text)
        pad = inner - len(raw) - 1
        if pad < 0:
            text = text[: inner - 1]
            pad = 0
        return "│ " + text + " " * pad + "│"

    out = [top, head, "│" + " " * inner + "│"]
    for line in lines:
        out.append(row(line))
    out.append(bot)
    return "\n".join(out)


def _strip_ansi(text):
    out = []
    i = 0
    while i < len(text):
        if text[i] == "\033":
            i += 1
            if i < len(text) and text[i] == "[":
                i += 1
                while i < len(text) and not text[i].isalpha():
                    i += 1
                i += 1
            continue
        out.append(text[i])
        i += 1
    return "".join(out)


def restore_terminal(fd=None, saved=None):
    """메뉴로 돌아갈 때 줄 입력과 키 에코를 강제로 복구한다."""
    if termios is None:
        return
    try:
        fd = sys.stdin.fileno() if fd is None else fd
        attrs = list(saved) if saved is not None else termios.tcgetattr(fd)
        attrs[0] |= termios.ICRNL
        attrs[1] |= termios.OPOST
        attrs[3] |= termios.ECHO | termios.ICANON | termios.ISIG
        attrs[6][termios.VMIN] = 1
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
    except (OSError, ValueError, termios.error):
        pass


def prompt(msg):
    restore_terminal()
    try:
        return input(msg)
    except EOFError:
        return ""


class Keys:
    def __init__(self):
        if termios is None or tty is None:
            raise RuntimeError("이 주행 키 입력은 Linux(파이)에서만 됩니다")
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        self.closed = False
        tty.setcbreak(self.fd)
        atexit.register(self.close)

    def read(self, timeout):
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return None
        ch = sys.stdin.read(1)
        if ch != "\x1b":
            return ch
        ready, _, _ = select.select([sys.stdin], [], [], 0.03)
        if not ready:
            return ch
        mid = sys.stdin.read(1)
        if mid not in ("[", "O"):
            return ch
        ready, _, _ = select.select([sys.stdin], [], [], 0.03)
        if not ready:
            return ch
        end = sys.stdin.read(1)
        if end == "A":
            return "up"
        if end == "B":
            return "down"
        if end == "C":
            return "right"
        if end == "D":
            return "left"
        return ch

    def close(self):
        if self.closed:
            return
        self.closed = True
        restore_terminal(self.fd, self.old)
