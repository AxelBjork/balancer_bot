#!/usr/bin/env python3
"""A small HTTP-to-HDPlayer bridge for the currently open text program.

HDPlayer's text editor is a custom ``TRichViewEdit`` control, so ordinary
UI-Automation text setters do not work. This bridge uses the native paste
messages supported by that editor and triggers Send through HDPlayer's Qt
child window.

Examples:
    # Safe API test; no HDPlayer input or device transfer happens.
    .\.venv\Scripts\python.exe hdplayer_automation_server.py --dry-run

    # Run this command from an elevated PowerShell when HDPlayer is elevated.
    .\.venv\Scripts\python.exe hdplayer_automation_server.py --send

    curl.exe -X PUT http://127.0.0.1:8765/message -H "Content-Type: text/plain" --data "HELLO"
    curl.exe http://127.0.0.1:8765/health

The default listener accepts connections on all local IPv4 interfaces,
including the browser editor from another LAN device.
"""

from __future__ import annotations

import argparse
import ctypes
import html
import ipaddress
import json
import socket
import sys
import threading
import time
import urllib.parse
from ctypes import wintypes
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Type


MAX_TEXT_BYTES = 8192
DEFAULT_EDITOR_CLASS = "TRichViewEdit"
DESKTOP_READOBJECTS = 0x0001
DESKTOP_ENUMERATE = 0x0040
DESKTOP_SWITCHDESKTOP = 0x0100
GA_ROOT = 2
SW_RESTORE = 9
CF_UNICODETEXT = 13
GMEM_MOVEABLE = 0x0002
EM_SETSEL = 0x00B1
WM_PASTE = 0x0302
SMTO_BLOCK = 0x0001
SMTO_ABORTIFHUNG = 0x0002
WM_CLOSE = 0x0010
WM_LBUTTONDOWN = 0x0201
WM_LBUTTONUP = 0x0202
MK_LBUTTON = 0x0001

user32 = ctypes.WinDLL("user32", use_last_error=True)
kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
shell32 = ctypes.WinDLL("shell32", use_last_error=True)
WNDENUMPROC = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)

user32.OpenInputDesktop.argtypes = (wintypes.DWORD, wintypes.BOOL, wintypes.DWORD)
user32.OpenInputDesktop.restype = wintypes.HANDLE
user32.SetThreadDesktop.argtypes = (wintypes.HANDLE,)
user32.SetThreadDesktop.restype = wintypes.BOOL
user32.EnumWindows.argtypes = (WNDENUMPROC, wintypes.LPARAM)
user32.EnumWindows.restype = wintypes.BOOL
user32.EnumChildWindows.argtypes = (wintypes.HWND, WNDENUMPROC, wintypes.LPARAM)
user32.EnumChildWindows.restype = wintypes.BOOL
user32.GetClassNameW.argtypes = (wintypes.HWND, wintypes.LPWSTR, ctypes.c_int)
user32.GetClassNameW.restype = ctypes.c_int
user32.GetWindowTextW.argtypes = (wintypes.HWND, wintypes.LPWSTR, ctypes.c_int)
user32.GetWindowTextW.restype = ctypes.c_int
user32.IsWindowVisible.argtypes = (wintypes.HWND,)
user32.IsWindowVisible.restype = wintypes.BOOL
user32.PostMessageW.argtypes = (
    wintypes.HWND,
    wintypes.UINT,
    wintypes.WPARAM,
    wintypes.LPARAM,
)
user32.PostMessageW.restype = wintypes.BOOL
user32.IsIconic.argtypes = (wintypes.HWND,)
user32.IsIconic.restype = wintypes.BOOL
user32.GetWindowRect.argtypes = (wintypes.HWND, ctypes.POINTER(wintypes.RECT))
user32.GetWindowRect.restype = wintypes.BOOL
user32.GetAncestor.argtypes = (wintypes.HWND, wintypes.UINT)
user32.GetAncestor.restype = wintypes.HWND
user32.ShowWindowAsync.argtypes = (wintypes.HWND, ctypes.c_int)
user32.ShowWindowAsync.restype = wintypes.BOOL
user32.SetForegroundWindow.argtypes = (wintypes.HWND,)
user32.SetForegroundWindow.restype = wintypes.BOOL
user32.GetForegroundWindow.restype = wintypes.HWND
user32.GetWindowThreadProcessId.argtypes = (wintypes.HWND, ctypes.POINTER(wintypes.DWORD))
user32.GetWindowThreadProcessId.restype = wintypes.DWORD
user32.AttachThreadInput.argtypes = (wintypes.DWORD, wintypes.DWORD, wintypes.BOOL)
user32.AttachThreadInput.restype = wintypes.BOOL
user32.BringWindowToTop.argtypes = (wintypes.HWND,)
user32.BringWindowToTop.restype = wintypes.BOOL
user32.SetActiveWindow.argtypes = (wintypes.HWND,)
user32.SetActiveWindow.restype = wintypes.HWND
user32.OpenClipboard.argtypes = (wintypes.HWND,)
user32.OpenClipboard.restype = wintypes.BOOL
user32.CloseClipboard.restype = wintypes.BOOL
user32.EmptyClipboard.restype = wintypes.BOOL
user32.SetClipboardData.argtypes = (wintypes.UINT, wintypes.HANDLE)
user32.SetClipboardData.restype = wintypes.HANDLE
user32.SendMessageTimeoutW.argtypes = (
    wintypes.HWND,
    wintypes.UINT,
    wintypes.WPARAM,
    wintypes.LPARAM,
    wintypes.UINT,
    wintypes.UINT,
    ctypes.POINTER(ctypes.c_size_t),
)
user32.SendMessageTimeoutW.restype = wintypes.LPARAM

kernel32.GlobalAlloc.argtypes = (wintypes.UINT, ctypes.c_size_t)
kernel32.GlobalAlloc.restype = wintypes.HGLOBAL
kernel32.GlobalLock.argtypes = (wintypes.HGLOBAL,)
kernel32.GlobalLock.restype = wintypes.LPVOID
kernel32.GlobalUnlock.argtypes = (wintypes.HGLOBAL,)
kernel32.GlobalUnlock.restype = wintypes.BOOL
kernel32.GlobalFree.argtypes = (wintypes.HGLOBAL,)
kernel32.GlobalFree.restype = wintypes.HGLOBAL
kernel32.GetCurrentThreadId.restype = wintypes.DWORD


class HDPlayerError(RuntimeError):
    """An actionable HDPlayer automation failure."""


def last_error(action: str) -> HDPlayerError:
    number = ctypes.get_last_error()
    return HDPlayerError(f"{action} failed: {ctypes.FormatError(number).strip()} ({number})")


def switch_to_input_desktop() -> None:
    """Attach the calling HTTP thread to the user's active Windows desktop."""
    desktop = user32.OpenInputDesktop(
        0, False, DESKTOP_READOBJECTS | DESKTOP_ENUMERATE | DESKTOP_SWITCHDESKTOP
    )
    if not desktop:
        raise last_error("OpenInputDesktop")
    if not user32.SetThreadDesktop(desktop):
        raise last_error("SetThreadDesktop")


def window_class(handle: int) -> str:
    value = ctypes.create_unicode_buffer(256)
    user32.GetClassNameW(handle, value, len(value))
    return value.value


def window_title(handle: int) -> str:
    value = ctypes.create_unicode_buffer(256)
    user32.GetWindowTextW(handle, value, len(value))
    return value.value


def window_process_id(handle: int) -> int:
    process_id = wintypes.DWORD()
    user32.GetWindowThreadProcessId(handle, ctypes.byref(process_id))
    return process_id.value


def current_send_dialog(root: int) -> int | None:
    """Return HDPlayer's visible Send Project dialog, if any."""
    process_id = window_process_id(root)
    matches: list[int] = []

    @WNDENUMPROC
    def visit(handle: int, _: int) -> bool:
        if (
            user32.IsWindowVisible(handle)
            and window_process_id(handle) == process_id
            and window_title(handle) == "Send Project"
        ):
            matches.append(handle)
        return True

    user32.EnumWindows(visit, 0)
    return matches[0] if matches else None


def find_send_dialog(root: int, timeout: float = 5.0) -> int:
    """Wait for HDPlayer's top-level Send Project dialog."""
    deadline = time.monotonic() + timeout
    while True:
        dialog = current_send_dialog(root)
        if dialog:
            return dialog
        if time.monotonic() >= deadline:
            raise HDPlayerError("HDPlayer's Send Project dialog did not appear")
        time.sleep(0.05)


def close_existing_send_dialog(root: int) -> None:
    """Close a completed Send Project dialog before preparing the next update."""
    dialog = current_send_dialog(root)
    if not dialog:
        return
    if not user32.PostMessageW(dialog, WM_CLOSE, 0, 0):
        raise last_error("PostMessage(WM_CLOSE)")
    deadline = time.monotonic() + 3.0
    while current_send_dialog(root):
        if time.monotonic() >= deadline:
            raise HDPlayerError("The previous Send Project dialog did not close")
        time.sleep(0.05)
    # Give Qt time to release the modal state and return control to the toolbar.
    time.sleep(0.35)


def visible_area(handle: int) -> int:
    rect = wintypes.RECT()
    if not user32.GetWindowRect(handle, ctypes.byref(rect)):
        return 0
    return max(0, rect.right - rect.left) * max(0, rect.bottom - rect.top)


def find_editor(editor_class: str) -> tuple[int, int]:
    """Return (rich-editor handle, HDPlayer root window handle)."""
    candidates: list[int] = []

    @WNDENUMPROC
    def visit_child(handle: int, _: int) -> bool:
        if user32.IsWindowVisible(handle) and window_class(handle) == editor_class:
            candidates.append(handle)
        return True

    @WNDENUMPROC
    def visit_top_level(handle: int, _: int) -> bool:
        user32.EnumChildWindows(handle, visit_child, 0)
        return True

    user32.EnumWindows(visit_top_level, 0)
    if not candidates:
        raise HDPlayerError(
            f"No visible {editor_class!r} editor was found. Open the HDPlayer text item first."
        )
    editor = max(candidates, key=visible_area)
    root = user32.GetAncestor(editor, GA_ROOT)
    if not root:
        raise last_error("GetAncestor")
    return editor, root


def find_send_button_target(root: int) -> tuple[int, tuple[int, int]]:
    """Find the right-aligned Send segment in HDPlayer's custom Qt toolbar.

    The toolbar does not expose a button name to UI Automation, but it does
    expose native child rectangles. Send is the medium-width segment near the
    top-right; its position scales with the HDPlayer window.
    """
    root_rect = wintypes.RECT()
    if not user32.GetWindowRect(root, ctypes.byref(root_rect)):
        raise last_error("GetWindowRect")
    root_width = root_rect.right - root_rect.left
    if root_width <= 0:
        raise HDPlayerError("HDPlayer window has an invalid size")

    candidates: list[tuple[float, int, wintypes.RECT]] = []

    @WNDENUMPROC
    def visit(handle: int, _: int) -> bool:
        if not user32.IsWindowVisible(handle):
            return True
        rect = wintypes.RECT()
        if not user32.GetWindowRect(handle, ctypes.byref(rect)):
            return True
        width = rect.right - rect.left
        height = rect.bottom - rect.top
        centre_ratio = ((rect.left + rect.right) / 2 - root_rect.left) / root_width
        top_offset = rect.top - root_rect.top
        if 60 <= width <= 160 and 20 <= height <= 60 and 0.80 <= centre_ratio <= 0.93 and 35 <= top_offset <= 100:
            candidates.append((centre_ratio, handle, rect))
        return True

    user32.EnumChildWindows(root, visit, 0)
    if not candidates:
        raise HDPlayerError(
            "Could not locate HDPlayer's Send toolbar button. Keep the normal toolbar visible."
        )
    _, handle, button = min(candidates, key=lambda candidate: abs(candidate[0] - 0.89))
    point = ((button.left + button.right) // 2, (button.top + button.bottom) // 2)
    return handle, point


def trigger_send_button(root: int) -> None:
    """Click Send by messaging its Qt child window directly."""
    handle, point = find_send_button_target(root)
    rect = wintypes.RECT()
    if not user32.GetWindowRect(handle, ctypes.byref(rect)):
        raise last_error("GetWindowRect")
    x = point[0] - rect.left
    y = point[1] - rect.top
    position = (y << 16) | (x & 0xFFFF)
    if not user32.PostMessageW(handle, WM_LBUTTONDOWN, MK_LBUTTON, position):
        raise last_error("PostMessage(WM_LBUTTONDOWN)")
    if not user32.PostMessageW(handle, WM_LBUTTONUP, 0, position):
        raise last_error("PostMessage(WM_LBUTTONUP)")


def put_clipboard_text(text: str) -> None:
    """Put Unicode text on the clipboard. Windows owns the memory after success."""
    encoded = (text + "\0").encode("utf-16-le")
    memory = kernel32.GlobalAlloc(GMEM_MOVEABLE, len(encoded))
    if not memory:
        raise last_error("GlobalAlloc")
    pointer = kernel32.GlobalLock(memory)
    if not pointer:
        kernel32.GlobalFree(memory)
        raise last_error("GlobalLock")
    ctypes.memmove(pointer, encoded, len(encoded))
    kernel32.GlobalUnlock(memory)

    for _ in range(10):
        if user32.OpenClipboard(None):
            break
        time.sleep(0.05)
    else:
        kernel32.GlobalFree(memory)
        raise HDPlayerError("Clipboard is busy; close the application that currently owns it and retry.")
    try:
        if not user32.EmptyClipboard():
            kernel32.GlobalFree(memory)
            raise last_error("EmptyClipboard")
        if not user32.SetClipboardData(CF_UNICODETEXT, memory):
            kernel32.GlobalFree(memory)
            raise last_error("SetClipboardData")
        memory = None  # Windows now owns it.
    finally:
        user32.CloseClipboard()
        if memory:
            kernel32.GlobalFree(memory)


def replace_editor_text(editor: int, text: str) -> None:
    """Replace TRichViewEdit content using the native paste messages it supports."""
    put_clipboard_text(text)
    result = ctypes.c_size_t()
    user32.SendMessageTimeoutW(
        editor,
        EM_SETSEL,
        0,
        -1,
        SMTO_BLOCK | SMTO_ABORTIFHUNG,
        2000,
        ctypes.byref(result),
    )
    delivered = user32.SendMessageTimeoutW(
        editor,
        WM_PASTE,
        0,
        0,
        SMTO_BLOCK | SMTO_ABORTIFHUNG,
        2000,
        ctypes.byref(result),
    )
    if not delivered:
        raise last_error("WM_PASTE")
    time.sleep(0.15)


def bring_to_foreground(root: int) -> None:
    """Bring HDPlayer forward so a subsequent physical toolbar click is safe."""
    current_thread = kernel32.GetCurrentThreadId()
    target_thread = user32.GetWindowThreadProcessId(root, None)
    foreground_before = user32.GetForegroundWindow()
    foreground_thread = user32.GetWindowThreadProcessId(foreground_before, None) if foreground_before else 0
    attached: list[int] = []
    # A server's worker thread is not normally allowed to steal focus from the
    # interactive application. Joining its input queue briefly permits the
    # standard foreground/focus calls, then is immediately undone.
    for thread_id in {target_thread, foreground_thread} - {0, current_thread}:
        if user32.AttachThreadInput(current_thread, thread_id, True):
            attached.append(thread_id)
    try:
        # Restoring an already maximized Qt window changes its geometry, which
        # invalidates toolbar positions. Restore only if it is actually minimized.
        if user32.IsIconic(root):
            user32.ShowWindowAsync(root, SW_RESTORE)
        user32.BringWindowToTop(root)
        user32.SetForegroundWindow(root)
        user32.SetActiveWindow(root)
        time.sleep(0.15)
        foreground = user32.GetForegroundWindow()
        # Qt may make an owned popup/root window foreground instead of precisely
        # the ancestor returned for TRichViewEdit. It is still safe when it belongs
        # to the same HDPlayer process.
        if foreground != root and window_process_id(foreground) != window_process_id(root):
            raise HDPlayerError(
                "HDPlayer could not be brought to the foreground "
                f"(root=0x{root:X}, foreground=0x{foreground:X})."
            )
    finally:
        for thread_id in attached:
            user32.AttachThreadInput(current_thread, thread_id, False)


def is_administrator() -> bool:
    return bool(shell32.IsUserAnAdmin())


def local_network_ipv4_addresses() -> list[str]:
    """Return likely LAN IPv4 addresses, with the default-route address first."""
    addresses: list[str] = []
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("192.0.2.1", 80))
            addresses.append(probe.getsockname()[0])
    except OSError:
        pass

    try:
        hostname_addresses = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
    except socket.gaierror:
        hostname_addresses = []
    for address_info in hostname_addresses:
        address = address_info[4][0]
        if address not in addresses:
            addresses.append(address)

    return [
        address
        for address in addresses
        if not ipaddress.ip_address(address).is_loopback
        and not ipaddress.ip_address(address).is_unspecified
    ]


@dataclass
class State:
    dry_run: bool
    send_after_update: bool
    editor_class: str
    token: str | None
    message: str = ""
    last_error: str | None = None
    sent: bool = False
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update(self, text: str, *, send: bool | None = None) -> None:
        if len(text.encode("utf-8")) > MAX_TEXT_BYTES:
            raise ValueError(f"text is limited to {MAX_TEXT_BYTES} UTF-8 bytes")
        should_send = self.send_after_update if send is None else send
        with self.lock:
            self.last_error = None
            self.sent = False
            if self.dry_run:
                self.message = text
                self.sent = should_send
                return
            switch_to_input_desktop()
            editor, root = find_editor(self.editor_class)
            close_existing_send_dialog(root)
            replace_editor_text(editor, text)
            if should_send:
                bring_to_foreground(root)
                trigger_send_button(root)
                # Opening Send Project starts the transfer automatically.
                find_send_dialog(root)
                self.sent = True
            self.message = text


def json_body(handler: BaseHTTPRequestHandler) -> dict[str, object]:
    try:
        length = int(handler.headers.get("Content-Length", ""))
    except ValueError as error:
        raise ValueError("Content-Length is required") from error
    if not 0 <= length <= MAX_TEXT_BYTES + 256:
        raise ValueError("request body is too large")
    raw = handler.rfile.read(length)
    content_type = handler.headers.get("Content-Type", "").split(";", 1)[0].lower()
    if content_type == "application/json":
        try:
            value = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ValueError("body must be valid UTF-8 JSON") from error
        if not isinstance(value, dict) or not isinstance(value.get("text"), str):
            raise ValueError('JSON body must be an object such as {"text":"HELLO"}')
        return value
    try:
        return {"text": raw.decode("utf-8")}
    except UnicodeDecodeError as error:
        raise ValueError("body must be UTF-8 text") from error


def form_text(handler: BaseHTTPRequestHandler) -> str:
    """Read the text field from the browser editor form."""
    try:
        length = int(handler.headers.get("Content-Length", ""))
    except ValueError as error:
        raise ValueError("Content-Length is required") from error
    # URL encoding can use up to three bytes per UTF-8 input byte.
    if not 0 <= length <= MAX_TEXT_BYTES * 3 + 256:
        raise ValueError("request body is too large")
    try:
        fields = urllib.parse.parse_qs(
            handler.rfile.read(length).decode("utf-8"), keep_blank_values=True
        )
    except UnicodeDecodeError as error:
        raise ValueError("form must be UTF-8") from error
    return fields.get("text", [""])[0]


def make_handler(state: State) -> Type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def authorized(self) -> bool:
            if state.token is None:
                return True
            return self.headers.get("Authorization") == f"Bearer {state.token}"

        def send_json(self, status: HTTPStatus, data: dict[str, object]) -> None:
            body = json.dumps(data, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def send_editor(self, status: HTTPStatus = HTTPStatus.OK, error: str | None = None) -> None:
            text = html.escape(state.message)
            status_message = (
                '<p class="success">Saved and sent to HDPlayer.</p>'
                if self.path == "/edit?saved=1"
                else ""
            )
            if error:
                status_message = f'<p class="error">{html.escape(error)}</p>'
            send_note = "and click Send" if state.send_after_update else "without clicking Send"
            page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>HDPlayer text</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{{font:16px system-ui,sans-serif;max-width:46rem;margin:2rem auto;padding:0 1rem}}
textarea{{box-sizing:border-box;width:100%;min-height:12rem;font:inherit;padding:.7rem}}
button{{font:inherit;padding:.6rem 1rem}}.success{{color:#087a2d}}.error{{color:#b00020}}</style></head>
<body><h1>HDPlayer text</h1>
<p>Saving will replace the open HDPlayer text item {send_note}.</p>{status_message}
<form method="post" action="/edit"><textarea name="text" autofocus>{text}</textarea>
<p><button type="submit">Save to HDPlayer</button></p></form></body></html>""".encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(page)

        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/health":
                self.send_json(
                    HTTPStatus.OK,
                    {
                        "ok": state.last_error is None,
                        "dry_run": state.dry_run,
                        "elevated": is_administrator(),
                        "send_after_update": state.send_after_update,
                        "last_error": state.last_error,
                    },
                )
            elif self.path == "/message":
                self.send_json(HTTPStatus.OK, {"text": state.message, "sent": state.sent})
            elif self.path == "/edit" or self.path == "/edit?saved=1":
                self.send_editor()
            else:
                self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            if self.path == "/edit":
                self.update_editor()
            else:
                self.update_message()

        def do_PUT(self) -> None:  # noqa: N802
            self.update_message()

        def update_message(self) -> None:
            if self.path != "/message":
                self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})
                return
            if not self.authorized():
                self.send_json(HTTPStatus.UNAUTHORIZED, {"error": "missing or invalid bearer token"})
                return
            try:
                value = json_body(self)
                text = value["text"]
                assert isinstance(text, str)
                send = value.get("send")
                if send is not None and not isinstance(send, bool):
                    raise ValueError("JSON field 'send' must be true or false")
                self.apply_update(text, send=send)
            except ValueError as error:
                self.send_json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
                return
            except HDPlayerError as error:
                self.send_json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": str(error)})
                return
            self.send_json(HTTPStatus.OK, {"ok": True, "text": text, "sent": state.sent})

        def update_editor(self) -> None:
            if not self.authorized():
                self.send_editor(HTTPStatus.UNAUTHORIZED, "Missing or invalid bearer token.")
                return
            try:
                self.apply_update(form_text(self))
            except ValueError as error:
                self.send_editor(HTTPStatus.BAD_REQUEST, str(error))
                return
            except HDPlayerError as error:
                self.send_editor(HTTPStatus.SERVICE_UNAVAILABLE, str(error))
                return
            self.send_response(HTTPStatus.SEE_OTHER)
            self.send_header("Location", "/edit?saved=1")
            self.send_header("Content-Length", "0")
            self.end_headers()

        def apply_update(self, text: str, *, send: bool | None = None) -> None:
            try:
                state.update(text, send=send)
            except HDPlayerError as error:
                state.last_error = str(error)
                raise

        def log_message(self, format: str, *args: object) -> None:
            print(f"{self.client_address[0]} - {format % args}")

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve HTTP updates into the open HDPlayer text editor.")
    parser.add_argument("--host", default="0.0.0.0", help="address to listen on (default: all IPv4 interfaces)")
    parser.add_argument("--port", type=int, default=8765, help="HTTP port (default: 8765)")
    parser.add_argument("--token", help="optional Bearer token for write requests")
    parser.add_argument("--dry-run", action="store_true", help="accept requests but do not control HDPlayer")
    parser.add_argument("--send", action="store_true", help="click HDPlayer's Send button after each update")
    parser.add_argument("--editor-class", default=DEFAULT_EDITOR_CLASS, help="native text editor window class")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    return args


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8", errors="backslashreplace")
    args = parse_args()
    if not args.dry_run and not is_administrator():
        print(
            "Refusing live automation because this process is not elevated. "
            "HDPlayer is elevated on this PC; start PowerShell as Administrator, then rerun with --dry-run omitted.",
            file=sys.stderr,
        )
        return 2
    state = State(
        dry_run=args.dry_run,
        send_after_update=args.send,
        editor_class=args.editor_class,
        token=args.token,
    )
    server = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    if args.host == "0.0.0.0":
        addresses = local_network_ipv4_addresses()
        print("HDPlayer automation editor available at:")
        print(f"  http://127.0.0.1:{args.port}/edit")
        for address in addresses:
            print(f"  http://{address}:{args.port}/edit")
        if not args.token:
            print("LAN access is unauthenticated; anyone on this network can update the sign.")
    else:
        print(f"HDPlayer automation editor: http://{args.host}:{args.port}/edit")
    if args.dry_run:
        print("Dry-run enabled: HDPlayer will not be changed.")
    elif args.send:
        print("Each update will trigger HDPlayer's detected Send toolbar button.")
    else:
        print("Updates will change text in HDPlayer but will not click Send. Add --send to transfer.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
