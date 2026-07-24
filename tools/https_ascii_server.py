#!/usr/bin/env python3
"""Serve editable HDPlayer JSON data over HTTP or HTTPS on a local network.

Examples:
  # Start a simple HTTP server (no certificate or OpenSSL needed).
  py tools/https_ascii_server.py

  # Open http://localhost:8080/edit to change the message.

  # Use HTTPS only when it is specifically required.
  py tools/https_ascii_server.py --https --generate-self-signed

The generated certificate is self-signed. Browsers and clients must explicitly
trust it, or use curl's --insecure option for a local test:
  curl --insecure https://LAN-IP-PRINTED-AT-STARTUP:8080/

HTTPS always needs a certificate and private key. OpenSSL is needed only to
make a local development certificate with --generate-self-signed; it is not
needed when --cert-file and --key-file point to an existing PEM certificate and
key. Plain HTTP, the default, does not need either.

Windows: install OpenSSL for the automatic development-certificate setup:
  winget install --exact --id FireDaemon.OpenSSL --source winget
Then close and reopen PowerShell, and check: openssl version

If OpenSSL is installed but still cannot be found, add the folder containing
openssl.exe to the Windows Path environment variable, then reopen PowerShell.
"""

from __future__ import annotations

import argparse
import html
import ipaddress
import json
import os
import shutil
import socket
import ssl
import subprocess
import sys
import threading
import urllib.parse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Type


DEFAULT_MESSAGE = "HELLO FROM THE HTTPS ASCII SERVER\n"
DEFAULT_CERT_FILE = Path("build/https_ascii_server/cert.pem")
DEFAULT_KEY_FILE = Path("build/https_ascii_server/key.pem")


def ascii_message(value: str) -> bytes:
    """Return *value* as a newline-terminated ASCII response body."""
    if not value.endswith("\n"):
        value += "\n"
    try:
        return value.encode("ascii")
    except UnicodeEncodeError as error:
        raise argparse.ArgumentTypeError("--message must contain ASCII characters only") from error


class TextStore:
    """Thread-safe, in-memory response text shared by all requests."""

    def __init__(self, message: bytes) -> None:
        self._message = message
        self._lock = threading.Lock()

    def get(self) -> bytes:
        with self._lock:
            return self._message

    def set(self, value: str) -> None:
        message = ascii_message(value)
        with self._lock:
            self._message = message


def make_handler(text_store: TextStore) -> Type[BaseHTTPRequestHandler]:
    """Create a handler for the HDPlayer JSON endpoint and browser editor."""

    class AsciiHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_GET(self) -> None:  # noqa: N802 - HTTP method name required by BaseHTTPRequestHandler
            path = urllib.parse.urlsplit(self.path).path
            if path == "/":
                self.send_json()
            elif path == "/text":
                self.send_ascii_text()
            elif path == "/json":
                self.send_json()
            elif path == "/edit":
                self.send_editor()
            elif path == "/favicon.ico":
                self.send_response(HTTPStatus.NO_CONTENT)
                self.send_header("Content-Length", "0")
                self.send_header("Cache-Control", "max-age=86400")
                self.end_headers()
            else:
                self.send_error(HTTPStatus.NOT_FOUND, "Not found")

        def do_POST(self) -> None:  # noqa: N802 - HTTP method name required by BaseHTTPRequestHandler
            path = urllib.parse.urlsplit(self.path).path
            if path != "/edit":
                self.send_error(HTTPStatus.METHOD_NOT_ALLOWED, "Only POST /edit is supported")
                return
            try:
                content_length = int(self.headers.get("Content-Length", ""))
            except ValueError:
                self.send_error(HTTPStatus.LENGTH_REQUIRED, "A Content-Length header is required")
                return
            if not 0 <= content_length <= 8192:
                self.send_error(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "Message is limited to 8192 bytes")
                return
            try:
                form = urllib.parse.parse_qs(
                    self.rfile.read(content_length).decode("utf-8"), keep_blank_values=True
                )
            except UnicodeDecodeError:
                self.send_error(HTTPStatus.BAD_REQUEST, "The form must be UTF-8")
                return
            try:
                text_store.set(form.get("message", [""])[0])
            except argparse.ArgumentTypeError as error:
                self.send_error(HTTPStatus.BAD_REQUEST, str(error))
                return
            self.send_response(HTTPStatus.SEE_OTHER)
            self.send_header("Location", "/edit")
            self.send_header("Content-Length", "0")
            self.end_headers()

        def send_ascii_text(self) -> None:
            message = text_store.get()
            self.send_content(message, "text/plain; charset=us-ascii")

        def send_json(self) -> None:
            message = text_store.get().decode("ascii").rstrip("\n")
            body = json.dumps({"message": message}, ensure_ascii=True, separators=(",", ":")).encode("ascii")
            self.send_content(body, "application/json; charset=us-ascii")

        def send_content(self, body: bytes, content_type: str) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def send_editor(self) -> None:
            value = html.escape(text_store.get().decode("ascii"))
            page = f"""<!doctype html>
<html lang="en"><meta charset="utf-8"><title>HDPlayer data</title>
<body><h1>HDPlayer message</h1>
<p>The HDPlayer endpoint (<code>/</code>) returns <code>{{&quot;message&quot;:&quot;...&quot;}}</code>. Changes are kept until the server stops.</p>
<form method="post" action="/edit"><textarea name="message" rows="8" cols="60">{value}</textarea>
<p><button type="submit">Save ASCII text</button></p></form></body></html>""".encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(page)

        def do_PUT(self) -> None:  # noqa: N802 - HTTP method name required by BaseHTTPRequestHandler
            self.send_error(HTTPStatus.METHOD_NOT_ALLOWED, "Only GET / and GET or POST /edit are supported")

        do_DELETE = do_PUT
        do_PATCH = do_PUT

        def log_message(self, format: str, *args: object) -> None:
            print(f"{self.client_address[0]} - {format % args}")

    return AsciiHandler


def certificate_subject_alternative_names(hosts: list[str]) -> str:
    """Return an OpenSSL SAN setting for IP addresses and DNS names."""
    alternatives = []
    for host in hosts:
        try:
            ipaddress.ip_address(host)
        except ValueError:
            alternatives.append(f"DNS:{host}")
        else:
            alternatives.append(f"IP:{host}")
    return ",".join(alternatives)


def find_openssl() -> str | None:
    """Find OpenSSL on PATH or in the normal FireDaemon Windows location."""
    executable = shutil.which("openssl")
    if executable:
        return executable
    if sys.platform != "win32":
        return None

    program_files = [
        os.environ.get("ProgramFiles"),
        os.environ.get("ProgramW6432"),
        os.environ.get("ProgramFiles(x86)"),
    ]
    for directory in dict.fromkeys(path for path in program_files if path):
        for candidate in Path(directory).glob("FireDaemon OpenSSL*/bin/openssl.exe"):
            if candidate.is_file():
                return str(candidate)
    return None


def generate_self_signed_certificate(cert_file: Path, key_file: Path, hosts: list[str]) -> None:
    """Generate a development-only certificate using the locally installed OpenSSL."""
    openssl = find_openssl()
    if openssl is None:
        raise RuntimeError(
            "OpenSSL is required for --generate-self-signed. Install it with: "
            "winget install --exact --id FireDaemon.OpenSSL --source winget"
        )
    if not hosts:
        raise RuntimeError("No certificate host is available")

    common_name = hosts[0]
    cert_file.parent.mkdir(parents=True, exist_ok=True)
    command = [
        openssl,
        "req",
        "-x509",
        "-newkey",
        "rsa:2048",
        "-sha256",
        "-nodes",
        "-keyout",
        str(key_file),
        "-out",
        str(cert_file),
        "-days",
        "365",
        "-subj",
        f"/CN={common_name}",
        "-addext",
        f"subjectAltName={certificate_subject_alternative_names(hosts)}",
    ]
    try:
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    except subprocess.CalledProcessError as error:
        detail = error.stderr.strip()
        raise RuntimeError(f"OpenSSL could not generate the certificate: {detail}") from error
    print(f"Generated development certificate: {cert_file}")


def local_network_ipv4_addresses() -> list[str]:
    """Return likely LAN IPv4 addresses, with the default-route address first."""
    addresses: list[str] = []
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            # No packet is sent.  Connecting a UDP socket asks the OS which
            # local address it would use for the default route.
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
        if not ipaddress.ip_address(address).is_loopback and not ipaddress.ip_address(address).is_unspecified
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve an HDPlayer JSON response over HTTP or HTTPS.")
    parser.add_argument("--host", default="0.0.0.0", help="Interface to listen on (default: all IPv4 interfaces)")
    parser.add_argument("--port", default=8080, type=int, help="HTTPS port (default: 8080)")
    parser.add_argument("--message", default=DEFAULT_MESSAGE, help="ASCII message to return in the JSON response")
    parser.add_argument("--cert-file", type=Path, default=DEFAULT_CERT_FILE, help="PEM certificate path")
    parser.add_argument("--key-file", type=Path, default=DEFAULT_KEY_FILE, help="PEM private-key path")
    parser.add_argument("--https", action="store_true", help="serve HTTPS instead of the default HTTP")
    parser.add_argument(
        "--generate-self-signed",
        action="store_true",
        help="generate a development HTTPS certificate before starting (implies --https)",
    )
    parser.add_argument(
        "--certificate-host",
        help="optional extra IP address or DNS name to include in a generated certificate",
    )
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.certificate_host and not args.generate_self_signed:
        parser.error("--certificate-host requires --generate-self-signed")
    args.message = ascii_message(args.message)
    return args


def main() -> int:
    args = parse_args()
    if args.generate_self_signed:
        args.https = True
    addresses = local_network_ipv4_addresses() if args.host == "0.0.0.0" else []
    if args.generate_self_signed:
        try:
            certificate_hosts = ["localhost", "127.0.0.1", *addresses]
            if args.certificate_host:
                certificate_hosts.append(args.certificate_host)
            generate_self_signed_certificate(args.cert_file, args.key_file, list(dict.fromkeys(certificate_hosts)))
        except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
            print(f"Unable to generate certificate: {error}", file=sys.stderr)
            return 1

    context: ssl.SSLContext | None = None
    if args.https:
        if not args.cert_file.is_file() or not args.key_file.is_file():
            print(
                "Certificate or private key is missing. Use --generate-self-signed for local testing, "
                "or pass --cert-file and --key-file.",
                file=sys.stderr,
            )
            return 1
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        try:
            context.load_cert_chain(certfile=args.cert_file, keyfile=args.key_file)
        except ssl.SSLError as error:
            print(f"Unable to load TLS certificate: {error}", file=sys.stderr)
            return 1

    server = ThreadingHTTPServer((args.host, args.port), make_handler(TextStore(args.message)))
    if context is not None:
        server.socket = context.wrap_socket(server.socket, server_side=True)
    scheme = "https" if args.https else "http"
    if args.host == "0.0.0.0":
        if addresses:
            print(f"Serving {scheme.upper()} HDPlayer JSON on:")
            for address in addresses:
                print(f"  {scheme}://{address}:{args.port}/")
            print("Plain ASCII text (for testing):")
            for address in addresses:
                print(f"  {scheme}://{address}:{args.port}/text")
            print("Edit text from this network:")
            for address in addresses:
                print(f"  {scheme}://{address}:{args.port}/edit")
        else:
            print(f"Serving {scheme.upper()} HDPlayer JSON on all IPv4 interfaces, port {args.port}.")
        print(f"Edit from this PC: {scheme}://localhost:{args.port}/edit")
    else:
        print(f"Serving {scheme.upper()} HDPlayer JSON at {scheme}://{args.host}:{args.port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping server.")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
