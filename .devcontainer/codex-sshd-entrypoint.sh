#!/usr/bin/env bash
set -Eeuo pipefail

readonly CODEX_HOME_DIR=/home/vscode/.codex
readonly SSH_HOSTKEY_DIR=/etc/ssh/hostkeys
readonly SSH_AUTHORIZED_KEYS=/home/vscode/.ssh/authorized_keys

if [[ "$(id -u)" -ne 0 ]]; then
    echo "codex-sshd-entrypoint must run as container root" >&2
    exit 1
fi

install -d -m 0700 -o vscode -g vscode /home/vscode/.ssh
install -d -m 0700 -o vscode -g vscode "$CODEX_HOME_DIR"
install -d -m 0755 "$SSH_HOSTKEY_DIR"
install -d -m 0755 /run/sshd

if [[ ! -s "$SSH_AUTHORIZED_KEYS" ]]; then
    echo "Missing $SSH_AUTHORIZED_KEYS" >&2
    exit 1
fi

if [[ ! -s "$SSH_HOSTKEY_DIR/ssh_host_ed25519_key" ]]; then
    ssh-keygen -q -t ed25519 -N '' -f "$SSH_HOSTKEY_DIR/ssh_host_ed25519_key"
fi
chmod 0600 "$SSH_HOSTKEY_DIR/ssh_host_ed25519_key"
chmod 0644 "$SSH_HOSTKEY_DIR/ssh_host_ed25519_key.pub"

if [[ ! -e "$CODEX_HOME_DIR/config.toml" ]]; then
    cat > "$CODEX_HOME_DIR/config.toml" <<'EOF'
# Container-only Codex policy. The container is the outer safety boundary.
sandbox_mode = "danger-full-access"
approval_policy = "never"
EOF
    chown vscode:vscode "$CODEX_HOME_DIR/config.toml"
    chmod 0600 "$CODEX_HOME_DIR/config.toml"
fi

exec /usr/sbin/sshd -D -e -f /etc/ssh/sshd_config
