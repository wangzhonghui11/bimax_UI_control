"""
SSH 远程命令执行客户端（独立类）
- 密码登录
- 可复用连接
- 返回 stdout/stderr/exit_code
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Optional

import paramiko


@dataclass
class SSHResult:
    ok: bool
    exit_code: int
    stdout: str
    stderr: str


class SSHCommandClient:
    def __init__(
        self,
        host: str,
        username: str,
        password: str,
        port: int = 22,
        timeout: float = 5.0,
    ):
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        self.timeout = timeout

        self._client: Optional[paramiko.SSHClient] = None
        self._lock = threading.Lock()

    def connect(self) -> None:
        with self._lock:
            if self._client is not None:
                return

            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(
                hostname=self.host,
                port=self.port,
                username=self.username,
                password=self.password,
                timeout=self.timeout,
                banner_timeout=self.timeout,
                auth_timeout=self.timeout,
            )
            self._client = client

    def close(self) -> None:
        with self._lock:
            if self._client:
                self._client.close()
            self._client = None

    def exec(self, command: str, timeout: Optional[float] = None) -> SSHResult:
        self.connect()
        assert self._client is not None

        to = self.timeout if timeout is None else timeout

        with self._lock:
            _, stdout, stderr = self._client.exec_command(command, timeout=to)
            out = stdout.read().decode("utf-8", errors="replace")
            err = stderr.read().decode("utf-8", errors="replace")
            exit_code = stdout.channel.recv_exit_status()

        return SSHResult(ok=(exit_code == 0), exit_code=exit_code, stdout=out, stderr=err)