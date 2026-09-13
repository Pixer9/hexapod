"""Synchronous HX1 link pump.

This layer composes the raw-byte transport, line framer, and transport-
independent client core. It deliberately owns no background thread, reconnect
loop, heartbeat scheduler, target scheduler, or robot lifecycle policy.

The caller decides when to build/send commands and when to poll the link.
"""

from __future__ import annotations

from .client import HX1ClientCore
from .messages import HX1Inbound, HX1MessageError, HX1Outbound
from .protocol import HX1LineFramer, HX1ProtocolError
from .transport import HX1Transport, HX1TransportError


class HX1LinkError(RuntimeError):
    """The synchronous HX1 link boundary was used incorrectly or failed."""


class HX1Link:
    """Compose one HX1 client core with one raw-byte transport."""

    def __init__(
        self,
        client: HX1ClientCore,
        transport: HX1Transport,
        *,
        framer: HX1LineFramer | None = None,
    ):
        if not isinstance(client, HX1ClientCore):
            raise TypeError("client must be an HX1ClientCore")
        if not isinstance(transport, HX1Transport):
            raise TypeError("transport must satisfy HX1Transport")

        self.client = client
        self.transport = transport
        self.framer = framer if framer is not None else HX1LineFramer()

        self.protocol_errors = 0
        self.message_errors = 0

    @property
    def is_open(self) -> bool:
        return self.transport.is_open

    @property
    def framing_errors(self) -> int:
        return self.framer.framing_errors

    def open(self) -> None:
        """Open the byte transport without sending any HX1 command.

        A newly opened physical/logical connection starts without local session
        authority. Repeated calls while already open are intentionally
        idempotent and do not destroy an active negotiated session.
        """
        if self.transport.is_open:
            return

        self._reset_connection_state(reset_counters=True)

        try:
            self.transport.open()
        except HX1TransportError:
            self.client.reset_link_state()
            raise

    def close(self) -> None:
        """Close the byte transport and forget all local session authority."""
        try:
            self.transport.close()
        finally:
            self._reset_connection_state(reset_counters=False)

    def send(self, outbound: HX1Outbound) -> int:
        """Send one already-built HX1 outbound frame synchronously."""
        if not isinstance(outbound, HX1Outbound):
            raise TypeError("outbound must be an HX1Outbound")

        try:
            accepted = self.transport.write(outbound.frame)
        except HX1TransportError:
            self._invalidate_after_io_failure()
            raise

        if accepted != len(outbound.frame):
            self._invalidate_after_io_failure()
            raise HX1LinkError("HX1 transport accepted an incomplete outbound frame")

        return accepted

    def poll(self, max_bytes: int = 4096) -> tuple[HX1Inbound, ...]:
        """Perform one non-blocking inbound pump iteration.

        Exactly one raw transport read is attempted. Any complete lines produced
        from that byte chunk are parsed in order.

        Framing/protocol-corrupt or semantically malformed frames are dropped
        and counted. Session/profile negotiation errors are not swallowed:
        those remain explicit client errors for the caller to handle.
        """
        try:
            data = self.transport.read(max_bytes)
        except HX1TransportError:
            self._invalidate_after_io_failure()
            raise

        if not data:
            return ()

        raw_frames = self.framer.feed(data)
        messages: list[HX1Inbound] = []

        for raw_frame in raw_frames:
            try:
                message = self.client.accept_frame(raw_frame)
            except HX1ProtocolError:
                self.protocol_errors += 1
                continue
            except HX1MessageError:
                self.message_errors += 1
                continue

            messages.append(message)

        return tuple(messages)

    def _invalidate_after_io_failure(self) -> None:
        """Discard local authority after the byte link can no longer be trusted."""
        self.client.reset_link_state()
        self.framer.reset()

    def _reset_connection_state(
        self,
        *,
        reset_counters: bool,
    ) -> None:
        self.client.reset_link_state()
        self.framer.reset()

        if reset_counters:
            self.protocol_errors = 0
            self.message_errors = 0
