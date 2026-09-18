from __future__ import annotations

import asyncio
import logging
import time
import uuid
from collections.abc import Callable, Iterable
from typing import Any

from forge_msgs import JointState

from forge_tool import ToolEndpointHandler, ToolEnvelope
from forge_tool._tool_message import ToolMessage
from forge_tool.dora import (
    DoraToolEndpointBinding,
    tool_envelope_to_message,
    tool_message_to_envelope,
)

from .config import MultiGroupRelativePosePolicyConfig, RelativePosePolicyConfig
from .endpoint_lease import EndpointLease
from .query import RELATIVE_DESCRIPTOR, RelativePoseQueryEndpoint
from .resolver import MultiGroupRelativePoseResolver, RelativePoseResolver

logger = logging.getLogger(__name__)


class RelativePosePolicyRunner:
    def __init__(
        self,
        node: Any,
        config: RelativePosePolicyConfig | MultiGroupRelativePosePolicyConfig,
        *,
        resolver: RelativePoseResolver | MultiGroupRelativePoseResolver | None = None,
        endpoint: RelativePoseQueryEndpoint | None = None,
        endpoint_instance_id: str | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.node = node
        if endpoint is not None and resolver is not None and endpoint.resolver is not resolver:
            raise ValueError("endpoint and resolver must refer to the same resolver instance")
        self.resolver = resolver or (endpoint.resolver if endpoint is not None else None)
        if self.resolver is None:
            if isinstance(config, MultiGroupRelativePosePolicyConfig):
                self.resolver = MultiGroupRelativePoseResolver(config)
            else:
                self.resolver = RelativePoseResolver(config)
        self.endpoint = endpoint or RelativePoseQueryEndpoint(self.resolver)
        self.endpoint_instance_id = endpoint_instance_id or str(uuid.uuid4())
        self.clock = clock
        self._async = asyncio.Runner()
        self.handler = ToolEndpointHandler(
            RELATIVE_DESCRIPTOR,
            endpoint_instance_id=self.endpoint_instance_id,
            operations={"resolve": self.endpoint},
        )
        self.binding = DoraToolEndpointBinding(self.handler, event_sink=self._send_batch)
        self.lease = EndpointLease(RELATIVE_DESCRIPTOR, self.endpoint_instance_id)

    async def _send_batch(self, batch: object) -> None:
        self.node.send_output("tool_out", batch)

    def _send_envelope(self, envelope: ToolEnvelope) -> None:
        self.node.send_output("tool_out", tool_envelope_to_message(envelope).to_arrow())

    def _drive_lease(self) -> None:
        request = self.lease.poll(self.clock())
        if request is not None:
            self._send_envelope(request)

    def _handle_tool(self, value: object) -> None:
        envelope = tool_message_to_envelope(ToolMessage.from_arrow(value))
        if envelope.message_type == "endpoint.registry.response":
            self.lease.acknowledge(envelope, self.clock())
            return
        batches = self._async.run(self.binding.dispatch_input(value))
        for batch in batches:
            self.node.send_output("tool_out", batch)

    def process_event(self, event: dict[str, Any]) -> int | None:
        event_type = event.get("type")
        if event_type == "STOP":
            unregister = self.lease.stop()
            if unregister is not None:
                self._send_envelope(unregister)
            self._async.close()
            return 0
        if event_type == "ERROR":
            logger.error("Dora node error: %s", event.get("error", "unknown"))
            self._async.close()
            return 1
        if event_type != "INPUT":
            return None

        self._drive_lease()
        try:
            if event.get("id") == "joint_state":
                self.resolver.update_joint_state(JointState.from_arrow(event["value"]))
            elif event.get("id") == "tool_in":
                self._handle_tool(event["value"])
        except (TypeError, ValueError) as exc:
            logger.warning("ignored invalid relative policy input: %s", exc)
        self._drive_lease()
        return None

    def run(self, events: Iterable[dict[str, Any]]) -> int:
        self._drive_lease()
        for event in events:
            result = self.process_event(event)
            if result is not None:
                return result
        return 0


def run_relative_policy(
    config: RelativePosePolicyConfig | MultiGroupRelativePosePolicyConfig,
) -> int:
    from dora import Node

    node = Node()
    return RelativePosePolicyRunner(node, config).run(node)


__all__ = ["RelativePosePolicyRunner", "run_relative_policy"]
