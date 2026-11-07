from abc import ABC, abstractmethod
import asyncio
from typing import Protocol, runtime_checkable

import numpy as np
from pydantic import BaseModel, SkipValidation, Field, ConfigDict

from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager


class NodeParams(BaseModel):
    """Base class for all node parameters."""
    pass


class Node(BaseModel, ABC):
    """Abstract base class for all nodes."""
    model_config = ConfigDict(arbitrary_types_allowed=True)

    pubsub: PubSubTopicManager
    params: NodeParams
    stop_event: SkipValidation[asyncio.Event] = Field(default_factory=asyncio.Event, exclude=True)

    @abstractmethod
    async def run(self) -> None:
        """Main node execution loop."""
        pass

    def stop(self) -> None:
        """Stop the node."""
        self.stop_event.set()


@runtime_checkable
class SensorNode(Protocol):
    """Protocol for nodes that produce sensor data."""
    async def publish_data(self, data: object) -> None:
        """Publish sensor data."""
        ...


@runtime_checkable
class ActuatorNode(Protocol):
    """Protocol for nodes that consume data to control actuators."""
    async def process_command(self, command: object) -> None:
        """Process actuator command."""
        ...


class DetectorNode(Node):
    """Base class for detector nodes that process images."""

    @abstractmethod
    def detect(self, image: np.ndarray) -> tuple[int | None, int | None, float | None]:
        """Detect target in image."""
        pass