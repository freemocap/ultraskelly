import asyncio
import logging
from typing import TypeVar, Generic, Callable, ClassVar

from pydantic import BaseModel, Field, ConfigDict, create_model, SkipValidation

logger = logging.getLogger(__name__)
TopicPublicationQueue = SkipValidation[asyncio.Queue]
TopicSubscriptionQueue = SkipValidation[asyncio.Queue]


class TopicMessageABC(BaseModel):
    """Base for all message data models."""
    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        frozen=True
    )


# MessageType represents "any type that is a subclass of TopicMessageABC"
# This allows each topic to be strongly typed with its specific message type
# e.g., PubSubTopicABC[TestMessage] means "a topic that handles TestMessage"
MessageType = TypeVar('MessageType', bound=TopicMessageABC)


class PubSubTopicABC(BaseModel, Generic[MessageType]):
    """Base pub/sub topic with default behavior."""
    topic_registry: ClassVar[set[type['PubSubTopicABC']]] = set()

    message_type: type[TopicMessageABC]
    publication: TopicPublicationQueue = Field(default_factory=lambda: asyncio.Queue(maxsize=1))
    subscriptions: list[TopicSubscriptionQueue] = Field(default_factory=list)

    model_config = ConfigDict(arbitrary_types_allowed=True)

    def __init_subclass__(cls, **kwargs) -> None:
        """Auto-register when subclassed."""
        super().__init_subclass__(**kwargs)
        if not cls.__name__.startswith('_'):  # Skip private/base classes
            PubSubTopicABC.topic_registry.add(cls)

    @classmethod
    def get_registered_topics(cls) -> set[type['PubSubTopicABC']]:
        return cls.topic_registry.copy()

    def get_subscription(self) -> TopicSubscriptionQueue:
        sub = asyncio.Queue(maxsize=1)
        self.subscriptions.append(sub)
        return sub

    async def publish(self, message: MessageType) -> None:
        if not isinstance(message, self.message_type):
            raise TypeError(f"Message must be of type {self.message_type.__name__}, got {type(message).__name__}")
        self._put_latest(self.publication, message)
        for sub in self.subscriptions:
            self._put_latest(sub, message)

    @staticmethod
    def _put_latest(queue: asyncio.Queue, message: TopicMessageABC) -> None:
        # State streams favor freshness over history. All access stays on one event loop.
        while not queue.empty():
            queue.get_nowait()
        queue.put_nowait(message)

    def close(self) -> None:
        for queue in [self.publication, *self.subscriptions]:
            while not queue.empty():
                queue.get_nowait()
        self.subscriptions.clear()


def create_topic(
        message_type: type[MessageType],
        publication_factory: Callable[[], TopicPublicationQueue] | None = None,
) -> type[PubSubTopicABC[MessageType]]:
    """
    Factory that creates a topic class for a message type.

    Uses Pydantic's create_model() to properly create the class.
    Returns a class (not instance) so it auto-registers via __init_subclass__.
    """
    topic_name = message_type.__name__.replace('Message', 'Topic')

    # Build field definitions for create_model
    # Format: field_name=(type, Field(...))
    field_definitions: dict[str, object] = {
        'message_type': (type[MessageType], message_type),
    }

    # Add custom publication factory if provided
    if publication_factory is not None:
        field_definitions['publication'] = (
            TopicPublicationQueue,
            Field(default_factory=publication_factory)
        )
    else:
        field_definitions['publication'] = (
            TopicPublicationQueue,
            Field(default_factory=lambda: asyncio.Queue(maxsize=1))
        )

    # Use Pydantic's create_model - the proper v2 way!
    topic_class = create_model(
        topic_name,
        __base__=PubSubTopicABC,
        __module__=message_type.__module__,
        **field_definitions
    )

    return topic_class
