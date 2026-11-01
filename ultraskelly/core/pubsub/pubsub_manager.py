# ============================================================================
# PubSubTopicManager - Auto-discovers and manages all registered topics
# ============================================================================

import asyncio
import logging

from pydantic import BaseModel, Field, ConfigDict

from ultraskelly.core.pubsub.pubsub_abcs import PubSubTopicABC, MessageType, TopicSubscriptionQueue

logger = logging.getLogger(__name__)

class PubSubTopicManager(BaseModel):
    """
    Manager for pub/sub topics. Auto-instantiates all registered topic classes.

    Usage:
        manager = PubSubTopicManager.create()
        sub = manager.get_subscription(ProcessFrameNumberTopic)
        await manager.publish(ProcessFrameNumberTopic, message)
    """

    # Dict maps topic classes to their instances: {ProcessFrameNumberTopic: <instance>}
    topics: dict[type[PubSubTopicABC], PubSubTopicABC] = Field(default_factory=dict)

    model_config = ConfigDict(arbitrary_types_allowed=True)

    @classmethod
    def create(cls) -> "PubSubTopicManager":
        """Factory method: creates manager and auto-instantiates all registered topics."""
        manager = cls()
        for topic_cls in PubSubTopicABC.get_registered_topics():
            manager.topics[topic_cls] = topic_cls()
            logger.debug(f"Instantiated topic: {topic_cls.__name__}")
        return manager

    def get_subscription(self, topic_type: type[PubSubTopicABC]) -> TopicSubscriptionQueue:
        """
        Get a subscription queue for a topic.

        Args:
            topic_type: The topic CLASS (e.g., ProcessFrameNumberTopic)
        """
        if topic_type not in self.topics:
            raise ValueError(
                f"Unknown topic type: {topic_type.__name__}. "
                f"Available topics: {[t.__name__ for t in self.topics.keys()]}"
            )

        sub = self.topics[topic_type].get_subscription()
        logger.debug(
            f"Subscribed to topic {topic_type.__name__} "
            f"with {len(self.topics[topic_type].subscriptions)} subscriptions"
        )
        return sub

    async def publish(
        self,
        topic_type: type[PubSubTopicABC[MessageType]],
        message: MessageType
    ) -> None:
        """
        Publish a message to a topic.

        Args:
            topic_type: The topic CLASS to publish to
            message: The message to publish
        """
        if topic_type not in self.topics:
            raise ValueError(
                f"Unknown topic type: {topic_type.__name__}. "
                f"Available topics: {[t.__name__ for t in self.topics.keys()]}"
            )

        await self.topics[topic_type].publish(message)

    def close(self) -> None:
        """Close all topics in the manager."""
        logger.debug("Closing PubSubTopicManager...")
        for topic in self.topics.values():
            topic.close()
        self.topics.clear()
        logger.debug("PubSubTopicManager closed.")


PIPELINE_PUB_SUB_MANAGER: PubSubTopicManager | None = None


def get_or_create_pipeline_pubsub_manager() -> PubSubTopicManager:
    """Create/replace manager for a pipeline."""
    global PIPELINE_PUB_SUB_MANAGER

    if PIPELINE_PUB_SUB_MANAGER is None:
        PIPELINE_PUB_SUB_MANAGER = PubSubTopicManager.create()

    return PIPELINE_PUB_SUB_MANAGER