"""Hardware-free regression tests for the real robot message bus."""
import asyncio

import pytest

from ultraskelly.core.pubsub.bot_topics import TargetLocationMessage, TargetLocationTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager


def test_slow_subscriber_cannot_freeze_producer():
    async def scenario():
        topic = TargetLocationTopic()
        slow = topic.get_subscription()
        fast = topic.get_subscription()
        for i in range(1000):
            await asyncio.wait_for(topic.publish(TargetLocationMessage(x=i)), timeout=0.1)
            assert (await fast.get()).x == i
        assert slow.qsize() == 1
        assert (await slow.get()).x == 999
    asyncio.run(scenario())


def test_publication_history_is_bounded_without_consumer():
    async def scenario():
        topic = TargetLocationTopic()
        for i in range(1000):
            await topic.publish(TargetLocationMessage(x=i))
        assert topic.publication.qsize() <= 1
    asyncio.run(scenario())


def test_wrong_message_type_rejected():
    async def scenario():
        with pytest.raises(TypeError):
            await TargetLocationTopic().publish(object())
    asyncio.run(scenario())


def test_manager_rejects_unknown_topic():
    with pytest.raises(ValueError, match="Unknown topic"):
        PubSubTopicManager().get_subscription(TargetLocationTopic)


def test_close_releases_queued_messages():
    async def scenario():
        topic = TargetLocationTopic()
        sub = topic.get_subscription()
        await topic.publish(TargetLocationMessage(x=1))
        topic.close()
        assert not topic.subscriptions
        assert sub.empty()
        assert topic.publication.empty()
    asyncio.run(scenario())
