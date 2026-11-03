"""The acceptance driver serves its page and the vendored client with types browsers accept."""
import asyncio

from aiohttp.test_utils import TestClient, TestServer

from scripts.stream_acceptance import FILES, network_address, page_app


def test_page_and_vendored_client_are_served_as_html_and_javascript():
    async def scenario():
        async with TestClient(TestServer(page_app())) as client:
            served = {}
            for route in FILES:
                response = await client.get(route)
                served[route] = (response.status, response.content_type, len(await response.read()))
            return served

    served = asyncio.run(scenario())

    assert served["/"][:2] == (200, "text/html")
    assert served["/stream_acceptance.js"][:2] == (200, "text/javascript")
    assert served["/vendor/freemocap-client.js"][:2] == (200, "text/javascript")
    assert all(size > 0 for _, _, size in served.values())


def test_network_address_is_an_ipv4_address():
    assert len(network_address().split(".")) == 4
