"""The acceptance driver serves its page and the vendored client with types browsers accept."""
import asyncio
import hashlib
import json
import tomllib

from aiohttp.test_utils import TestClient, TestServer

from scripts.stream_acceptance import FILES, REPO, VENDOR, network_address, page_app


def test_shipped_client_matches_python_sdk_pin():
    with (REPO / "pyproject.toml").open("rb") as project:
        source = tomllib.load(project)["tool"]["uv"]["sources"]["freemocap-sdk-stream"]
    manifest = json.loads((VENDOR / "freemocap-client.SOURCE").read_bytes())
    assert manifest["sdk_revision"] == source["rev"]
    assert manifest["bundle_sha256"] == hashlib.sha256((VENDOR / "freemocap-client.js").read_bytes()).hexdigest()


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


def test_candidate_bundle_is_served_only_by_its_test_page(tmp_path):
    candidate = tmp_path / "candidate.js"
    candidate.write_text("export const candidate = true;", encoding="utf-8")
    route = "/vendor/freemocap-client.js"

    async def scenario():
        async with TestClient(TestServer(page_app(candidate))) as client:
            response = await client.get(route)
            assert response.content_type == "text/javascript"
            assert await response.read() == candidate.read_bytes()
            assert (await client.get(route + ".map")).status == 404
        async with TestClient(TestServer(page_app())) as client:
            assert await (await client.get(route)).read() == FILES[route][0].read_bytes()

    asyncio.run(scenario())
